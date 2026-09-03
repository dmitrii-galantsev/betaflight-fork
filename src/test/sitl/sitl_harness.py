#!/usr/bin/env python3
"""SITL end-to-end harness: UDP RC + FDM feeds, MSP introspection, scenario runner.

Drives a betaflight_SITL binary the way a simulator and transmitter would:
  - RC channels over UDP :9004 (rc_packet: double timestamp + 16 x uint16)
  - FDM state over UDP :9003 (fdm_packet: 18 doubles; virtual-GPS mode puts
    lon/lat/alt in position_xyz and ENU velocity in velocity_xyz)
  - MSP over TCP :5761 for runtime state (modes, arming disable flags)
  - one-shot `--config <file>` runs to provision eeprom.bin per scenario

Scenarios exercise the flight plan / AUTOPILOT safety behaviour end to end:
mode wiring, rx-loss policies (DISABLE / CONTINUE / LAND) and geofence
(LAND / RTH). Requires a SITL binary built with USE_FLIGHT_PLAN.

Usage:
  sitl_harness.py --binary obj/main/betaflight_SITL.elf --scenario all
  sitl_harness.py --binary ... --scenario rx_continue -v
"""

import argparse
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid

MSP_STATUS = 101
MSP_RAW_GPS = 106
MSP_BOXIDS = 119
MSP_ACC_CALIBRATION = 205

TCP_PORT = 5761
RC_PORT = 9004
FDM_PORT = 9003
PWM_PORT = 9002

HOME_LAT = -27.5000000
HOME_LON = 153.0000000
HOME_ALT_M = 30.0
M_PER_DEG = 111319.49

# Box permanent IDs (msp_box.c)
BOX_ARM = 0
BOX_ALTHOLD = 3
BOX_POSHOLD = 11
BOX_FAILSAFE = 27
BOX_GPSRESCUE = 46
BOX_AUTOPILOT = 56

RC_MID = 1500
RC_LOW = 1000
RC_HIGH = 2000

VERBOSE = False
TELEMETRY_PORT = 9005  # ground-truth JSON fan-out for external visualisers, 0 disables


# --- simulated clock -------------------------------------------------------
# SITL derives its own time scale from the ratio of the harness's FDM packet timestamps to
# wall clock (sitl.c: simRate = deltaSim / wallDelta) and divides every internal
# sleep by it, so driving the sim clock faster than real time speeds the whole FC
# up with no firmware change. Note SITL only updates that ratio while
# deltaSim < 0.02, so the FDM step must stay strictly under 20ms.
#
# All scenario waits go through this clock, so they are expressed in simulated
# seconds and scale automatically with --speed. The step is fixed rather than
# measured, which also makes runs far more repeatable than wall-clock stepping.
FDM_STEP_S = 0.01          # 100Hz nominal pacing, strictly under SITL's 20ms gate
FDM_STEP_MAX_S = 0.0199    # at or above 20ms SITL stops updating simRate at all
SPEED = 1.0                # wall-clock divisor, set from --speed


class SimClock:
    """Virtual time, advanced by the FDM feed - the only thread that steps physics."""

    def __init__(self):
        self._t = 0.0
        self._cv = threading.Condition()

    def advance(self, dt):
        with self._cv:
            self._t += dt
            self._cv.notify_all()

    def now(self):
        with self._cv:
            return self._t

    def sleep(self, seconds):
        """Block until `seconds` of simulated time have passed."""
        target = self.now() + seconds
        with self._cv:
            while self._t < target:
                # bounded wait so a stalled FDM feed cannot deadlock a scenario
                if not self._cv.wait(timeout=5.0):
                    if self._t < target:
                        # Returning here would leave simulated time short of the target.
                        # wait_for() derives its deadline from the same clock, so it would
                        # spin forever and hang the whole suite. Fail the scenario instead:
                        # run_scenario() catches TimeoutError and records it.
                        raise TimeoutError(
                            f"simulated clock stalled at t={self._t:.3f}s waiting for "
                            f"t={target:.3f}s (FDM feed stopped advancing time)"
                        )

    def deadline(self, seconds):
        return self.now() + seconds


CLOCK = SimClock()


def sim_now():
    return CLOCK.now()


def sim_sleep(seconds):
    CLOCK.sleep(seconds)


def log(msg):
    print(f"[harness] {msg}", flush=True)


def debug(msg):
    if VERBOSE:
        log(msg)


class RcFeed(threading.Thread):
    """50 Hz rc_packet stream. Stop the stream to simulate RX loss."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.channels = [RC_MID, RC_MID, RC_LOW, RC_MID] + [RC_LOW] * 12  # AERT + AUX
        self.streaming = True
        self.running = True
        self.t0 = 0.0

    def set(self, index, value):
        self.channels[index] = value

    def run(self):
        while self.running:
            if self.streaming:
                pkt = struct.pack("<d16H", CLOCK.now() - self.t0, *self.channels)
                self.sock.sendto(pkt, ("127.0.0.1", RC_PORT))
            time.sleep(max(0.0, 0.02 / SPEED))

    def stop_stream(self):
        self.streaming = False

    def shutdown(self):
        self.running = False


class MotorFeed(threading.Thread):
    """Listens for SITL's normalised motor outputs (servo_packet on UDP 9002)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", PWM_PORT))
        self.sock.settimeout(0.2)
        self.motors = [0.0, 0.0, 0.0, 0.0]
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(64)
                if len(data) >= 16:
                    self.motors = list(struct.unpack("<4f", data[:16]))
            except socket.timeout:
                pass
            except OSError:
                break  # socket closed during shutdown

    def shutdown(self):
        # release the port 9002 bind before the next scenario constructs its feed
        self.running = False
        if self.is_alive():
            self.join(timeout=1.0)
        self.sock.close()


GRAVITY = 9.80665
HOVER_THRUST = 0.30        # just above the FC hoverThrottle default (1275 -> 0.275)
RATE_GAIN = 12.0           # rad/s of body rate per unit differential thrust
RATE_TAU = 0.08            # attitude response time constant, seconds
# Low-drag plant: the nav velocity loop (ap_velocity_*) enforces cruise speed,
# so the drag no longer needs to mask integrator overshoot. base_config sets
# ap_velocity_drag_coeff to match this plant (atan(K_DRAG*v/g) over 5-7.5 m/s).
K_DRAG = 0.8               # 1/s, linear drag: 22 deg of tilt holds ~5 m/s
# Vertical axis. The horizontal axis is a force model (dv/dt = g*tan(tilt) - drag);
# the vertical axis was a *velocity* source, relaxing toward
# VERT_V_GAIN*(thrust - HOVER_THRUST) with a 0.3 s lag. That bounds sink at
# VERT_V_GAIN*HOVER_THRUST = 1.8 m/s whatever the FC commands, so a throttle-floor
# free-fall renders as a benign 1.8 m/s descent and a measured "descent rate" is as
# much plant gain as controller behaviour. Model thrust as a force instead,
# normalised so HOVER_THRUST exactly cancels gravity: zero throttle is a true -1 g.
THRUST_TAU = 0.06          # motor/prop spin-up lag, seconds
V_TERM_FALL = 15.0         # unpowered terminal velocity, m/s (sets the quadratic drag)
K_DRAG_V = GRAVITY / (V_TERM_FALL ** 2)


class MotionModel:
    """Crude quad kinematics: motor outputs -> body rates -> attitude -> motion.

    Differential thrust maps to first-order body-rate targets (quad-X, BF motor
    order M1=RR M2=FR M3=RL M4=FL), collective maps to thrust along body Z.
    Just enough plant for Betaflight's real rate/angle/position loops to close
    around; not a physics simulation.
    """

    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]   # ENU metres relative to home (ground = 0 up)
        self.vel = [0.0, 0.0, 0.0]   # ENU m/s
        self.accel = [0.0, 0.0, 0.0]  # ENU m/s^2 (world frame, for the acc feed)
        self.roll = 0.0              # rad, right positive
        self.pitch = 0.0             # rad, nose-down positive (BF command convention)
        self.yaw = 0.0               # rad, compass (CW from north) positive
        self.rates = [0.0, 0.0, 0.0]
        self.impact_ticks = 0
        self.thrust = 0.0            # lagged collective
        self.touchdown_speed = 0.0   # hardest ground contact of the run, m/s

    def on_ground(self):
        return self.pos[2] <= 0.001

    def step(self, dt, m):
        # First-order motor/prop lag. The velocity-source model folded this into
        # its 0.3 s velocity lag; with a force model it belongs on the thrust.
        self.thrust += (sum(m) / 4.0 - self.thrust) * min(1.0, dt / THRUST_TAU)
        thrust = self.thrust

        if self.on_ground() and thrust < HOVER_THRUST * 0.8:
            self.pos[2] = 0.0
            self.vel = [0.0, 0.0, 0.0]
            # touchdown impact: a short accelerometer spike, as a real landing
            # produces, so the FC's jerk-based disarmOnImpact can trigger
            self.accel = [0.0, 0.0, 60.0] if self.impact_ticks > 0 else [0.0, 0.0, 0.0]
            self.impact_ticks = max(0, self.impact_ticks - 1)
            self.rates = [0.0, 0.0, 0.0]
            return

        right = m[0] + m[1]   # M1 RR + M2 FR
        left = m[2] + m[3]    # M3 RL + M4 FL
        rear = m[0] + m[2]
        front = m[1] + m[3]
        # BF quad-X defaults: M1/M4 spin CW, M2/M3 CCW; reaction torque yaws
        # the frame opposite the prop direction.
        ccw = m[1] + m[2]
        cw = m[0] + m[3]

        target = [
            RATE_GAIN * (left - right) / 2.0,   # roll right
            RATE_GAIN * (rear - front) / 2.0,   # nose down (BF mixer: +pitch = rear up)
            RATE_GAIN * (ccw - cw) / 2.0,       # yaw CW (compass positive)
        ]
        for i in range(3):
            self.rates[i] += (target[i] - self.rates[i]) * min(1.0, dt / RATE_TAU)
        self.roll += self.rates[0] * dt
        self.pitch += self.rates[1] * dt
        self.yaw += self.rates[2] * dt
        self.roll = max(-1.2, min(1.2, self.roll))
        self.pitch = max(-1.2, min(1.2, self.pitch))

        # Horizontal: thrust-vector acceleration with linear drag
        # (dv/dt = g*tan(tilt) - K_DRAG*v). Vertical: first-order response
        # toward the thrust-above-hover terminal rate, the plant shape
        # alt-hold is tuned for; tilt reduces the vertical thrust component
        # (the FC compensates with 1/cos(tilt); the plant must charge for it
        # or that boost becomes a climb bias during fast forward flight).
        a_fwd = GRAVITY * math.tan(self.pitch)
        a_right = GRAVITY * math.tan(self.roll)
        sin_y, cos_y = math.sin(self.yaw), math.cos(self.yaw)
        acc_h = [
            a_fwd * sin_y + a_right * cos_y - K_DRAG * self.vel[0],  # east
            a_fwd * cos_y - a_right * sin_y - K_DRAG * self.vel[1],  # north
        ]
        for i in range(2):
            self.accel[i] = acc_h[i]
            self.vel[i] += acc_h[i] * dt
            self.pos[i] += self.vel[i] * dt

        # Vertical: force model. Thrust is normalised so HOVER_THRUST cancels
        # gravity exactly, giving a 1/HOVER_THRUST : 1 thrust-to-weight ratio and,
        # at zero throttle, a true -1 g free-fall bounded only by drag. Tilt
        # reduces the vertical component (the FC compensates with 1/cos(tilt); the
        # plant must charge for it or that boost becomes a climb bias in fast
        # forward flight).
        tilt = math.cos(self.pitch) * math.cos(self.roll)
        a_z = (GRAVITY * (thrust * tilt / HOVER_THRUST - 1.0)
               - K_DRAG_V * self.vel[2] * abs(self.vel[2]))
        self.accel[2] = a_z
        self.vel[2] += a_z * dt
        self.pos[2] += self.vel[2] * dt

        if self.pos[2] < 0.0:
            self.touchdown_speed = max(self.touchdown_speed, -self.vel[2])
            self.pos[2] = 0.0
            self.vel = [0.0, 0.0, 0.0]
            self.accel = [0.0, 0.0, 0.0]
            self.rates = [0.0, 0.0, 0.0]
            self.roll = self.pitch = 0.0
            self.impact_ticks = 4


def quat_from_euler_bf(roll, pitch, yaw):
    """Body->world quaternion in Betaflight's internal NWU frames from the
    model conventions (roll right+, pitch nose-down+, yaw compass CW+):
    NWU yaw is CCW-positive, pitch and roll map directly."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(-yaw / 2), math.sin(-yaw / 2)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def quat_conj_x180(q):
    """Similarity transform by 180 deg about x: what the gazebo plugin applies
    to its quaternion, and what the FC's bridge undoes on receive."""
    return (q[0], q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate_inv(q, v):
    """Rotate world vector v into the body frame (q is body->world)."""
    qc = (q[0], -q[1], -q[2], -q[3])
    p = quat_mul(quat_mul(qc, (0.0, *v)), q)
    return (p[1], p[2], p[3])


K_RZ_NEG90 = (math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5))


class FdmFeed(threading.Thread):
    """50 Hz fdm_packet stream driven by the motion model.

    Emits in the Gazebo-bridge conventions the default SITL build expects:
    quaternion pre-multiplied by Rz(-90deg) (the FC re-applies Rz(+90deg)),
    gyro in the plugin sensor frame (pitch and yaw negated from the model's
    nose-down/compass-CW conventions), and GPS lat/lon mirrored around the
    first packet's origin (the FC un-mirrors).
    """

    def __init__(self, motors=None, initial_yaw_deg=0.0, status=None):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.model = MotionModel()
        self.model.yaw = math.radians(initial_yaw_deg)
        self.motors = motors
        self.status = status
        self.sid = uuid.uuid4().hex[:8]  # telemetry session id: lets visualisers detect restarts
        self.running = True
        self.gps_valid = True     # False emits out-of-range lat/lon: the FC's GPS goes dark
        self.history = []         # (t, east, north, up, ve, vn, vu, heading_deg) at ~10 Hz
        self._hist_lock = threading.Lock()
        self._hist_decim = 0
        self.t0 = 0.0

    def move_east(self, metres):
        self.model.pos[0] += metres

    def distance_from_home(self):
        return math.hypot(self.model.pos[0], self.model.pos[1])

    def distance_to_wp(self, east_m, north_m):
        return math.hypot(self.model.pos[0] - east_m, self.model.pos[1] - north_m)

    def heading_deg(self):
        return math.degrees(self.model.yaw) % 360.0

    def snapshot_history(self):
        with self._hist_lock:
            return list(self.history)

    def max_altitude(self):
        return max((s[3] for s in self.snapshot_history()), default=0.0)

    def time_to_home(self, radius_m=10.0, after_t=0.0):
        """First recorded time the craft is within radius_m of home, after after_t."""
        for s in self.snapshot_history():
            if s[0] > after_t and math.hypot(s[1], s[2]) < radius_m:
                return s[0]
        return None

    def touchdown(self, after_t=0.0):
        """(t, east, north) of the first on-ground sample following airborne flight."""
        airborne = False
        for s in self.snapshot_history():
            if s[0] < after_t:
                continue
            if s[3] > 1.0:
                airborne = True
            elif airborne and s[3] <= 0.01:
                return (s[0], s[1], s[2])
        return None

    def max_distance_from_home(self, after_t=0.0):
        return max((math.hypot(s[1], s[2]) for s in self.snapshot_history() if s[0] >= after_t), default=0.0)

    def max_sink(self, after_t=0.0):
        """Fastest recorded descent, m/s positive down."""
        return max((-s[6] for s in self.snapshot_history() if s[0] >= after_t), default=0.0)

    def now_t(self):
        return CLOCK.now() - self.t0

    def run(self):
        last_wall = time.monotonic()
        while self.running:
            # Timestamp from measured wall time * SPEED rather than a fixed step.
            # SITL infers its entire time scale from deltaSim/wallDelta, so a fixed
            # step turns that ratio into a measurement of this thread's sleep jitter.
            # Deriving it from the same wall delta SITL measures makes the ratio
            # exactly SPEED, with no jitter and no drift between the two clocks.
            wall = time.monotonic()
            dt = min(FDM_STEP_MAX_S, max(0.0002, (wall - last_wall) * SPEED))
            last_wall = wall
            CLOCK.advance(dt)
            now = CLOCK.now()
            m = self.motors.motors if self.motors else [0.0] * 4
            self.model.step(dt, m)

            self._hist_decim += 1
            if self._hist_decim >= 10:  # ~10 Hz of the 100 Hz loop
                self._hist_decim = 0
                with self._hist_lock:
                    self.history.append((now - self.t0,
                                         self.model.pos[0], self.model.pos[1], self.model.pos[2],
                                         self.model.vel[0], self.model.vel[1], self.model.vel[2],
                                         self.heading_deg()))

            lat_true = HOME_LAT + self.model.pos[1] / M_PER_DEG
            lon_true = HOME_LON + self.model.pos[0] / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))

            if TELEMETRY_PORT:
                try:
                    # Ground-truth state for external visualisers. The model
                    # keeps pitch nose-down/yaw-CW positive; emit display
                    # conventions (pitch nose-up positive) once, here.
                    self.sock.sendto(json.dumps({
                        "sid": self.sid,
                        "t": now - self.t0,
                        "pos": list(self.model.pos),
                        "vel": list(self.model.vel),
                        "att": [math.degrees(self.model.roll),
                                -math.degrees(self.model.pitch),
                                math.degrees(self.model.yaw) % 360.0],
                        "rates": [math.degrees(self.model.rates[0]),
                                  -math.degrees(self.model.rates[1]),
                                  math.degrees(self.model.rates[2])],
                        "motors": list(m),
                        "lat": lat_true,
                        "lon": lon_true,
                        "alt": HOME_ALT_M + self.model.pos[2],
                        "gps": bool(self.gps_valid),
                        "armed": self.status.armed if self.status else None,
                        "modes": self.status.modes if self.status else [],
                        "home": [HOME_LAT, HOME_LON, HOME_ALT_M],
                    }).encode(), ("127.0.0.1", TELEMETRY_PORT))
                except OSError:
                    pass  # fire-and-forget; a visualiser must never affect a scenario

            # The FC's bridge computes q = Rz(+90) * Rx(180) * q_packet * Rx(180),
            # so emit the true NWU attitude pre-rotated by Rz(-90) and
            # pre-conjugated: the FC recovers exactly q_nwu.
            q_nwu = quat_from_euler_bf(self.model.roll, self.model.pitch, self.model.yaw)
            q = quat_conj_x180(quat_mul(K_RZ_NEG90, q_nwu))

            # Specific force in the FC's earth frame (NWU), rotated into the
            # body with the same attitude the FC reconstructs, so the
            # estimator's tilt-compensation inverts this rotation exactly at
            # any heading. The packet carries the negated body vector - the
            # SITL acc driver negates all three axes on read.
            f_world_nwu = (
                self.model.accel[1],                     # north
                -self.model.accel[0],                    # west
                self.model.accel[2] + GRAVITY,           # up
            )
            f_body = quat_rotate_inv(q_nwu, f_world_nwu)

            # Out-of-range lat/lon = GPS-loss sentinel; the FC skips the
            # virtual GPS update and its receive timeout trips, while IMU
            # feeds stay live. (NaN would be folded away by -ffast-math.)
            lon_pkt = 2.0 * HOME_LON - lon_true if self.gps_valid else 999.0
            lat_pkt = 2.0 * HOME_LAT - lat_true if self.gps_valid else 999.0
            pkt = struct.pack(
                "<18d",
                now - self.t0,
                # Gazebo-plugin gyro frame: roll right +, pitch nose-up +,
                # yaw CCW +. The model keeps nose-down/CW positive (compass
                # conventions), so pitch and yaw are negated on emit.
                self.model.rates[0], -self.model.rates[1], -self.model.rates[2],
                -f_body[0], -f_body[1], -f_body[2],                              # negated NWU-body specific force
                q[0], q[1], q[2], q[3],
                self.model.vel[0], self.model.vel[1], self.model.vel[2],         # ENU m/s
                lon_pkt,                                                         # mirrored for the bridge
                lat_pkt,
                HOME_ALT_M + self.model.pos[2],
                101325.0,
            )
            self.sock.sendto(pkt, ("127.0.0.1", FDM_PORT))
            # Real-time pacing only; simulated time already advanced by FDM_STEP_S.
            # At high --speed this becomes a yield and the loop runs as fast as the
            # FC can consume, which is what actually buys the speedup.
            time.sleep(max(0.0, FDM_STEP_S / SPEED))

    def shutdown(self):
        self.running = False


class Msp:
    """Minimal MSP v1 client over the SITL TCP port."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""
        # serialises request/reply pairs: the status poller thread shares this
        # connection with scenario bodies
        self.lock = threading.Lock()

    def request(self, cmd, payload=b"", timeout=2.0):
        with self.lock:
            frame = struct.pack("<BB", len(payload), cmd) + payload
            checksum = 0
            for b in frame:
                checksum ^= b
            self.sock.sendall(b"$M<" + frame + bytes([checksum]))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                reply = self._read_frame(cmd, deadline)
                if reply is not None:
                    return reply
            raise TimeoutError(f"no MSP reply for cmd {cmd}")

    def _read_frame(self, want_cmd, deadline):
        while time.monotonic() < deadline:
            start = self.buf.find(b"$M>")
            if start < 0:
                # also tolerate error frames
                if self.buf.find(b"$M!") >= 0:
                    raise RuntimeError(f"MSP error frame for cmd {want_cmd}")
                self._fill(deadline)
                continue
            if len(self.buf) < start + 5:
                self._fill(deadline)
                continue
            size = self.buf[start + 3]
            cmd = self.buf[start + 4]
            end = start + 5 + size + 1
            if len(self.buf) < end:
                self._fill(deadline)
                continue
            payload = self.buf[start + 5 : start + 5 + size]
            self.buf = self.buf[end:]
            if cmd == want_cmd:
                return payload
        return None

    def _fill(self, deadline):
        self.sock.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            data = self.sock.recv(4096)
            if data:
                self.buf += data
        except socket.timeout:
            pass


class Sitl:
    def __init__(self, binary, workdir):
        self.binary = os.path.abspath(binary)
        self.workdir = workdir
        self.proc = None
        self.sock = None
        self.msp = None
        self.boxids = []

    def provision(self, cli_lines):
        cfg = os.path.join(self.workdir, "scenario_config.txt")
        if DEBUG_MODE:
            cli_lines = list(cli_lines) + [f"set debug_mode = {DEBUG_MODE}"]
        with open(cfg, "w") as f:
            f.write("\n".join(cli_lines) + "\n")
        eeprom = os.path.join(self.workdir, "eeprom.bin")
        if os.path.exists(eeprom):
            os.remove(eeprom)
        res = subprocess.run(
            [self.binary, "--config", cfg],
            cwd=self.workdir,
            capture_output=True,
            text=True,
            timeout=60,
        )
        debug(f"provision rc={res.returncode}")
        if res.returncode != 0:
            raise RuntimeError(f"provisioning failed (rc={res.returncode}):\n{res.stdout}\n{res.stderr}")
        if not os.path.exists(eeprom):
            raise RuntimeError(f"provisioning produced no eeprom.bin:\n{res.stdout}\n{res.stderr}")

    @staticmethod
    def wait_port_free(timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                probe = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=0.3)
                probe.close()
                time.sleep(0.3)
            except OSError:
                return
        raise RuntimeError("previous SITL still holds the MSP port")

    def start(self):
        # The TCP listener has no SO_REUSEADDR; sockets from a previous scenario
        # lingering in TIME_WAIT can make the bind fail silently, so retry the
        # whole launch a few times rather than only polling for the port.
        for attempt in range(3):
            self.wait_port_free()
            logf = open(os.path.join(self.workdir, "sitl.log"), "a")
            self.proc = subprocess.Popen(["stdbuf", "-oL", self.binary], cwd=self.workdir, stdout=logf, stderr=logf)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if self.proc.poll() is not None:
                    debug(f"SITL exited early (rc={self.proc.returncode}); relaunching")
                    break
                try:
                    self.sock = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=1)
                    self.msp = Msp(self.sock)
                    self.boxids = list(self.msp.request(MSP_BOXIDS))
                    debug(f"boxids: {self.boxids}")
                    return
                except (OSError, TimeoutError, RuntimeError) as exc:
                    debug(f"MSP startup probe failed: {exc}")
                    time.sleep(0.3)
            debug(f"launch attempt {attempt + 1} failed; relaunching")
            self.stop()
            time.sleep(2.0)
        raise RuntimeError("SITL did not open the MSP port after 3 launches")

    def status(self):
        p = self.msp.request(MSP_STATUS)
        mode_flags = struct.unpack_from("<I", p, 6)[0]
        extra_count = p[15]
        off = 16 + extra_count
        arming_count = p[off]
        arming_flags = struct.unpack_from("<I", p, off + 1)[0]
        active = {self.boxids[i] for i in range(min(32, len(self.boxids))) if mode_flags & (1 << i)}
        return {"modes": active, "arming_flags": arming_flags, "arming_count": arming_count}

    def modes(self):
        return self.status()["modes"]

    def gps(self):
        p = self.msp.request(MSP_RAW_GPS)
        lat, lon = struct.unpack_from("<ii", p, 2)
        return {"lat": lat / 1e7, "lon": lon / 1e7}

    def distance_to_m(self, lat, lon):
        g = self.gps()
        dn = (g["lat"] - lat) * M_PER_DEG
        de = (g["lon"] - lon) * M_PER_DEG * math.cos(math.radians(lat))
        return math.hypot(dn, de)

    def acc_calibrate(self):
        self.msp.request(MSP_ACC_CALIBRATION)

    def stop(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None


class StatusPoller(threading.Thread):
    """5 Hz MSP_STATUS poll feeding true arm/mode state into the telemetry
    fan-out. Read-only observer: errors are swallowed and the last state kept,
    so it can never fail a scenario."""

    BOX_NAMES = {
        BOX_ARM: "ARM",
        BOX_ALTHOLD: "ALTHOLD",
        BOX_POSHOLD: "POSHOLD",
        BOX_FAILSAFE: "FAILSAFE",
        BOX_GPSRESCUE: "GPSRESCUE",
        BOX_AUTOPILOT: "AUTOPILOT",
    }

    def __init__(self, sitl):
        super().__init__(daemon=True)
        self.sitl = sitl
        self.running = True
        self.armed = False
        self.modes = []

    def run(self):
        while self.running:
            try:
                if self.sitl.msp is not None:
                    modes = self.sitl.modes()
                    self.armed = BOX_ARM in modes
                    self.modes = [self.BOX_NAMES.get(b, f"BOX{b}") for b in sorted(modes) if b != BOX_ARM]
            except (TimeoutError, RuntimeError, OSError):
                pass
            time.sleep(max(0.01, 0.2 / SPEED))

    def shutdown(self):
        self.running = False


def wait_for(description, predicate, timeout=20.0, interval=0.2):
    # simulated-time deadline: scenario timeouts are in sim seconds and scale with --speed
    deadline = CLOCK.now() + timeout
    last = None
    while CLOCK.now() < deadline:
        last = predicate()
        if last:
            log(f"ok: {description}")
            return last
        CLOCK.sleep(interval)
    raise AssertionError(f"timeout waiting for: {description}")


WP_LAT = HOME_LAT + 300.0 / M_PER_DEG  # default waypoint 300 m north of home
WP_EAST_LON = HOME_LON + 150.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))  # 150 m east
WP_NORTH40_LAT = HOME_LAT + 40.0 / M_PER_DEG  # short leg for the landing mission
WP_EAST25_LON = HOME_LON + 25.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))
WP_NORTH90_LAT = HOME_LAT + 90.0 / M_PER_DEG  # far leg for the backwards-engage mission
# ~130 deg corner: a 60 m north leg into wp0, then out to (42 m east, 25 m north),
# so the outgoing leg bears ~130 deg and the pre-turn swings the nose past 90 deg
# off the inbound leg
WP_NORTH60_LAT = HOME_LAT + 60.0 / M_PER_DEG
WP_CORNER_LAT = HOME_LAT + 25.0 / M_PER_DEG
WP_CORNER_LON = HOME_LON + 42.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))

# --- landing horizontal-gain scenario --------------------------------------
# Existing LAND scenarios (mission_land, rx_land, geofence_land) either brake
# to a small residual before the descent starts, or land at the *current*
# position (zero horizontal error by construction), so none of them exercise
# a meaningful horizontal error during the descent. This waypoint pair
# deliberately engineers one: settle in a HOLD, then dispatch a LAND target
# offset ~1.5 m sideways from that settled point. Because the offset is
# inside the default 2 m hold radius and the craft is nearly stationary, the
# executor's arrival gate (positionNavUpdate's withinAcceptanceRadius check,
# any-speed completion) is satisfied on the very first tick of the LAND leg,
# so the descent begins from a clean, deliberate ~1.5 m horizontal step
# rather than whatever an approach happens to brake down to.
LAND_OFFSET_HOLD_DIST_M = 20.0   # metres north of home for the settle point
LAND_OFFSET_M = 0.3              # metres east: calibration run
LAND_OFFSET_HOLD_DURATION_DS = 100   # calibration: longer settle
LAND_OFFSET_HOLD_DURATION_S = LAND_OFFSET_HOLD_DURATION_DS / 10.0
LAND_OFFSET_HOLD_LAT = HOME_LAT + LAND_OFFSET_HOLD_DIST_M / M_PER_DEG
LAND_OFFSET_LAND_LON = HOME_LON + LAND_OFFSET_M / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))


def base_config(extra):
    return [
        "feature GPS",
        "set gps_provider = VIRTUAL",
        "set failsafe_procedure = AUTO-LAND",
        "set failsafe_delay = 10",
        "set small_angle = 180",
        "aux 0 0 0 1700 2100 0 0",   # ARM on AUX1
        "aux 1 56 1 1700 2100 0 0",  # AUTOPILOT on AUX2
        "aux 2 1 2 1700 2100 0 0",   # ANGLE on AUX3 (heading-validation flight)
        # the estimator needs the truth-fed virtual mag as a heading source
        "set trust_mag = ON",
        # Unified velocity-primitive controller: cruise tilt is carried by the
        # virtual-distance integral, so drag compensation is a small term kept
        # well below the D (velocity) gain rather than the cruise feedforward.
        "set ap_velocity_drag_coeff = 50",
        # 10 m above home, 5 m/s — low and quick keeps landing scenarios short
        f"waypoint insert 0 {WP_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
    ] + extra


def boot_and_engage(sitl, rc, fdm):
    """Common preamble: boot, GPS fix, arm, raise throttle, engage AUTOPILOT."""
    rc.start()
    fdm.start()

    wait_for("GPS fix + RX recovery (arming flags clear)", lambda: sitl.status()["arming_flags"] == 0, timeout=40)

    # Recalibrate the accelerometer now the FDM feed is live: the boot-time
    # calibration can capture offsets from a not-yet-settled feed, and the
    # resulting bias integrates into a phantom vertical velocity.
    sitl.acc_calibrate()
    sim_sleep(2.0)
    wait_for("recalibration complete", lambda: sitl.status()["arming_flags"] == 0, timeout=20)

    rc.set(6, RC_HIGH)  # AUX3: ANGLE for the manual segment
    # A transient arming-disable (e.g. an RXLOSS blip) at the moment the switch
    # goes high latches ARM_SWITCH until the switch is cycled; retry the arm.
    for attempt in range(3):
        rc.set(4, RC_HIGH)  # AUX1: arm (throttle is low)
        try:
            wait_for("armed", lambda: BOX_ARM in sitl.modes(), timeout=8)
            break
        except AssertionError:
            if attempt == 2:
                raise
            log("arm attempt latched ARM_SWITCH; cycling the switch")
            rc.set(4, 1000)
            sim_sleep(1.0)

    # Closed-loop climb. Under a force plant a throttle cut does not stop the
    # climb, it only removes the acceleration, so the craft coasts; every
    # scenario that gates on an altitude has to be handed a settled craft, not a
    # ballistic one. Climb, actively arrest, then hover.
    rc.set(2, 1600)     # raise throttle (wasThrottleRaised) and break ground
    wait_for("airborne", lambda: fdm.model.pos[2] > 2.5, timeout=20, interval=0.05)
    rc.set(2, 1100)     # below hover: arrest the climb instead of coasting on it
    wait_for("climb arrested", lambda: fdm.model.vel[2] < 0.5, timeout=20, interval=0.05)
    rc.set(2, 1300)     # ~hover collective; alt hold takes the altitude from here
    sim_sleep(1.0)

    rc.set(5, RC_HIGH)  # AUX2: AUTOPILOT
    required_modes = {BOX_AUTOPILOT, BOX_ALTHOLD, BOX_POSHOLD}

    def active_required_modes():
        modes = sitl.modes()
        return modes if modes >= required_modes else None

    modes = wait_for(
        "AUTOPILOT + ALTHOLD + POSHOLD active (mode wiring)",
        active_required_modes,
    )
    rc.set(2, 1300)     # throttle into the alt-hold deadband: no stick adjustments
    return modes


def scenario_mission_flight(sitl, rc, fdm):
    """Closed-loop flight: the mission leg is actually flown by the motion
    model under Betaflight's own controllers, ending parked at the waypoint."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "vehicle departs toward the waypoint (>15 m from home)",
        lambda: fdm.distance_from_home() > 15.0,
        timeout=30,
    )
    # Assertions use the model's ground truth; the FC estimator tracks it,
    # while MSP_RAW_GPS leads the true position (virtual-GPS extrapolation).
    # Mid-leg cruise: sample in the plateau (past the accel ramp, before the
    # ~42 m braking taper) and check the velocity loop holds the commanded
    # 5 m/s without overshoot.
    cruise_samples = []

    def reached_or_sample():
        d = fdm.distance_to_wp(0.0, 300.0)
        if fdm.distance_from_home() > 50.0 and d > 100.0:
            cruise_samples.append(math.hypot(fdm.model.vel[0], fdm.model.vel[1]))
        return d < 8.0

    wait_for(
        "waypoint reached (within 8 m, ground truth)",
        lambda: reached_or_sample(),
        timeout=150,
        interval=1.0,
    )
    assert len(cruise_samples) >= 5, f"cruise plateau too short: {len(cruise_samples)} samples"
    cruise_avg = sum(cruise_samples) / len(cruise_samples)
    cruise_max = max(cruise_samples)
    assert 0.8 * 5.0 <= cruise_avg <= 1.2 * 5.0, f"cruise speed off target: avg {cruise_avg:.2f} m/s"
    assert cruise_max <= 1.3 * 5.0, f"cruise overshoot: peak {cruise_max:.2f} m/s"
    log(f"cruise avg {cruise_avg:.2f} m/s, peak {cruise_max:.2f} m/s over {len(cruise_samples)} samples")
    # Mission complete: executor parks in position hold at the waypoint.
    # Legs complete on radius entry; the hold-mode braking parks a short
    # distance past the point at cruise speed.
    wait_for(
        "settled near the waypoint",
        lambda: math.hypot(fdm.model.vel[0], fdm.model.vel[1]) < 1.0 and fdm.distance_to_wp(0.0, 300.0) < 25.0,
        timeout=60,
        interval=2.0,
    )
    # dwell: a transit averages cruise speed, a hold oscillates about the
    # point (instantaneous peaks reach ~2 m/s with SITL's 15 Hz position loop)
    samples = []
    for _ in range(10):
        sim_sleep(1.0)
        samples.append(math.hypot(fdm.model.vel[0], fdm.model.vel[1]))
    dist = fdm.distance_to_wp(0.0, 300.0)
    avg_speed = sum(samples) / len(samples)
    assert dist < 25.0, f"did not hold position near waypoint: {dist:.1f} m away"
    assert avg_speed < 1.5, f"did not settle at waypoint: averaging {avg_speed:.1f} m/s"
    assert BOX_ARM in sitl.modes(), "unexpected disarm at mission end"
    log(f"parked {dist:.1f} m from the waypoint")


def scenario_mission_yaw(sitl, rc, fdm):
    """Default VELOCITY yaw mode: flying an eastbound leg, the nose must swing
    from north to the ground course and hold it while the leg is flown."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "vehicle departs east toward the waypoint",
        lambda: fdm.model.pos[0] > 15.0,
        timeout=30,
    )

    def on_course():
        err = (fdm.heading_deg() - 90.0 + 180.0) % 360.0 - 180.0
        return abs(err) < 25.0

    wait_for("nose tracks the course (heading ~090)", on_course, timeout=30)
    sim_sleep(3.0)
    assert on_course(), f"heading did not hold the course: {fdm.heading_deg():.0f} deg"

    # SITL's starved LOW-priority scheduler runs the position controller at
    # ~15-20 Hz (vs 100 Hz on hardware), so the braking phase can wander
    # before converging; the timeout allows for it.
    wait_for(
        "waypoint reached (ground truth)",
        lambda: fdm.distance_to_wp(150.0, 0.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm during yaw mission"
    log(f"leg flown nose-first, heading {fdm.heading_deg():.0f} deg at arrival")


def scenario_mission_engage_backwards(sitl, rc, fdm):
    """Engage with the nose pointing away from the first leg (initial_yaw_deg=180,
    leg runs north). In the default VELOCITY yaw mode no course develops while the
    craft sits still, so the executor must rotate the nose onto the leg and fly it
    rather than freezing the carrot into a STALL abort."""
    boot_and_engage(sitl, rc, fdm)

    # Departing at all proves it didn't deadlock: a frozen carrot never moves,
    # develops no course, and aborts STALLED after 30 s.
    wait_for(
        "rotates onto the leg and departs (>15 m from home)",
        lambda: fdm.distance_from_home() > 15.0,
        timeout=40,
    )

    def on_leg():
        err = (fdm.heading_deg() - 0.0 + 180.0) % 360.0 - 180.0
        return abs(err) < 30.0

    wait_for("nose swung onto the northbound leg (~000)", on_leg, timeout=25)

    wait_for(
        "reaches the far waypoint (ground truth)",
        lambda: fdm.distance_to_wp(0.0, 90.0) < 10.0,
        timeout=120,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm on the backwards-engage mission"
    log(f"rotated onto the leg from a backwards engage, heading {fdm.heading_deg():.0f} deg")


def scenario_mission_corner(sitl, rc, fdm):
    """A ~130 deg corner. The pre-turn deliberately swings the nose past 90 deg
    off the inbound leg approaching the gate; the march gate must exempt the
    pre-turn so the craft carries corner speed through the gate instead of
    freezing in it (the hairpin-stall bug)."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "flies the first leg to the corner waypoint",
        lambda: fdm.distance_to_wp(0.0, 60.0) < 12.0,
        timeout=90,
        interval=1.0,
    )

    # Sample ground speed while crossing the corner: a stalled gate would drop it
    # toward zero; a working pre-turn carries it through near the corner speed.
    corner_samples = []

    def carried_through():
        if fdm.distance_to_wp(0.0, 60.0) < 20.0:
            corner_samples.append(math.hypot(fdm.model.vel[0], fdm.model.vel[1]))
        return fdm.distance_to_wp(42.0, 25.0) < 10.0

    wait_for(
        "carries through the corner to the second waypoint",
        carried_through,
        timeout=120,
        interval=1.0,
    )
    assert corner_samples, "never sampled near the corner"
    corner_min = min(corner_samples)
    assert corner_min > 0.8, f"stalled in the corner: min ground speed {corner_min:.2f} m/s"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the corner mission"
    log(f"carried the corner, min ground speed {corner_min:.2f} m/s over {len(corner_samples)} samples")


def scenario_mission_land(sitl, rc, fdm):
    """LAND waypoint: fly the first leg north, divert to the offset LAND
    waypoint, arrive through the 3D hold gate, loiter for the waypoint
    duration, descend, and disarm on touchdown (impact jerk; the estimator's
    vz is unreliable when grounded)."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "vehicle departs toward the first waypoint",
        lambda: fdm.distance_from_home() > 15.0,
        timeout=30,
    )
    wait_for(
        "LAND waypoint approach (ground truth)",
        lambda: fdm.distance_to_wp(25.0, 40.0) < 6.0,
        timeout=120,
        interval=1.0,
    )
    # 5 s pre-descent loiter: shortly after arrival the vehicle must still be
    # holding altitude (an immediate 2 m/s descent would be ~4 m down by now)
    sim_sleep(2.0)
    assert BOX_ARM in sitl.modes(), "disarmed during the loiter"
    assert fdm.model.pos[2] > 7.0, f"descended during the loiter: alt {fdm.model.pos[2]:.1f} m"
    log(f"loitering at {fdm.model.pos[2]:.1f} m before descent")
    wait_for(
        "touchdown disarms",
        lambda: BOX_ARM not in sitl.modes(),
        timeout=90,
        interval=1.0,
    )
    assert fdm.model.on_ground(), f"disarmed in the air: alt {fdm.model.pos[2]:.1f} m"
    dist = fdm.distance_to_wp(25.0, 40.0)
    assert dist < 10.0, f"landed {dist:.1f} m from the LAND waypoint"
    log(f"landed {dist:.1f} m from the LAND waypoint")


LAND_OFFSET_ONSET_BACKOFF_S = 0.5   # window starts this far before the detected onset


def horizontal_descent_metrics(fdm, t_onset, land_east_m, land_north_m):
    """Analyse horizontal position/velocity from the LAND horizontal step to
    ground contact, testing the landing-gain hypothesis: that the decoupled
    horizontal budget raises the effective position gain enough to under-damp
    the approach and swing around the target.

    The window opens LAND_OFFSET_ONSET_BACKOFF_S before t_onset because the
    onset is detected from the craft's *response*, which necessarily lags the
    command step by the position-loop and plant lag; backing off keeps the
    step itself inside the window.

    Returns None if too few samples were recorded. Otherwise:
      reversals         - sign changes of the velocity component along the
                          fixed axis from the craft's position at window open
                          toward the LAND target. This is the oscillation
                          metric: 0 means a single monotonic closing move,
                          >= 1 means the craft reversed direction along the
                          error axis at least once
      reversal_ivl_mean/std - mean and standard deviation of the interval
                          between successive reversals (s). A coherent limit
                          cycle has std << mean; irregular noise-driven
                          dither has std comparable to or larger than mean.
                          Both None with fewer than 3 reversals.
      peak_hspeed       - peak horizontal ground speed in the window (m/s)
      herr_touchdown    - horizontal distance from the LAND target at first
                          ground contact (m)
      hspeed_touchdown  - horizontal ground speed at first ground contact (m/s)
      entry_err         - largest horizontal error in the first second of the
                          window, i.e. the size of the commanded step (m)
      settle_err        - horizontal error averaged over the last 1.5 s before
                          contact (m): where the loop actually converged
      hspeed_std_early/late - standard deviation of horizontal speed over the
                          first and last thirds of the descent. Decaying
                          (late << early) is a damped approach; sustained or
                          growing is the oscillation the hypothesis predicts
      touchdown_found   - whether ground contact was seen in the window
      duration          - length of the analysed window (s)
      n_samples         - sample count in the window (~10 Hz)
    """
    t_start = max(0.0, t_onset - LAND_OFFSET_ONSET_BACKOFF_S)
    samples = [s for s in fdm.snapshot_history() if s[0] >= t_start]
    if len(samples) < 5:
        return None

    # Fixed radial axis: from the craft's position at window open toward the
    # (horizontally static) LAND point. Positive velocity along it is closing
    # on the target, negative is moving away - the axis the horizontal control
    # law is actually driving, not an arbitrary east/north split.
    e0, n0 = samples[0][1], samples[0][2]
    axis_e, axis_n = land_east_m - e0, land_north_m - n0
    norm = math.hypot(axis_e, axis_n)
    if norm > 0.01:
        axis_e, axis_n = axis_e / norm, axis_n / norm
    else:
        axis_e, axis_n = 1.0, 0.0  # degenerate: already on target, axis choice is moot

    times, vr_series, hspeed_series, herr_series = [], [], [], []
    touchdown_idx = None
    for i, s in enumerate(samples):
        t, e, n, up, ve, vn, _vu, _hdg = s
        times.append(t)
        vr_series.append(ve * axis_e + vn * axis_n)
        hspeed_series.append(math.hypot(ve, vn))
        herr_series.append(math.hypot(land_east_m - e, land_north_m - n))
        if touchdown_idx is None and i > 0 and up <= 0.02:
            touchdown_idx = i

    end = touchdown_idx if touchdown_idx is not None else len(samples) - 1
    if end < 4:
        return None

    # Zero crossings of the along-axis velocity, with a deadband so estimator
    # and loop-rate noise near zero is not counted as a reversal.
    DEADBAND_MPS = 0.05
    reversal_times = []
    last_sign = 0
    for t, vr in zip(times[: end + 1], vr_series[: end + 1]):
        sign = 1 if vr > DEADBAND_MPS else (-1 if vr < -DEADBAND_MPS else 0)
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            reversal_times.append(t)
        last_sign = sign

    ivl_mean = ivl_std = None
    if len(reversal_times) >= 3:
        ivls = [b - a for a, b in zip(reversal_times, reversal_times[1:])]
        ivl_mean = sum(ivls) / len(ivls)
        ivl_std = math.sqrt(sum((x - ivl_mean) ** 2 for x in ivls) / len(ivls))

    def stdev(xs):
        if len(xs) < 2:
            return 0.0
        mu = sum(xs) / len(xs)
        return math.sqrt(sum((x - mu) ** 2 for x in xs) / len(xs))

    third = max(2, (end + 1) // 3)
    t_end = times[end]
    settle_window = [h for t, h in zip(times[: end + 1], herr_series[: end + 1]) if t >= t_end - 1.5]
    entry_window = [h for t, h in zip(times[: end + 1], herr_series[: end + 1]) if t <= times[0] + 1.0]

    return {
        "reversals": len(reversal_times),
        "reversal_ivl_mean": ivl_mean,
        "reversal_ivl_std": ivl_std,
        "peak_hspeed": max(hspeed_series[: end + 1], default=0.0),
        "herr_touchdown": herr_series[end],
        "hspeed_touchdown": hspeed_series[end],
        "entry_err": max(entry_window, default=herr_series[0]),
        "settle_err": (sum(settle_window) / len(settle_window)) if settle_window else herr_series[end],
        "hspeed_std_early": stdev(hspeed_series[:third]),
        "hspeed_std_late": stdev(hspeed_series[end + 1 - third : end + 1]),
        "touchdown_found": touchdown_idx is not None,
        "duration": t_end - times[0],
        "n_samples": end + 1,
    }


def scenario_mission_land_offset(sitl, rc, fdm):
    """LAND engaged with a deliberate ~1.5 m horizontal offset (landing-gain
    investigation): settle in a HOLD, then dispatch a LAND target offset from
    that settled point but still inside the LAND arrival gate, so the descent
    begins immediately from a clean horizontal step rather than from whatever
    residual an approach happens to brake down to.

    Records horizontal position and velocity throughout the descent and
    reports the oscillation / peak / touchdown metrics. Deliberately does NOT
    assert on them: they are the quantity under measurement and are noisy run
    to run, so the pass/fail assertions here cover only that the manoeuvre
    actually happened (offset engaged, descended, landed)."""
    boot_and_engage(sitl, rc, fdm)

    # Detecting the LAND offset step from the craft's own motion is unreliable:
    # the outbound leg to the hold point and the hold's own altitude-settle
    # wobble both already exceed small velocity thresholds, so a
    # velocity-threshold detector fires early (verified against a real run).
    # Instead, mark the hold's own arrival-gate crossing (any-speed, horizontal
    # radius only - see positionNavUpdate's withinAcceptanceRadius check) and
    # schedule the LAND dispatch from the fixed HOLD duration in the mission,
    # which runs off the same simulated clock the harness itself advances.
    wait_for(
        "vehicle crosses the hold arrival radius",
        lambda: fdm.distance_to_wp(0.0, LAND_OFFSET_HOLD_DIST_M) < 2.0,
        timeout=60,
        interval=0.05,
    )
    hold_dist = fdm.distance_to_wp(0.0, LAND_OFFSET_HOLD_DIST_M)
    hold_speed = math.hypot(fdm.model.vel[0], fdm.model.vel[1])
    log(f"hold arrival gate crossed: {hold_dist:.2f} m, {hold_speed:.2f} m/s "
        f"(not yet settled - the {LAND_OFFSET_HOLD_DURATION_S:.1f} s hold duration below is what settles it)")

    sim_sleep(LAND_OFFSET_HOLD_DURATION_S + 0.4)   # + margin over the scheduler/poll slack
    t_onset = fdm.now_t()

    land_east_m, land_north_m = LAND_OFFSET_M, LAND_OFFSET_HOLD_DIST_M
    entry_err_now = fdm.distance_to_wp(land_east_m, land_north_m)
    log(f"LAND dispatch scheduled at t={t_onset:.1f} s: horizontal error {entry_err_now:.2f} m, "
        f"speed {math.hypot(fdm.model.vel[0], fdm.model.vel[1]):.2f} m/s")

    wait_for("touchdown disarms", lambda: BOX_ARM not in sitl.modes(), timeout=90, interval=0.5)
    assert fdm.model.on_ground(), f"disarmed in the air: alt {fdm.model.pos[2]:.1f} m"

    m = horizontal_descent_metrics(fdm, t_onset, land_east_m, land_north_m)
    assert m is not None, "no usable descent samples recorded"
    assert m["entry_err"] > 0.5 * LAND_OFFSET_M, (
        f"the offset step never materialised: entry error only {m['entry_err']:.2f} m "
        f"(expected ~{LAND_OFFSET_M:.1f} m)")
    m["hold_dist_at_gate"] = hold_dist
    m["touchdown_speed"] = fdm.model.touchdown_speed
    log("[metrics] " + json.dumps({k: (round(v, 4) if isinstance(v, float) else v)
                                   for k, v in m.items()}))
    return m


def scenario_mission_takeoff(sitl, rc, fdm):
    """TAKEOFF waypoint: climb in place to the waypoint altitude (its lat/lon
    are advisory), then fly the following leg. The climb must not translate."""
    boot_and_engage(sitl, rc, fdm)
    t_engage = fdm.now_t()

    wait_for(
        "climb through 12 m (TAKEOFF target 15 m)",
        lambda: fdm.model.pos[2] > 12.0,
        timeout=90,
        interval=0.5,
    )
    t_top = fdm.now_t()

    # Horizontal drift during the climb, relative to where the mission engaged
    # (TAKEOFF holds the current position, not home).
    climb = [s for s in fdm.snapshot_history() if t_engage <= s[0] <= t_top]
    assert climb, "no recorded samples during the climb"
    e0, n0 = climb[0][1], climb[0][2]
    drift = max(math.hypot(s[1] - e0, s[2] - n0) for s in climb)
    assert drift < 8.0, f"translated {drift:.1f} m during the TAKEOFF climb"
    log(f"climbed to {fdm.model.pos[2]:.1f} m with {drift:.1f} m drift")

    wait_for(
        "leg to the north waypoint after the climb",
        lambda: fdm.distance_to_wp(0.0, 40.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the takeoff mission"


def hold_window_samples(fdm, centre_e, centre_n, t0, t1):
    """(distances, azimuth sweep in rad) of history samples in [t0, t1],
    measured about the hold point. Sweep accumulates wrapped step deltas, so
    systematic circulation grows it while hover noise cancels out."""
    pts = [s for s in fdm.snapshot_history() if t0 <= s[0] <= t1]
    dists = [math.hypot(s[1] - centre_e, s[2] - centre_n) for s in pts]
    azimuths = [math.atan2(s[2] - centre_n, s[1] - centre_e) for s in pts]
    sweep = 0.0
    for a, b in zip(azimuths, azimuths[1:]):
        sweep += (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return dists, sweep


def scenario_mission_orbit(sitl, rc, fdm):
    """HOLD with the ORBIT pattern: after arriving at the hold point the
    vehicle must circulate around it on the hold radius for the duration."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "arrival at the hold point",
        lambda: fdm.distance_to_wp(0.0, 40.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    t_arrive = fdm.now_t()

    # Analysis window: skip 12 s (arrival braking + pattern spin-up), observe
    # 40 s of the 60 s hold. Carrot rate 0.25 rad/s -> ~1.6 laps in the window.
    wait_for("orbit window elapsed", lambda: fdm.now_t() > t_arrive + 52.0,
             timeout=70, interval=2.0)
    dists, sweep = hold_window_samples(fdm, 0.0, 40.0, t_arrive + 12.0, t_arrive + 52.0)
    assert len(dists) > 250, f"recorder too sparse over the hold window: {len(dists)} samples"

    mean_dist = sum(dists) / len(dists)
    log(f"orbit mean radius {mean_dist:.1f} m, peak {max(dists):.1f} m, "
        f"swept {math.degrees(sweep):.0f} deg")
    # The vehicle rides the ring with pursuit lag (a little inside) plus the
    # loop phase lag (a little outside); a hover at the hold point would sit
    # near zero and a runaway pursuit far outside.
    assert 3.0 < mean_dist < 12.0, f"orbit radius off: mean {mean_dist:.1f} m from the hold point"
    assert max(dists) < 16.0, f"orbit excursion: {max(dists):.1f} m from the hold point"
    assert sweep > math.radians(270.0), f"no sustained circulation: swept {math.degrees(sweep):.0f} deg"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the orbit"


def scenario_mission_figure8(sitl, rc, fdm):
    """HOLD with the FIGURE8 pattern: bounded excursion about the hold point
    with repeated passes back through the centre."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "arrival at the hold point",
        lambda: fdm.distance_to_wp(0.0, 40.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    t_arrive = fdm.now_t()

    wait_for("figure-8 window elapsed", lambda: fdm.now_t() > t_arrive + 52.0,
             timeout=70, interval=2.0)
    dists, _ = hold_window_samples(fdm, 0.0, 40.0, t_arrive + 12.0, t_arrive + 52.0)
    assert len(dists) > 250, f"recorder too sparse over the hold window: {len(dists)} samples"

    # Lemniscate on an 8 m radius: lobes reach the ring, the path re-crosses
    # the centre twice per cycle (~25 s), and never leaves the hold radius.
    log(f"figure-8 peak {max(dists):.1f} m from the hold point")
    assert max(dists) < 13.0, f"figure-8 excursion: {max(dists):.1f} m from the hold point"
    assert max(dists) > 4.0, f"no pattern motion: peak {max(dists):.1f} m from the hold point"
    crossings = 0
    away = False
    for d in dists:
        if d > 5.0:
            away = True
        elif away and d < 4.0:
            crossings += 1
            away = False
    assert crossings >= 2, f"path did not re-cross the centre: {crossings} passes"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the figure-8"
    log(f"figure-8 {crossings} centre passes")


def scenario_rx_loss(sitl, rc, fdm, policy):
    boot_and_engage(sitl, rc, fdm)
    log(f"killing RC stream (policy={policy})")
    rc.stop_stream()

    if policy == "CONTINUE":
        wait_for(
            "failsafe active with mission continuing",
            lambda: {BOX_FAILSAFE, BOX_AUTOPILOT} <= sitl.modes(),
        )
        sim_sleep(3)
        modes = sitl.modes()
        assert {BOX_FAILSAFE, BOX_AUTOPILOT} <= modes, f"CONTINUE state did not persist: {modes}"
        log("mission still flying 3 s into failsafe")
    elif policy == "LAND":
        wait_for(
            "failsafe landing with mission disengaged",
            lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT not in m
                     and {BOX_ALTHOLD, BOX_POSHOLD} <= m)(sitl.modes()),
        )
    else:  # DISABLE
        wait_for(
            "failsafe active with mission disengaged",
            lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT not in m)(sitl.modes()),
        )


def scenario_geofence(sitl, rc, fdm, action):
    boot_and_engage(sitl, rc, fdm)
    log(f"flying north until the 50 m geofence trips (action={action})")
    # The FC's gpsSol leads ground truth, so the fence fires around 40-50 m of
    # true distance; the resulting action is the observable, not the distance.
    wait_for("mission departs toward the fence", lambda: fdm.distance_from_home() > 25.0, timeout=60)

    if action == "RTH":
        # Plan swap, not rescue: the mission keeps flying (an injected
        # [fly home, land] plan) and the vehicle comes back inside the fence.
        sim_sleep(3)
        modes = sitl.modes()
        assert BOX_GPSRESCUE not in modes, f"rescue engaged instead of plan swap: {modes}"
        assert BOX_AUTOPILOT in modes, f"mission dropped on breach: {modes}"
        wait_for(
            "vehicle returns inside the fence",
            lambda: fdm.distance_from_home() < 25.0,
            timeout=120,
            interval=1.0,
        )
        wait_for(
            "touchdown at home disarms",
            lambda: BOX_ARM not in sitl.modes(),
            timeout=120,
            interval=1.0,
        )
        dist = fdm.distance_from_home()
        assert fdm.model.on_ground(), f"disarmed in the air: alt {fdm.model.pos[2]:.1f} m"
        assert dist < 10.0, f"landed {dist:.1f} m from home"
        log(f"returned and landed {dist:.1f} m from home")
    else:  # LAND
        sim_sleep(8)
        modes = sitl.modes()
        if BOX_ARM in modes:
            assert BOX_AUTOPILOT in modes, f"mission dropped instead of landing: {modes}"
        assert BOX_GPSRESCUE not in modes, f"unexpected rescue: {modes}"
        log("mission holds LANDING state at current position")
        # The motion model descends under the landing command until ground
        # contact; the touchdown detector must then disarm.
        wait_for(
            "touchdown detection disarms",
            lambda: BOX_ARM not in sitl.modes(),
            timeout=45,
            interval=1.0,
        )


def scenario_geofence_rth_rxloss(sitl, rc, fdm):
    """The geofence return must survive RX loss: with rx-loss policy CONTINUE
    the failsafe keeps flying the injected return plan while stage-2 rxfail
    values force the AUTOPILOT switch low."""
    boot_and_engage(sitl, rc, fdm)
    wait_for("mission departs toward the fence", lambda: fdm.distance_from_home() > 40.0, timeout=60)
    wait_for(
        "return leg underway (heading back inside 40 m)",
        lambda: fdm.distance_from_home() < 40.0,
        timeout=90,
        interval=0.5,
    )

    log("killing RC stream mid-return")
    rc.stop_stream()
    wait_for("failsafe active", lambda: BOX_FAILSAFE in sitl.modes())
    sim_sleep(3)
    modes = sitl.modes()
    assert {BOX_FAILSAFE, BOX_AUTOPILOT} <= modes, f"return plan dropped on RX loss: {modes}"
    assert BOX_GPSRESCUE not in modes, f"unexpected rescue: {modes}"
    log("return continues through RX loss")
    wait_for(
        "lands at home under failsafe",
        lambda: BOX_ARM not in sitl.modes(),
        timeout=150,
        interval=1.0,
    )
    dist = fdm.distance_from_home()
    assert dist < 10.0, f"landed {dist:.1f} m from home"
    log(f"landed {dist:.1f} m from home under failsafe")


# --- GPS rescue scenarios -------------------------------------------------
# GPS rescue is flown as an autopilot mission (ENABLE_RESCUE_PLAN, now the
# default on flight-plan targets): on RC loss the failsafe procedure stages a
# rescue plan and flies it under FAILSAFE + AUTOPILOT. Each scenario asserts the
# safety outcome directly: engage, return, land near home, disarm, no flyaway.

RESCUE_CFG = [
    "set failsafe_procedure = GPS-RESCUE",
    # FIXED_ALT: the default MAX mode keys the return altitude to each leg's
    # own outbound peak, which varies run to run and would dominate the A/B
    # altitude comparison; MAX synthesis is unit-tested instead
    "set gps_rescue_alt_mode = FIXED_ALT",
    "set gps_rescue_return_alt = 30",     # long climb widens the ascendRate-clamp margin
    # ascendRate 1 m/s clamps the climb feedforward well under the ~2.2 m/s the
    # model reaches on this climb under the alt-hold climbRate (5 m/s), so the
    # climb rate proves ascendRate. descendRate 0.8 m/s governs the fallback
    # descent (baro-only, velocity-tracked) held under the ~1.26 m/s throttle-
    # floor descent the alt-hold climbRate would otherwise drive it to.
    "set gps_rescue_ascend_rate = 100",
    "set gps_rescue_descend_rate = 80",
    "set ap_yaw_mode = FIXED",
    "set ap_waypoint_hold_radius = 400",
    "set ap_landing_descent_rate = 200",
    "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
    "aux 3 3 3 1700 2100 0 0",   # ALTHOLD on AUX4: pilot-flown hold after the mission leg
    "aux 4 11 3 1700 2100 0 0",  # POSHOLD on AUX4
    "feature BLACKBOX",
    "set blackbox_device = VIRTUAL",       # .BFL artifact in the scenario dir
]

DEBUG_MODE = None          # set from --debug-mode; appended to every scenario config


def fly_out_and_park(sitl, rc, fdm, dist_m):
    """Mission leg out to dist_m, then hand to pilot-held ALTHOLD+POSHOLD."""
    boot_and_engage(sitl, rc, fdm)
    wait_for(
        f"vehicle {dist_m:.0f} m out",
        lambda: fdm.distance_from_home() > dist_m,
        timeout=90,
        interval=1.0,
    )
    rc.set(7, RC_HIGH)  # AUX4: ALTHOLD + POSHOLD (pilot hold)
    rc.set(5, 1000)     # AUX2: AUTOPILOT off
    wait_for(
        "pilot hold (AUTOPILOT off, POSHOLD on)",
        lambda: (lambda m: BOX_AUTOPILOT not in m and BOX_POSHOLD in m)(sitl.modes()),
        timeout=10,
    )
    sim_sleep(2.0)


def rescue_engagement_asserts(sitl, variant="B"):
    # Converged rescue: GPS rescue is flown as an autopilot mission, so it runs
    # under FAILSAFE + AUTOPILOT and never enables the legacy GPS_RESCUE box.
    wait_for(
        "rescue mission engaged (FAILSAFE + AUTOPILOT)",
        lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT in m)(sitl.modes()),
        timeout=20,
    )
    assert BOX_GPSRESCUE not in sitl.modes(), "legacy GPS_RESCUE engaged instead of the rescue mission"


def rescue_metrics(fdm, t0, kill_dist):
    return {
        "kill_dist": kill_dist,
        "max_alt": max((s[3] for s in fdm.snapshot_history() if s[0] >= t0), default=0.0),
        "max_dist": fdm.max_distance_from_home(after_t=t0),
        "time_to_home": fdm.time_to_home(radius_m=20.0, after_t=t0),
        "touchdown": fdm.touchdown(after_t=t0),
    }


def band_descent_rate(fdm, t0, lo_alt, hi_alt):
    """Median descent rate (m/s, positive down) over an altitude band, ignoring
    the ramp-in at the top and the near-ground slowdown."""
    s = sorted(-r[6] for r in fdm.snapshot_history()
               if r[0] >= t0 and lo_alt <= r[3] <= hi_alt and r[6] < -0.1)
    return s[len(s) // 2] if s else 0.0


def assert_rescue_climb_rate(fdm, t0, variant):
    # The rescue climb feedforward is clamped to gps_rescue_ascend_rate (1 m/s).
    # The altitude P-term still drives a transient above the cap, but at a much
    # lower peak (~1.8 m/s) than the ~2.6 m/s this climb reaches under the
    # alt-hold climbRate (5 m/s): the peak shows ascendRate shaping the climb.
    peak_climb = max((s[6] for s in fdm.snapshot_history() if s[0] >= t0), default=0.0)
    log(f"[{variant}] climb rate: peak {peak_climb:.2f} m/s (ascendRate 1.0)")
    assert 0.6 <= peak_climb <= 2.25, f"climb not held to ascendRate: {peak_climb:.2f} m/s"


def scenario_rescue_ab(sitl, rc, fdm, variant="B"):
    fly_out_and_park(sitl, rc, fdm, 120.0)
    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {kill_dist:.0f} m out")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for(
        "returns within 20 m of home",
        lambda: fdm.distance_from_home() < 20.0,
        timeout=120,
        interval=1.0,
    )
    wait_for(
        "touchdown disarms",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=120,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    td_dist = math.hypot(td[1], td[2])
    assert td_dist < 15.0, f"landed {td_dist:.1f} m from home"
    log(f"[{variant}] landed {td_dist:.1f} m from home, peak alt {m['max_alt']:.1f} m")
    m["td_dist"] = td_dist

    assert_rescue_climb_rate(fdm, t0, variant)
    return m


def scenario_rescue_heading(sitl, rc, fdm, variant="B"):
    """No mag, true heading east while the FC believes north: the rescue must
    recover heading via GPS course-over-ground (pitch-forward phase) before
    flying home."""
    rc.start()
    fdm.start()
    wait_for("GPS fix + RX recovery (arming flags clear)", lambda: sitl.status()["arming_flags"] == 0, timeout=40)
    sitl.acc_calibrate()
    sim_sleep(2.0)
    wait_for("recalibration complete", lambda: sitl.status()["arming_flags"] == 0, timeout=20)

    rc.set(6, RC_HIGH)  # ANGLE
    for attempt in range(3):
        rc.set(4, RC_HIGH)
        try:
            wait_for("armed", lambda: BOX_ARM in sitl.modes(), timeout=8)
            break
        except AssertionError:
            if attempt == 2:
                raise
            rc.set(4, 1000)
            sim_sleep(1.0)
    rc.set(2, 1600)
    rc.set(7, RC_HIGH)  # ALTHOLD + POSHOLD hover (switch must be off at arm time)
    wait_for("climbed clear of ground", lambda: fdm.model.pos[2] > 6.0, timeout=20)
    rc.set(2, 1300)
    sim_sleep(2.0)

    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC at hover (heading wrong by 90 deg)")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    # heading recovery needs forward flight: the craft must depart, learn its
    # heading from GPS course, then come home and land
    wait_for(
        "touchdown disarms (heading recovered, rescue completed)",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=240,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    assert m["max_dist"] <= 150.0, f"heading-recovery excursion ran away: {m['max_dist']:.0f} m"
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    td_dist = math.hypot(td[1], td[2])
    assert td_dist < 30.0, f"landed {td_dist:.1f} m from home"
    log(f"recovered heading and landed {td_dist:.1f} m from home")
    m["td_dist"] = td_dist
    return m


def scenario_rescue_gps_loss(sitl, rc, fdm, variant="B"):
    fly_out_and_park(sitl, rc, fdm, 120.0)
    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {kill_dist:.0f} m out")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for(
        "return underway (30 m closer)",
        lambda: fdm.distance_from_home() < kill_dist - 30.0,
        timeout=90,
        interval=1.0,
    )
    loss_dist = fdm.distance_from_home()
    log(f"[{variant}] GPS dark {loss_dist:.0f} m out")
    fdm.gps_valid = False

    # A: legacy emergency descent (baro only). B: mission aborts on estimator
    # loss and failsafe degrades to the baro auto-landing. Both must get down
    # and disarm without flying away.
    wait_for(
        "descends and disarms without GPS",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=180,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    assert m["max_dist"] < kill_dist + 40.0, f"flew away after GPS loss: {m['max_dist']:.0f} m"
    log(f"[{variant}] down and disarmed after GPS loss")
    return m


def scenario_rescue_switch_descent(sitl, rc, fdm, variant="B"):
    """Switch-invoked rescue (no RC loss): the pilot flips BOXGPSRESCUE. The plan
    flies as an autopilot mission, then GPS goes dark mid-return. Because the
    SWITCH (not failsafe) invoked it, the aborted plan degrades to a controlled
    altitude-only descent and disarms - never entering FAILSAFE."""
    fly_out_and_park(sitl, rc, fdm, 120.0)   # out, then pilot hold; RC stays live
    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] flipping GPS-RESCUE switch {kill_dist:.0f} m out")
    rc.set(7, RC_LOW)    # hand over: drop pilot ALTHOLD+POSHOLD
    rc.set(8, RC_HIGH)   # AUX5: BOXGPSRESCUE

    # The switch flies the rescue as an autopilot mission: AUTOPILOT engages, never
    # the failsafe path or the legacy GPS_RESCUE box.
    wait_for(
        "rescue plan engaged via switch (AUTOPILOT, no FAILSAFE)",
        lambda: (lambda m: BOX_AUTOPILOT in m and BOX_FAILSAFE not in m)(sitl.modes()),
        timeout=20,
    )
    assert BOX_GPSRESCUE not in sitl.modes(), "legacy GPS_RESCUE engaged instead of the plan"

    wait_for(
        "return underway (30 m closer)",
        lambda: fdm.distance_from_home() < kill_dist - 30.0,
        timeout=90,
        interval=1.0,
    )
    loss_dist = fdm.distance_from_home()
    log(f"[{variant}] GPS dark {loss_dist:.0f} m out (switch still held)")
    fdm.gps_valid = False

    # Plan aborts on estimator loss; the switch invoker degrades to the baro-only
    # descent (not failsafe) and disarms without flying away.
    wait_for(
        "descends and disarms without GPS",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=180,
        interval=1.0,
    )
    assert BOX_FAILSAFE not in sitl.modes(), "entered FAILSAFE on a switch-invoked rescue"
    m = rescue_metrics(fdm, t0, kill_dist)
    assert m["max_dist"] < kill_dist + 40.0, f"flew away after GPS loss: {m['max_dist']:.0f} m"
    log(f"[{variant}] down and disarmed via switch-fallback descent")

    # The climb ran under ascendRate; the baro-only fallback descent runs at
    # gps_rescue_descend_rate (0.8 m/s) - the alt-hold climbRate (5 m/s) would
    # drive it to the ~1.26 m/s throttle floor. Sample the descent near ground,
    # where the failsafe-landing profile's altitude scaling has decayed to ~1x.
    assert_rescue_climb_rate(fdm, t0, variant)
    descent = band_descent_rate(fdm, t0, 2.0, 7.0)
    log(f"[{variant}] fallback descent: {descent:.2f} m/s (descendRate 0.8)")
    assert 0.5 <= descent <= 1.1, f"fallback descent not held to descendRate: {descent:.2f} m/s"
    return m


SCENARIOS = {
    "baseline": (lambda s, r, f: boot_and_engage(s, r, f), []),
    # FIXED yaw: this scenario validates pure translation control; yaw-coupled
    # flight is mission_yaw's job (SITL's 15-20 Hz position loop wanders under
    # the default VELOCITY yaw during braking)
    "mission_flight": (scenario_mission_flight, ["set ap_yaw_mode = FIXED"]),
    "mission_yaw": (
        scenario_mission_yaw,
        # redirect the default waypoint east so the course demands a 90 deg swing
        [f"waypoint update 0 {HOME_LAT:.7f} {WP_EAST_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none"],
    ),
    "mission_engage_backwards": (
        scenario_mission_engage_backwards,
        [
            # two north legs so the first is a pass-through carrot leg; the nose
            # starts backwards (initial_yaw_deg) in the default VELOCITY yaw mode
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
            f"waypoint insert 1 {WP_NORTH90_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
        ],
        {"initial_yaw_deg": 180.0},
    ),
    "mission_corner": (
        scenario_mission_corner,
        [
            # 60 m north into the corner, then out on a ~130 deg turn
            f"waypoint update 0 {WP_NORTH60_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
            f"waypoint insert 1 {WP_CORNER_LAT:.7f} {WP_CORNER_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
        ],
    ),
    "mission_land": (
        scenario_mission_land,
        [
            "set ap_yaw_mode = FIXED",
            # short north leg, then LAND at an offset so the executor must fly
            # to the landing point before descending
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
            # 5 s pre-descent loiter at the LAND waypoint
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {WP_EAST25_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 land 50 none",
            # SITL's 15-20 Hz position loop wanders during braking; a fatter
            # 3D gate keeps arrival deterministic
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",  # keep the descent short
            "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
        ],
    ),
    # Landing-gain investigation. Not a pass/fail regression test: it prints
    # metrics for an A/B across firmware builds (see horizontal_descent_metrics).
    # The existing LAND scenarios all begin their descent with essentially zero
    # horizontal error, so none of them can show a horizontal-loop effect.
    "mission_land_offset": (
        scenario_mission_land_offset,
        [
            "set ap_yaw_mode = FIXED",   # isolate translation: no yaw coupling in the metric
            # HOLD at 20 m north to settle, then LAND 1.5 m east of that point.
            f"waypoint update 0 {LAND_OFFSET_HOLD_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 300 hold {LAND_OFFSET_HOLD_DURATION_DS} none",
            f"waypoint insert 1 {LAND_OFFSET_HOLD_LAT:.7f} {LAND_OFFSET_LAND_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 300 land 0 none",
            # Default 2 m hold radius: the 1.5 m offset must sit INSIDE the LAND
            # arrival gate so the descent starts on the first tick of the leg.
            # (Other landing scenarios widen this to 4 m for determinism; here
            # the gate width is part of what is being tested.)
            "set ap_waypoint_hold_radius = 200",
            # 1 m/s from ~10 m: ~10 s of descent, long enough for a horizontal
            # oscillation at the expected ~0.3-1 rad/s to show several cycles.
            # A 2 m/s descent (as mission_land uses) is over too fast to tell.
            "set ap_landing_descent_rate = 100",
            "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
            # DEBUG_AUTOPILOT_PID rides in the blackbox artifact: debug[2] is
            # pidP*10 and debug[6] pidF*10 on the earth-frame EAST axis (the
            # offset axis), which is the F:P gain ratio the theory predicts.
            "feature BLACKBOX",
            "set blackbox_device = VIRTUAL",
            "set blackbox_sample_rate = 1/1",
        ],
    ),
    "mission_takeoff": (
        scenario_mission_takeoff,
        [
            "set ap_yaw_mode = FIXED",
            # TAKEOFF's lat/lon are advisory; the climb happens in place
            f"waypoint update 0 {HOME_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 15) * 100)} 500 takeoff 0 none",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 15) * 100)} 500 flyover 0 none",
        ],
    ),
    "mission_orbit": (
        scenario_mission_orbit,
        [
            "set ap_yaw_mode = FIXED",
            # 8 m pattern radius: caps the carrot at 2 m/s (0.25 rad/s), big
            # enough to be unambiguous against SITL's coarse position loop
            "set ap_waypoint_hold_radius = 800",
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 hold 600 orbit",
        ],
    ),
    "mission_figure8": (
        scenario_mission_figure8,
        [
            "set ap_yaw_mode = FIXED",
            "set ap_waypoint_hold_radius = 800",
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 hold 600 figure8",
        ],
    ),
    "rx_disable": (lambda s, r, f: scenario_rx_loss(s, r, f, "DISABLE"), ["set ap_rx_loss_policy = DISABLE"]),
    "rx_continue": (lambda s, r, f: scenario_rx_loss(s, r, f, "CONTINUE"), ["set ap_rx_loss_policy = CONTINUE"]),
    "rx_land": (lambda s, r, f: scenario_rx_loss(s, r, f, "LAND"), ["set ap_rx_loss_policy = LAND"]),
    "geofence_land": (
        lambda s, r, f: scenario_geofence(s, r, f, "LAND"),
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = LAND",
            "set ap_landing_descent_rate = 200",  # keep the descent short
            "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
        ],
    ),
    "geofence_rth": (
        lambda s, r, f: scenario_geofence(s, r, f, "RTH"),
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = RTH",
            "set ap_yaw_mode = FIXED",
            "set gps_rescue_return_alt = 15",     # short climb keeps the return quick
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",
            "set landing_disarm_threshold = 10",
        ],
    ),
    "geofence_rth_rxloss": (
        scenario_geofence_rth_rxloss,
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = RTH",
            "set ap_rx_loss_policy = CONTINUE",
            "rxfail 5 s 1000",  # stage 2 forces the AUTOPILOT switch low
            "set ap_yaw_mode = FIXED",
            "set gps_rescue_return_alt = 15",
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",
            "set landing_disarm_threshold = 10",
        ],
    ),
    "rescue": (
        scenario_rescue_ab,
        RESCUE_CFG,
    ),
    "rescue_heading_recovery": (
        scenario_rescue_heading,
        [*RESCUE_CFG, "set mag_hardware = NONE"],
        {"initial_yaw_deg": 90.0},
    ),
    "rescue_gps_loss": (
        scenario_rescue_gps_loss,
        RESCUE_CFG,
    ),
    "rescue_switch_descent": (
        scenario_rescue_switch_descent,
        [*RESCUE_CFG, "aux 5 46 4 1700 2100 0 0"],   # BOXGPSRESCUE on AUX5
    ),
}


def decode_blackbox_logs(scenario_dir):
    """Best-effort: decode .BFL artifacts when blackbox_decode is available.
    Never gates pass/fail — the trajectory recorder is the authority."""
    if not shutil.which("blackbox_decode"):
        return
    for entry in sorted(os.listdir(scenario_dir)):
        if entry.upper().endswith(".BFL"):
            subprocess.run(["blackbox_decode", os.path.join(scenario_dir, entry)],
                           capture_output=True, check=False)


def run_leg(name, variant, body, extra_cfg, opts, binary, leg_dir):
    os.makedirs(leg_dir)
    sitl = Sitl(binary, leg_dir)
    rc = motors = fdm = poller = None
    try:
        # feed construction can fail (port 9002 bind); it must fail the
        # scenario, not abort the suite
        rc = RcFeed()
        motors = MotorFeed()
        poller = StatusPoller(sitl) if TELEMETRY_PORT else None
        fdm = FdmFeed(motors, initial_yaw_deg=opts.get("initial_yaw_deg", 0.0), status=poller)
        sitl.provision(base_config(extra_cfg))
        sitl.start()
        motors.start()
        if poller:
            poller.start()
        if variant is None:
            return body(sitl, rc, fdm)
        return body(sitl, rc, fdm, variant)
    finally:
        if fdm is not None:
            # Vertical outcome of every run, reported but not asserted. With a
            # force-model plant these are unbounded, so they are the direct
            # measure of a throttle-floor descent; under the old velocity-source
            # plant both were pinned near 1.18 m/s no matter what the FC did.
            log(f"plant: peak sink {fdm.max_sink():.2f} m/s, "
                f"hardest touchdown {fdm.model.touchdown_speed:.2f} m/s, "
                f"max alt {fdm.max_altitude():.1f} m")
            try:
                with open(os.path.join(leg_dir, "traj.csv"), "w") as tf:
                    tf.write("t,east,north,up,ve,vn,vu,heading\n")
                    for sample in fdm.snapshot_history():
                        tf.write(",".join(f"{v:.3f}" for v in sample) + "\n")
            except OSError:
                pass  # diagnostics only; never fail a scenario on this
        for feed in (rc, fdm, motors, poller):
            if feed is not None:
                feed.shutdown()
        sitl.stop()
        decode_blackbox_logs(leg_dir)


def run_scenario(name, binary, workdir, binary_b=None):
    spec = SCENARIOS[name]
    body, extra_cfg = spec[0], spec[1]
    opts = spec[2] if len(spec) > 2 else {}
    scenario_dir = os.path.join(workdir, name)
    shutil.rmtree(scenario_dir, ignore_errors=True)
    os.makedirs(scenario_dir)

    log(f"=== scenario: {name}")
    try:
        if opts.get("ab"):
            if binary_b is None:
                log(f"=== SKIP: {name} (A/B scenario, no --binary-b)")
                return None
            metrics_a = run_leg(name, "A", body, extra_cfg, opts, binary, os.path.join(scenario_dir, "A"))
            metrics_b = run_leg(name, "B", body, extra_cfg, opts, binary_b, os.path.join(scenario_dir, "B"))
            opts["compare"](metrics_a, metrics_b)
        else:
            run_leg(name, None, body, extra_cfg, opts, binary, os.path.join(scenario_dir, "run"))
        log(f"=== PASS: {name}")
        return True
    except (AssertionError, RuntimeError, TimeoutError, OSError) as e:
        log(f"=== FAIL: {name}: {e}")
        return False



def run_scenarios_parallel(names, args, jobs):
    """Fan scenarios out across processes, each in its own network namespace.

    SITL binds its UDP/TCP ports to INADDR_ANY and they are compile-time constants, so two
    instances on one host collide. An unprivileged user+network namespace gives each run a
    private loopback, which sidesteps that without touching the firmware. Unlike --speed this
    changes no timing, so results are identical to a serial run.
    """
    import concurrent.futures

    def one(name):
        cmd = ["unshare", "-rn", "--", "bash", "-c",
               "ip link set lo up; exec \"$@\"", "--",
               sys.executable, os.path.abspath(__file__),
               "--binary", os.path.abspath(args.binary),
               "--scenario", name,
               "--workdir", os.path.join(args.workdir, f"par_{name}"),
               "--telemetry-port", "0",
               "--speed", str(args.speed)]
        if args.binary_b:
            cmd += ["--binary-b", os.path.abspath(args.binary_b)]
        if args.debug_mode:
            cmd += ["--debug-mode", args.debug_mode]
        res = subprocess.run(cmd, capture_output=True, text=True)
        out = res.stdout + res.stderr
        ok = f"PASS  {name}" in out or f"PASS: {name}" in out
        skipped = f"SKIP: {name}" in out
        for line in out.splitlines():
            if "===" in line or "FAIL" in line:
                log(f"[{name}] {line.replace('[harness] ', '')}")
        return name, (None if skipped else ok)

    log(f"--- running {len(names)} scenarios, {jobs} at a time, in private network namespaces")
    results = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        for name, ok in pool.map(one, names):
            results[name] = ok
    return results


def main():
    global VERBOSE, TELEMETRY_PORT, SPEED, DEBUG_MODE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", required=True, help="path to betaflight_SITL.elf (built with USE_FLIGHT_PLAN)")
    ap.add_argument("--binary-b", help="rescue-plan binary (-DENABLE_RESCUE_PLAN=1) for A/B scenarios")
    ap.add_argument("--scenario", default="all", choices=["all"] + list(SCENARIOS))
    ap.add_argument("--workdir", default="/tmp/sitl_harness")
    ap.add_argument("--telemetry-port", type=int, default=TELEMETRY_PORT,
                    help="UDP port for ground-truth JSON telemetry (0 disables)")
    ap.add_argument("--jobs", "-j", type=int, default=1,
                    help="run scenarios in parallel, each in its own network namespace so the "
                         "fixed SITL ports cannot collide. Unlike --speed this costs no control "
                         "fidelity. Needs unprivileged user namespaces (unshare -rn).")
    ap.add_argument("--debug-mode",
                    help="set debug_mode in every scenario config, e.g. AUTOPILOT_ALTITUDE, so "
                         "debug[0..7] land in the blackbox artifact")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="simulated-time multiplier. SITL derives its internal time scale from "
                         "the harness's FDM timestamps, so this speeds up the whole FC. 1.0 = real time.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    VERBOSE = args.verbose
    TELEMETRY_PORT = args.telemetry_port
    DEBUG_MODE = args.debug_mode
    SPEED = max(0.1, args.speed)
    if SPEED != 1.0:
        log(f"--- simulated time x{SPEED:g}")

    os.makedirs(args.workdir, exist_ok=True)
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]

    if args.jobs > 1 and len(names) > 1:
        results = run_scenarios_parallel(names, args, max(1, args.jobs))
    else:
        results = {name: run_scenario(name, args.binary, args.workdir, args.binary_b) for name in names}

    log("--- summary")
    for name, ok in results.items():
        log(f"{'PASS' if ok else 'SKIP' if ok is None else 'FAIL'}  {name}")
    sys.exit(0 if all(ok is not False for ok in results.values()) else 1)


if __name__ == "__main__":
    main()
