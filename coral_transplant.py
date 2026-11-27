#!/usr/bin/env python3
"""
MATE ROV 2024 - EXPLORER - Task 3.2 Coral Restoration
Autonomous transplant of the BRAIN CORAL (30 pts) from the nursery to the
red Velcro square on the coral restoration area.

Rules this program follows (manual p.30-31):
  * The pilot picks the coral up by hand (manual control), then presses ENTER.
  * From that moment NO human touches the controls - this program flies.
  * Nursery -> restoration area -> place coral on red Velcro -> release.
  * One autonomous attempt only. If anything fails the program stops thrusters
    and hands back to the pilot (no autonomous points, manual = 10 pts).

Hardware layer is abstract:
  --sim       : built-in physics simulator (no hardware needed)
  --mavlink   : ArduSub / BlueROV2-style flight controller via pymavlink
Every distance / heading / depth is in Config - measure them at the pool.
"""
import argparse, math, random, sys, time
from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# Configuration - EDIT THESE AT THE POOL (all metres / degrees / seconds)
# --------------------------------------------------------------------------
@dataclass
class Config:
    # Mission geometry, measured from the pickup point (nursery) in the
    # pool frame. x = forward along start heading, y = right, z = depth(+down)
    target_x: float = 3.0          # distance nursery -> above red Velcro (fwd)
    target_y: float = 0.0          # sideways offset
    cruise_depth: float = 1.2      # carry depth (keep coral clear of frames)
    place_depth: float = 1.8       # depth at which bowl touches the Velcro
    start_heading: float = 0.0     # heading (deg) when coral is picked up
    # Control
    pos_tol: float = 0.06          # m, horizontal arrival tolerance
    depth_tol: float = 0.05
    heading_tol: float = 3.0
    max_speed: float = 0.40        # thrust-command cap (~0.4 m/s) - coral on a rope can swing
    kp_pos: float = 0.6
    kp_depth: float = 0.6
    kp_head: float = 0.02
    ki_pos: float = 0.25           # integral: cancels steady current / thruster bias
    i_max: float = 0.3
    settle_time: float = 1.5       # s inside tolerance before advancing
    press_time: float = 2.0        # s gentle downward push so Velcro bites
    press_thrust: float = 0.08
    phase_timeout: float = 45.0    # s per phase, then abort to manual
    total_timeout: float = 150.0   # product demo time safety
    dt: float = 0.05


# --------------------------------------------------------------------------
# Vehicle interfaces
# --------------------------------------------------------------------------
class Vehicle:
    """State: x,y (m, pool frame from pickup point), depth (m), heading (deg)."""
    def state(self): raise NotImplementedError
    def command(self, surge, sway, heave, yaw): raise NotImplementedError  # -1..1
    def release_gripper(self): raise NotImplementedError
    def camera_offset(self):
        """(forward, right) metres from vehicle to the red Velcro centre, or None if not seen."""
        return None
    def stop(self): self.command(0, 0, 0, 0)


class SimVehicle(Vehicle):
    """Simple 4-DOF model with drag, thruster lag, current and sensor noise."""
    def __init__(self, cfg, seed=0, current=(0.0, 0.0), noise=0.01, grip_ok=True, dead_reckon=False):
        r = random.Random(seed); self.r = r; self.cfg = cfg
        self.x = self.y = 0.0
        self.z = cfg.cruise_depth * 0.5 + r.uniform(-0.2, 0.2)
        self.h = cfg.start_heading + r.uniform(-15, 15)
        self.vx = self.vy = self.vz = self.vh = 0.0
        self.u = [0, 0, 0, 0]
        self.cur = current; self.noise = noise
        self.coral_attached = True; self.coral_on_velcro = False
        self.t = 0.0; self.grip_ok = grip_ok
        self.sway_drift = r.uniform(-0.04, 0.04)   # imperfect thruster balance
        # dead-reckoning (no DVL): position estimated from thrust command x calibrated speed
        self.dead_reckon = dead_reckon
        self.cam_range = 1.5                       # m: how far away the camera can see the red square
        self.ex = self.ey = 0.0
        self.k_cal = (2.0/1.8) * (1 + r.uniform(-0.12, 0.12))   # +-12 % speed-calibration error
    def _step(self, dt):
        a = 0.3 ** (dt / 0.2)                       # thruster lag
        self.u = [a*p + (1-a)*c for p, c in zip(self.u, self._cmd)]
        rad = math.radians(self.h - self.cfg.start_heading)
        fx, fy = self.u[0]*2.0, self.u[1]*2.0 + self.sway_drift
        ax = fx*math.cos(rad) - fy*math.sin(rad)
        ay = fx*math.sin(rad) + fy*math.cos(rad)
        drag = 1.8
        self.vx += (ax - drag*(self.vx - self.cur[0])) * dt
        self.vy += (ay - drag*(self.vy - self.cur[1])) * dt
        self.vz += (self.u[2]*2.0 - drag*self.vz) * dt
        self.vh += (self.u[3]*90 - 3.0*self.vh) * dt
        self.x += self.vx*dt; self.y += self.vy*dt
        if self.dead_reckon:
            self.ex += (self.u[0]*math.cos(rad) - self.u[1]*math.sin(rad)) * self.k_cal * dt
            self.ey += (self.u[0]*math.sin(rad) + self.u[1]*math.cos(rad)) * self.k_cal * dt
        self.z = max(0.0, self.z + self.vz*dt)
        # the Velcro sheet physically stops the bowl (contact model)
        if abs(self.x - self.cfg.target_x) < 0.12 and abs(self.y - self.cfg.target_y) < 0.12:
            if self.z > self.cfg.place_depth + 0.02:
                self.z = self.cfg.place_depth + 0.02; self.vz = min(self.vz, 0)
        self.h = (self.h + self.vh*dt) % 360
        self.t += dt
    _cmd = [0, 0, 0, 0]
    def state(self):
        n = lambda s: random.gauss(0, self.noise*s)
        if self.dead_reckon:
            return (self.ex, self.ey, self.z + n(0.5), (self.h + n(50)) % 360)
        return (self.x + n(1), self.y + n(1), self.z + n(0.5), (self.h + n(50)) % 360)
    def camera_offset(self):
        dx, dy = self.cfg.target_x - self.x, self.cfg.target_y - self.y
        if math.hypot(dx, dy) > self.cam_range: return None            # Velcro not in the field of view
        r = math.radians(self.h - self.cfg.start_heading)
        fwd = dx*math.cos(r) + dy*math.sin(r); right = -dx*math.sin(r) + dy*math.cos(r)
        return (fwd + random.gauss(0, 0.01), right + random.gauss(0, 0.01))
    def command(self, surge, sway, heave, yaw):
        self._cmd = [max(-1, min(1, v)) for v in (surge, sway, heave, yaw)]
        self._step(self.cfg.dt)
    def release_gripper(self):
        if not self.coral_attached: return
        self.coral_attached = False
        cfg = self.cfg
        # Coral is on the Velcro if released within the 15 cm square, at the
        # place depth, and the vehicle was gently pressing down.
        dx, dy = self.x - cfg.target_x, self.y - cfg.target_y
        ok = abs(dx) < 0.075 + 0.05 and abs(dy) < 0.075 + 0.05 \
             and abs(self.z - cfg.place_depth) < 0.12 and self.grip_ok
        self.coral_on_velcro = ok


class MavlinkVehicle(Vehicle):
    """ArduSub (BlueROV2) backend. Needs: pip install pymavlink.
    Position is dead-reckoned from the Kalman-filtered LOCAL_POSITION_NED
    (needs DVL / optical flow / USBL) - if you have none, use timed surge
    (see Config + calibrate speed) and set use_dead_reckon=True."""
    def __init__(self, cfg, conn="udpin:0.0.0.0:14550", gripper_servo=9,
                 grip_open_pwm=1900, grip_closed_pwm=1100):
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(conn); self.m.wait_heartbeat()
        self.cfg = cfg; self.servo = gripper_servo
        self.open_pwm, self.closed_pwm = grip_open_pwm, grip_closed_pwm
        self.origin = None; self.mavutil = mavutil
        self.m.arducopter_arm(); self.m.motors_armed_wait()
        self.m.set_mode_apm("MANUAL")
        self._last = (0, 0, 0, 0)
    def state(self):
        p = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        a = self.m.recv_match(type="VFR_HUD", blocking=True, timeout=1)
        if p is None or a is None: raise RuntimeError("telemetry lost")
        if self.origin is None: self.origin = (p.x, p.y)
        dx, dy = p.x - self.origin[0], p.y - self.origin[1]   # NED
        h0 = math.radians(self.cfg.start_heading)
        fwd = dx*math.cos(h0) + dy*math.sin(h0)
        right = -dx*math.sin(h0) + dy*math.cos(h0)
        return (fwd, right, p.z, a.heading)
    def command(self, surge, sway, heave, yaw):
        c = lambda v: int(max(-1, min(1, v)) * 1000)
        # MANUAL_CONTROL: z 0..1000 (500 = neutral); positive = ascend
        self.m.mav.manual_control_send(self.m.target_system, c(surge), c(sway),
                                       500 - c(heave)//2, c(yaw), 0)
        time.sleep(self.cfg.dt)
    def release_gripper(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
            self.mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, self.servo,
            self.open_pwm, 0, 0, 0, 0, 0)


# --------------------------------------------------------------------------
# Autonomous mission
# --------------------------------------------------------------------------
def wrap180(a): return (a + 180) % 360 - 180

class Abort(Exception): pass

class BrainCoralMission:
    PHASES = ["LIFT", "TRANSIT", "ALIGN", "DESCEND", "PRESS", "RELEASE", "BACKOFF"]

    def __init__(self, veh, cfg, log=print):
        self.v, self.c, self.log = veh, cfg, log
        self.t0 = time.monotonic(); self.simt = 0.0
        self.seen = False
        self._f = None; self.alpha = 0.25          # low-pass for noisy sensors

    def fstate(self):
        """Low-pass filtered (x, y, z, heading). Heading filtered via its error."""
        x, y, z, h = self.v.state()
        if self._f is None: self._f = [x, y, z, h]
        else:
            a = self.alpha
            self._f[0] += a*(x - self._f[0]); self._f[1] += a*(y - self._f[1])
            self._f[2] += a*(z - self._f[2])
            self._f[3] = (self._f[3] + a*wrap180(h - self._f[3])) % 360
        return tuple(self._f)

    # time source (sim runs faster than real time)
    def now(self):
        return self.v.t if isinstance(self.v, SimVehicle) else time.monotonic() - self.t0

    def clamp(self, v, m): return max(-m, min(m, v))

    def heading_cmd(self, h):
        return self.clamp(self.c.kp_head * wrap180(self.c.start_heading - h), 1.0)

    def depth_cmd(self, z, target):
        return self.clamp(self.c.kp_depth * (target - z), 1.0)

    def goto(self, phase, tx, ty, tz, check_pos=True, vision=False, stop_on_seen=False, tol=None):
        """Hold heading + depth, drive to (tx,ty) in the pool frame."""
        c = self.c; t_phase = self.now(); inside_since = None; ix = iy = 0.0
        while True:
            t = self.now()
            if t - t_phase > c.phase_timeout: raise Abort(f"{phase}: timeout")
            if t > c.total_timeout: raise Abort("total timeout")
            x, y, z, h = self.fstate()
            ex, ey = tx - x, ty - y
            r = math.radians(h - c.start_heading)
            bx =  ex*math.cos(r) + ey*math.sin(r)
            by = -ex*math.sin(r) + ey*math.cos(r)
            if vision:                                    # camera sees the red Velcro: trust it
                off = self.v.camera_offset()
                if off is not None:
                    bx, by = off
                    ex, ey = bx, by                       # tolerance check on camera error
                    if not self.seen: self.log("  vision lock on red Velcro"); self.seen = True
                    if stop_on_seen: return
            if math.hypot(bx, by) < 0.5:                  # integrate only when close (body frame)
                ix = self.clamp(ix + bx*c.dt, c.i_max/c.ki_pos)
                iy = self.clamp(iy + by*c.dt, c.i_max/c.ki_pos)
            bx, by = bx + c.ki_pos/c.kp_pos*ix, by + c.ki_pos/c.kp_pos*iy
            sp = c.max_speed
            surge = self.clamp(c.kp_pos*bx, sp) if check_pos else 0
            sway  = self.clamp(c.kp_pos*by, sp) if check_pos else 0
            # slow down for the last 0.5 m so the coral does not swing
            self.v.command(surge, sway, self.depth_cmd(z, tz), self.heading_cmd(h))
            ok = (math.hypot(ex, ey) < (tol or c.pos_tol) or not check_pos) \
                 and abs(tz - z) < c.depth_tol and abs(wrap180(c.start_heading - h)) < c.heading_tol
            if ok:
                inside_since = inside_since or t
                if t - inside_since >= c.settle_time: return
            else:
                inside_since = None

    def search(self):
        """If the red square is not visible after TRANSIT (dead-reckoning drift), sweep an
        expanding square around the estimated target until the camera locks on."""
        c = self.c
        if self.v.camera_offset() is not None: return
        self.log("PHASE SEARCH    - red Velcro not visible, expanding-square search")
        pts = []
        for ring in range(1, 5):
            d = 0.5 * ring
            pts += [(d, 0), (d, d), (-d, d), (-d, -d), (d, -d)]
        self.seen = False
        for ox, oy in pts:
            self.goto("SEARCH", c.target_x + ox, c.target_y + oy, c.cruise_depth,
                      vision=True, stop_on_seen=True, tol=0.2)
            if self.seen: return
        raise Abort("SEARCH: red Velcro never found")

    def run(self):
        c = self.c
        try:
            self.log("PHASE LIFT      - rise to carry depth")
            x, y, _, _ = self.fstate()
            self.goto("LIFT", x, y, c.cruise_depth, check_pos=False)
            self.log("PHASE TRANSIT   - nursery -> restoration area")
            self.goto("TRANSIT", c.target_x, c.target_y, c.cruise_depth)
            self.search()
            self.log("PHASE ALIGN     - fine position over Velcro")
            self.goto("ALIGN", c.target_x, c.target_y, c.cruise_depth, vision=True)
            self.log("PHASE DESCEND   - lower onto Velcro")
            self.goto("DESCEND", c.target_x, c.target_y, c.place_depth, vision=True)
            self.log("PHASE PRESS     - gentle press so hooks grab loops")
            t = self.now()
            while self.now() - t < c.press_time:
                x, y, z, h = self.fstate()
                off = self.v.camera_offset() or (0.0, 0.0)     # keep centred on the Velcro
                self.v.command(0.5*off[0], 0.5*off[1], c.press_thrust, self.heading_cmd(h))
            self.v.stop()
            self.log("PHASE RELEASE   - open gripper")
            self.v.release_gripper()
            for _ in range(int(1.0 / c.dt)): self.v.command(0, 0, 0, 0)
            self.log("PHASE BACKOFF   - rise straight up, coral stays")
            x, y, _, _ = self.fstate()
            self.goto("BACKOFF", x, y, c.cruise_depth, check_pos=False)
            self.v.stop()
            self.log("MISSION COMPLETE (autonomous)")
            return True
        except Abort as e:
            self.v.stop()
            self.log(f"ABORT: {e}  -> tell the judge, switch to MANUAL (10 pts)")
            return False
        except Exception as e:             # telemetry loss etc. - always fail safe
            try: self.v.stop()
            except Exception: pass
            self.log(f"FAULT: {e}  -> MANUAL")
            return False


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", action="store_true"); g.add_argument("--mavlink")
    g.add_argument("--custom", action="store_true", help="team ROV: hardware.py + camera")
    g.add_argument("--pi", action="store_true", help="Raspberry Pi ROV: hardware_pi.py + camera")
    ap.add_argument("--x", type=float); ap.add_argument("--y", type=float)
    a = ap.parse_args()
    cfg = Config()
    if a.x is not None: cfg.target_x = a.x
    if a.y is not None: cfg.target_y = a.y
    if a.sim:
        veh = SimVehicle(cfg, seed=1)
    elif a.pi:
        from hardware_pi import PiROV; from vision import Camera
        veh = PiROV(cfg, Camera(0))
        input("Pilot: pick the brain coral up, tell the judge, then press ENTER "
              "and take hands OFF the controls...")
        cfg.start_heading = veh.io_read_heading()      # "forward" = the way the ROV faces now
        veh.reset_origin()                             # this spot = (0, 0)
        print(f"Start heading {cfg.start_heading:.0f} deg, origin set. Target = "
              f"{cfg.target_x} m forward, {cfg.target_y} m right.")
    elif a.custom:
        from hardware import CustomROV; from vision import Camera
        veh = CustomROV(cfg, Camera(0))
        input("Pilot: pick the brain coral up, tell the judge, then press ENTER "
              "and take hands OFF the controls...")
        cfg.start_heading = veh.io_read_heading()      # "forward" = the way the ROV faces now
        veh.reset_origin()                             # this spot = (0, 0)
        print(f"Start heading {cfg.start_heading:.0f} deg, origin set. Target = "
              f"{cfg.target_x} m forward, {cfg.target_y} m right.")
    else:
        veh = MavlinkVehicle(cfg, a.mavlink)
        input("Pilot: pick the brain coral up, tell the judge, then press ENTER "
              "and take hands OFF the controls...")
        cfg.start_heading = veh.io_read_heading()      # "forward" = the way the ROV faces now
        veh.reset_origin()                             # this spot = (0, 0)
        print(f"Start heading {cfg.start_heading:.0f} deg, origin set. Target = "
              f"{cfg.target_x} m forward, {cfg.target_y} m right.")
    ok = BrainCoralMission(veh, cfg).run()
    if a.sim: print("coral on velcro:", veh.coral_on_velcro)
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
