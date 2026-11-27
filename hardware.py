"""
Backend for the team's CUSTOM ROV (green frame, white enclosure, vectored thrusters,
pneumatic gripper). Fill in the 4 marked IO functions for your electronics - nothing
else in the mission needs to change.

Position: there is no DVL, so position is DEAD-RECKONED from commanded thrust.
Calibrate Config-style constant SPEED_PER_CMD once at the pool:
   command surge=0.4 for 5 s in still water, measure distance D  ->  SPEED_PER_CMD = D/5/0.4
The camera (vision.py) then removes the accumulated error in the final approach.
"""
import math, time
from coral_transplant import Vehicle

SPEED_PER_CMD = 1.0        # m/s per unit command (CALIBRATE! run calibrate.py)
try:                       # calibrate.py writes calibration.json next to this file
    import json, os
    _p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
    if os.path.exists(_p): SPEED_PER_CMD = json.load(open(_p))["SPEED_PER_CMD"]
except Exception:
    pass

class CustomROV(Vehicle):
    def __init__(self, cfg, camera=None):
        self.cfg, self.cam = cfg, camera
        self.ex = self.ey = 0.0; self._h = cfg.start_heading; self._z = 0.0
        self._u = (0, 0, 0, 0); self._t = time.monotonic()

    # ---------------- FILL THESE 4 FOR YOUR ELECTRONICS ----------------
    def io_read_heading(self):            # degrees 0-360 from your IMU / compass
        raise NotImplementedError
    def io_read_depth(self):              # metres, positive down, from pressure sensor
        raise NotImplementedError
    def io_set_thrusters(self, surge, sway, heave, yaw):
        """-1..1 each. Mix into your thruster layout here and send to the ESCs."""
        raise NotImplementedError
    def io_gripper(self, release):        # True = open the pneumatic valve / release
        raise NotImplementedError
    # --------------------------------------------------------------------

    def reset_origin(self):
        """Call when the pilot hands over: current spot = (0,0), current heading = forward."""
        self.ex = self.ey = 0.0; self._t = time.monotonic()

    def state(self):
        now = time.monotonic(); dt = now - self._t; self._t = now
        self._h = self.io_read_heading(); self._z = self.io_read_depth()
        r = math.radians(self._h - self.cfg.start_heading)
        s, w = self._u[0]*SPEED_PER_CMD, self._u[1]*SPEED_PER_CMD
        self.ex += (s*math.cos(r) - w*math.sin(r)) * dt
        self.ey += (s*math.sin(r) + w*math.cos(r)) * dt
        return (self.ex, self.ey, self._z, self._h)

    def command(self, surge, sway, heave, yaw):
        self._u = (surge, sway, heave, yaw)
        self.io_set_thrusters(surge, sway, heave, yaw)
        time.sleep(self.cfg.dt)

    def release_gripper(self): self.io_gripper(True)

    def camera_offset(self):
        return self.cam.offset() if self.cam else None
