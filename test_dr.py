import random
from coral_transplant import *
class NoCam(SimVehicle):
    def camera_offset(self): return None
def run(seed, cam, cur=0.05):
    r = random.Random(seed); cfg = Config()
    cfg.target_x = r.uniform(1.5, 5.0); cfg.target_y = r.uniform(-1.0, 1.0); cfg.start_heading = r.uniform(0, 360)
    V = SimVehicle if cam else NoCam
    v = V(cfg, seed=seed, current=(r.uniform(-cur,cur), r.uniform(-cur,cur)), noise=0.02, dead_reckon=True)
    ok = BrainCoralMission(v, cfg, log=lambda s: None).run()
    return ok and v.coral_on_velcro
for cam in (False, True):
    for cur in (0.05, 0.10):
        n = sum(run(s, cam, cur) for s in range(300))
        print(f"no DVL, camera={'yes' if cam else 'no ':3s} current={cur:.2f} m/s -> {n}/300")
