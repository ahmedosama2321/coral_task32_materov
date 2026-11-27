import random
from coral_transplant import *
def trial(seed):
    r = random.Random(seed); cfg = Config()
    cfg.target_x = r.uniform(1.5, 5.0); cfg.target_y = r.uniform(-1.0, 1.0)
    cfg.start_heading = r.uniform(0, 360)
    v = SimVehicle(cfg, seed=seed, current=(r.uniform(-.05,.05), r.uniform(-.05,.05)),
                   noise=r.uniform(0.005, 0.02))
    # sim pool frame is aligned with start heading; rotate current accordingly
    ok = BrainCoralMission(v, cfg, log=lambda s: None).run()
    return ok and v.coral_on_velcro, v.t
res = [trial(s) for s in range(500)]
n = sum(1 for ok,_ in res if ok)
print(f"{n}/500 successful, max time {max(t for _,t in res):.1f}s, mean {sum(t for _,t in res)/500:.1f}s")
