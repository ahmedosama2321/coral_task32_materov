import random
from coral_transplant import *
def run(seed, cur, noise, **kw):
    r = random.Random(seed); cfg = Config()
    cfg.target_x = r.uniform(1.5, 6.0); cfg.target_y = r.uniform(-1.5, 1.5); cfg.start_heading = r.uniform(0, 360)
    v = SimVehicle(cfg, seed=seed, current=(r.uniform(-cur,cur), r.uniform(-cur,cur)), noise=noise, **kw)
    msgs=[]; ok = BrainCoralMission(v, cfg, log=msgs.append).run()
    return ok, v.coral_on_velcro, v.t, msgs
for name,cur,noise in [("calm",0.0,0.005),("normal",0.05,0.02),("strong current 0.10 m/s",0.10,0.02),("noisy sensors 0.03",0.05,0.03)]:
    res=[run(s,cur,noise) for s in range(300)]
    print(f"{name:26s} {sum(1 for o,c,_,_ in res if o and c)}/300  max {max(t for *_,t,_ in res):.0f}s")
# fail-safe: unreachable target must abort to manual, never claim success
ok,velcro,t,m = run(1,0.0,0.01); cfgbad = Config(); cfgbad.target_x=60; cfgbad.phase_timeout=20
v=SimVehicle(cfgbad,seed=1); msgs=[]; ok=BrainCoralMission(v,cfgbad,log=msgs.append).run()
print("unreachable target ->", ok, msgs[-1])
# fail-safe: telemetry loss
class Dead(SimVehicle):
    def state(self):
        if self.t>5: raise RuntimeError("telemetry lost")
        return super().state()
cfg=Config(); v=Dead(cfg,seed=2); msgs=[]; ok=BrainCoralMission(v,cfg,log=msgs.append).run()
print("telemetry loss ->", ok, msgs[-1])
