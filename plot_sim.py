"""Run one simulated mission and draw the path. Needs: pip install matplotlib
Use:  python plot_sim.py            or     python plot_sim.py 4.0 0.5   (target x, y in metres)"""
import sys, random
import matplotlib.pyplot as plt
from coral_transplant import *
cfg = Config()
if len(sys.argv) >= 3: cfg.target_x, cfg.target_y = float(sys.argv[1]), float(sys.argv[2])
cfg.start_heading = 0
v = SimVehicle(cfg, seed=7, current=(0.03, -0.02), noise=0.015, dead_reckon=True)   # no DVL, like your ROV
path = []; orig = v.command
def spy(*a):
    path.append((v.t, v.x, v.y, v.z)); orig(*a)
v.command = spy
ok = BrainCoralMission(v, cfg, log=print).run()
t, x, y, z = zip(*path)
fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
ax[0].plot(y, x, label="ROV path"); ax[0].plot(0, 0, "go", label="pickup (nursery)")
ax[0].add_patch(plt.Rectangle((cfg.target_y-.075, cfg.target_x-.075), .15, .15, color="red", label="red Velcro"))
ax[0].set_xlabel("right (m)"); ax[0].set_ylabel("forward (m)"); ax[0].axis("equal"); ax[0].legend(); ax[0].set_title("top view")
ax[1].plot(t, z); ax[1].invert_yaxis(); ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("depth (m)"); ax[1].set_title("depth")
fig.suptitle(f"mission {'OK' if ok and v.coral_on_velcro else 'FAILED'}")
plt.tight_layout(); plt.show()
