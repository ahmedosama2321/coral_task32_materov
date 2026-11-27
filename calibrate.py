"""
Speed calibration for the ROV (run ON THE ROBOT, in the pool, still water).
  python3 calibrate.py
It drives forward at a fixed command for a fixed time, you measure how far it went,
and it saves SPEED_PER_CMD to calibration.json (hardware.py loads it automatically).
Repeat 3 times - the median is saved.
"""
import json, statistics, time
from coral_transplant import Config

CMD, SECONDS = 0.4, 5.0

def main():
    from hardware_pi import PiROV
    cfg = Config(); v = PiROV(cfg)
    vals = []
    for i in range(3):
        input(f"\nRun {i+1}/3: put the ROV at a start mark, hands off, press ENTER (it drives {SECONDS:.0f} s)...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < SECONDS:
            v.io_set_thrusters(CMD, 0, 0, 0); time.sleep(0.05)
        v.io_set_thrusters(0, 0, 0, 0)
        d = float(input("Distance it travelled in metres (measure from the start mark): "))
        vals.append(d / SECONDS / CMD); print(f"  -> {vals[-1]:.3f} m/s per unit command")
    k = statistics.median(vals)
    json.dump({"SPEED_PER_CMD": k}, open("calibration.json", "w"))
    print(f"\nSaved SPEED_PER_CMD = {k:.3f}  (spread {max(vals)-min(vals):.3f}; if > 15 %, repeat)")

if __name__ == "__main__":
    main()
