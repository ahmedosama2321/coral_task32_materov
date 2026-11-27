"""
Ready-to-run backend for a Raspberry Pi ROV. ASSUMED hardware (change the CONFIG block
at the top if yours differs - nothing else needs editing):
  * Thruster ESCs on a PCA9685 PWM board (I2C 0x40), 50 Hz, 1100-1900 us, 1500 = stop
  * BNO055 IMU (I2C) for heading
  * Blue Robotics Bar30 / MS5837 pressure sensor (I2C) for depth
  * Pneumatic gripper released by a solenoid valve on a relay, GPIO pin GRIPPER_PIN
  * USB camera 0 looking down / forward-down
Run:  python3 coral_transplant.py --pi
"""
import math, time
from hardware import CustomROV
import hardware

# ----------------------------- CONFIG ---------------------------------
ESC_CHANNELS = [0, 1, 2, 3, 4, 5, 6, 7]       # PCA9685 channel of each thruster T0..T7
# Mixing matrix: rows = thrusters, columns = [surge, sway, heave, yaw] (-1..1)
# Default: T0-T3 = 4 horizontal vectored thrusters at the corners (FL, FR, RL, RR),
#          T4-T7 = 4 vertical thrusters. Flip a sign if a thruster spins the wrong way.
MIX = [
    [ 1,  1, 0,  1],    # T0 front-left
    [ 1, -1, 0, -1],    # T1 front-right
    [ 1, -1, 0,  1],    # T2 rear-left
    [ 1,  1, 0, -1],    # T3 rear-right
    [ 0,  0, 1,  0],    # T4 vertical
    [ 0,  0, 1,  0],    # T5 vertical
    [ 0,  0, 1,  0],    # T6 vertical
    [ 0,  0, 1,  0],    # T7 vertical
]
US_STOP, US_RANGE = 1500, 400                  # 1100..1900 us
GRIPPER_PIN = 17                               # BCM pin of the valve relay
GRIPPER_ACTIVE_HIGH = True
FLUID_DENSITY = 997.0                          # kg/m3 (fresh water pool)
HEADING_OFFSET_DEG = 0.0                       # mount offset of the IMU
# ----------------------------------------------------------------------

def mix(surge, sway, heave, yaw, matrix=MIX):
    """Pure function (unit-tested): body commands -> per-thruster -1..1 with desaturation."""
    cmd = [m[0]*surge + m[1]*sway + m[2]*heave + m[3]*yaw for m in matrix]
    peak = max(1.0, max(abs(c) for c in cmd))
    return [c/peak for c in cmd]

def to_us(c):
    return int(US_STOP + max(-1, min(1, c)) * US_RANGE)

class PiROV(CustomROV):
    def __init__(self, cfg, camera=None):
        super().__init__(cfg, camera)
        import board, busio
        from adafruit_pca9685 import PCA9685
        import adafruit_bno055, ms5837
        from gpiozero import OutputDevice
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c); self.pca.frequency = 50
        self.imu = adafruit_bno055.BNO055_I2C(i2c)
        self.baro = ms5837.MS5837_30BA(); self.baro.init()
        self.baro.setFluidDensity(FLUID_DENSITY)
        self.valve = OutputDevice(GRIPPER_PIN, active_high=GRIPPER_ACTIVE_HIGH, initial_value=False)
        self._surface_p = None
        for _ in range(20):                       # zero depth NOW: start the script with the ROV at the surface
            if self.baro.read(): self._surface_p = self.baro.pressure(); break
            time.sleep(0.1)
        if self._surface_p is None: raise RuntimeError("pressure sensor not responding")
        self._arm()

    def _write_us(self, ch, us):
        self.pca.channels[ch].duty_cycle = int(us / 20000.0 * 0xFFFF)   # 20 ms period

    def _arm(self):
        for ch in ESC_CHANNELS: self._write_us(ch, US_STOP)
        time.sleep(3)                                                  # ESC arming beep

    def io_read_heading(self):
        e = self.imu.euler
        if e is None or e[0] is None: raise RuntimeError("IMU not ready")
        return (e[0] + HEADING_OFFSET_DEG) % 360

    def io_read_depth(self):
        if not self.baro.read(): raise RuntimeError("pressure sensor read failed")
        return (self.baro.pressure() - self._surface_p) * 100.0 / (FLUID_DENSITY * 9.80665)

    def io_set_thrusters(self, surge, sway, heave, yaw):
        for ch, c in zip(ESC_CHANNELS, mix(surge, sway, heave, yaw)):
            self._write_us(ch, to_us(c))

    def io_gripper(self, release):
        self.valve.on() if release else self.valve.off()

    def shutdown(self):
        for ch in ESC_CHANNELS: self._write_us(ch, US_STOP)
