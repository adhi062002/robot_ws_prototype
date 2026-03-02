from .base_drive import BaseDrive


class DifferentialDrive(BaseDrive):
    def __init__(self, wheel_base):
        self.wheel_base = wheel_base

    def compute_wheel_speeds(self, vx, vy, omega):
        # vy ignored in differential drive
        v_left = vx - (omega * self.wheel_base / 2.0)
        v_right = vx + (omega * self.wheel_base / 2.0)

        return {
            "front_left": v_left,
            "rear_left": v_left,
            "front_right": v_right,
            "rear_right": v_right
        }
