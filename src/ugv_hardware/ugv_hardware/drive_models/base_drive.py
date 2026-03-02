class BaseDrive:
    def compute_wheel_speeds(self, vx, vy, omega):
        raise NotImplementedError("Drive model must implement compute_wheel_speeds")
