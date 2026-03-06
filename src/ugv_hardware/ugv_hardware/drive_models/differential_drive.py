
from .base_drive import BaseDrive

class DifferentialDrive(BaseDrive):

    def __init__(self, wheel_base, wheel_radius):
        self.L = wheel_base
        self.r = wheel_radius

    def forward(self, w_left, w_right):

        vx = self.r * (w_left + w_right) / 2.0
        vth = self.r * (w_right - w_left) / self.L
        vy = 0.0

        return vx, vy, vth
