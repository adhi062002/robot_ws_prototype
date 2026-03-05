from .base_drive import BaseDrive


class MecanumDrive(BaseDrive):

    def __init__(self, wheel_radius, length, width):

        self.r = wheel_radius
        self.L = length
        self.W = width

    def forward(self, w_fl, w_fr, w_bl, w_br):

        r = self.r
        l_plus_w = self.L + self.W

        vx = (w_fl + w_fr + w_bl + w_br) * (r / 4.0)

        vy = (-w_fl + w_fr + w_bl - w_br) * (r / 4.0)

        vth = (-w_fl + w_fr - w_bl + w_br) * (r / (4.0 * l_plus_w))

        return vx, vy, vth 
