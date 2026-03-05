class BaseDrive:

    def forward(self, w_fl, w_fr, w_bl, w_br):
        """
        Convert wheel angular velocities to robot velocities.

        Returns:
            vx  (m/s)
            vy  (m/s)
            vth (rad/s)
        """
        raise NotImplementedError("Drive model must implement forward()")
