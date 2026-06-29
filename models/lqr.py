import numpy as np
from scipy.linalg import solve_continuous_are


class LQR:
    """
    Continuous-time LQR for a quadrotor stabilized around hover.

    State (12):   [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    Control (4):  [T_cmd, roll_cmd, pitch_cmd, yaw_cmd]

    Coordinate frame: ZED RIGHT_HANDED_Y_UP — Y is altitude.
    Angles/rates use the flight-controller sign convention (roll and pitch
    are negated relative to raw ZED output; see Zed.get_state()).

    Control outputs are in motor-speed units (0–100 scale) so they feed
    directly into the motor mixer without an additional scaling step.
    """

    G = 9.81  # m/s²

    # Effective physical response per unit of motor command (0–100 scale).
    # These encode physically reasonable starting values but require in-flight tuning.
    # COLLECTIVE_ACCEL: altitude acceleration per unit of collective thrust.
    #   Assumes full throttle (100 units) produces ~2g of thrust → 2g/100 per unit.
    # ROLL/PITCH_ACCEL: angular acceleration per unit of attitude command — placeholder.
    # YAW_ACCEL: smaller because yaw inertia is larger and authority is lower.
    COLLECTIVE_ACCEL = 2 * G / 100   # m/s²  per motor-speed unit (≈ 0.196)
    ROLL_ACCEL       = 0.50           # rad/s² per motor-speed unit
    PITCH_ACCEL      = 0.50           # rad/s² per motor-speed unit
    YAW_ACCEL        = 0.10           # rad/s² per motor-speed unit

    def __init__(self, Q=None, R=None):
        A, B = self._build_system()
        Q = Q if Q is not None else self._default_Q()
        R = R if R is not None else self._default_R()
        self.K = self._compute_gain(A, B, Q, R)

    def _build_system(self):
        g = self.G
        A = np.zeros((12, 12))

        # position ← velocity
        A[0, 3] = 1.0
        A[1, 4] = 1.0
        A[2, 5] = 1.0

        # horizontal velocity ← attitude (small-angle, ZED frame)
        # X is lateral-right, Z is backward; signs may need verification on hardware.
        A[3, 6] =  g   # dvx/dt = g·phi   (rolling right → rightward acceleration)
        A[5, 7] = -g   # dvz/dt = -g·theta (pitching forward → forward / -Z acceleration)

        # attitude ← angular rates
        A[6,  9] = 1.0
        A[7, 10] = 1.0
        A[8, 11] = 1.0

        B = np.zeros((12, 4))
        B[4,  0] = self.COLLECTIVE_ACCEL  # dvy/dt  (altitude, Y-up)
        B[9,  1] = self.ROLL_ACCEL        # dp/dt
        B[10, 2] = self.PITCH_ACCEL       # dq/dt
        B[11, 3] = self.YAW_ACCEL         # dr/dt

        return A, B

    def _default_Q(self):
        # State cost: [x, y(alt), z, vx, vy(alt), vz, phi, theta, psi, p, q, r]
        # Attitude is heavily penalized; position lightly penalized initially.
        return np.diag([
            1.0,  2.0,  1.0,    # position (altitude weighted higher)
            0.5,  1.0,  0.5,    # velocity
            10.0, 10.0, 1.0,    # attitude (roll/pitch critical; yaw softer)
            1.0,  1.0,  0.5,    # angular rates
        ])

    def _default_R(self):
        # Control cost: [T_cmd, roll_cmd, pitch_cmd, yaw_cmd]
        return np.diag([1.0, 1.0, 1.0, 2.0])

    def _compute_gain(self, A, B, Q, R):
        P = solve_continuous_are(A, B, Q, R)
        return np.linalg.inv(R) @ B.T @ P

    def calc(self, x_ref, x):
        """
        Returns u = -K(x − x_ref): [T_cmd, roll_cmd, pitch_cmd, yaw_cmd]
        in motor-speed units, ready for the motor mixer.
        """
        return -self.K @ (x - x_ref)
