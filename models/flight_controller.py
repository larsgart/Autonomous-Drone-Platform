r'''
CHANNEL MAPPINGS

   CH1 (ROLL): LEFT = 1000, RIGHT = 2000
   CH2 (PITCH): DOWN = 1000, UP = 2000
   CH3 (THROTTLE): DOWN = 1000, UP = 2000
   CH4 (YAW): LEFT = 1000, RIGHT = 2000

MOTOR MAPPINGS

  3  ^  0
   \_|_/
   |   |
   |___|
   /   \
  2     1

'''

import math
import logging
import numpy as np

from .motors import Motors
from .rx import RX
from .lqr import LQR
from .zed import Zed

log = logging.getLogger(__name__)


class FlightController:

    MAX_FORWARD_VEL = 2.0             # m/s  ±forward/backward velocity
    MAX_LATERAL_VEL = 2.0             # m/s  ±lateral velocity
    MAX_VERT_VEL    = 1.0             # m/s  ±climb/descent rate
    MAX_YAW_RATE    = math.radians(30)  # rad/s ±yaw rate (unchanged)

    def __init__(self, test_mode=False):
        self.test_mode = test_mode == 'test'
        self.throttle_scale = 0.5
        self.throttle_cutoff = 1012

        if not self.test_mode:
            self.motors = Motors()
            if not self.motors.test_motors():
                log.error("Motor tests failed")
            self.rx = RX()

        self.zed = Zed()
        self.lqr = LQR()

    def run(self):
        log.info("Entering main event loop")
        while True:
            rx_data = self.rx.read()

            x = self.zed.get_state()
            if x is None:
                continue

            hover = 50 * self.throttle_scale  # fixed feedforward to approximately offset gravity

            # Body-frame stick inputs → world-frame velocity setpoints.
            # ZED frame: X=right, Y=up, Z=backward. Positive psi = CCW yaw (left turn).
            psi   = x[8]
            v_fwd = self.MAX_FORWARD_VEL * (rx_data[1] - 1500) / 500  # +ve = forward
            v_lat = self.MAX_LATERAL_VEL * (rx_data[0] - 1500) / 500  # +ve = right

            x_ref = np.zeros(12)
            x_ref[3]  = -v_fwd * math.sin(psi) + v_lat * math.cos(psi)  # vx (world)
            x_ref[4]  =  self.MAX_VERT_VEL * (rx_data[2] - 1500) / 500  # vy (climb rate)
            x_ref[5]  = -v_fwd * math.cos(psi) - v_lat * math.sin(psi)  # vz (world, Z=backward)
            x_ref[8]  =  x[8]                                             # hold current yaw; rate-only yaw control
            x_ref[11] =  self.MAX_YAW_RATE * (rx_data[3] - 1500) / 500  # r (yaw rate)

            T_cmd, roll_cmd, pitch_cmd, yaw_cmd = self.lqr.calc(x_ref, x)

            m_speeds = np.clip([
                hover + T_cmd + pitch_cmd + roll_cmd - yaw_cmd,  # motor 0: front-right
                hover + T_cmd - pitch_cmd + roll_cmd + yaw_cmd,  # motor 1: rear-right
                hover + T_cmd - pitch_cmd - roll_cmd - yaw_cmd,  # motor 2: rear-left
                hover + T_cmd + pitch_cmd - roll_cmd + yaw_cmd,  # motor 3: front-left
            ], 0, 100).tolist()

            if rx_data[2] > self.throttle_cutoff:
                self.motors.output_speeds(m_speeds)
            else:
                self.motors.zero_throttle()

    def close(self):
        self.motors.zero_throttle()
        self.zed.close()
