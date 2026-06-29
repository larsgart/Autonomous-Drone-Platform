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

    MAX_TILT_RAD = math.radians(10)  # ±10° max roll/pitch setpoint
    MAX_YAW_RATE = math.radians(30)  # ±30°/s max yaw-rate setpoint

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

    def _output_speeds(self, speeds: list):
        reordered = [speeds[0], speeds[3], speeds[1], speeds[2]]
        self.motors.output_speeds(reordered)

    def run(self):
        log.info("Entering main event loop")
        while True:
            rx_data = self.rx.read()

            x = self.zed.get_state()
            if x is None:
                continue

            throttle_norm = (rx_data[2] - 1000) / 1000
            hover = 100 * self.throttle_scale * throttle_norm

            x_ref = np.zeros(12)
            x_ref[6]  = self.MAX_TILT_RAD * (rx_data[0] - 1500) / 500  # phi   (roll)
            x_ref[7]  = self.MAX_TILT_RAD * (rx_data[1] - 1500) / 500  # theta (pitch)
            x_ref[11] = self.MAX_YAW_RATE * (rx_data[3] - 1500) / 500  # r     (yaw rate)

            T_cmd, roll_cmd, pitch_cmd, yaw_cmd = self.lqr.calc(x_ref, x)

            m_speeds = [
                hover + T_cmd + pitch_cmd + roll_cmd - yaw_cmd,  # motor 0: front-right
                hover + T_cmd - pitch_cmd + roll_cmd + yaw_cmd,  # motor 1: rear-right
                hover + T_cmd - pitch_cmd - roll_cmd - yaw_cmd,  # motor 2: rear-left
                hover + T_cmd + pitch_cmd - roll_cmd + yaw_cmd,  # motor 3: front-left
            ]

            if rx_data[2] > self.throttle_cutoff:
                self._output_speeds(m_speeds)
            else:
                self.motors.zero_throttle()

    def close(self):
        self.motors.zero_throttle()
        self.zed.close()
