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

import sys
import math
import numpy as np
from time import sleep
import time
import logging
from datetime import datetime

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from motor_model import Motors
from rx_model import RX
from lqr_model import LQR
from zed_model import ZedModel


class FlightController:

    # RC stick scaling
    MAX_TILT_RAD   = math.radians(10)   # ±10° max roll/pitch setpoint
    MAX_YAW_RATE   = math.radians(30)   # ±30°/s max yaw-rate setpoint

    def __init__(self, test_mode=True):
        logging.basicConfig(
            filename=f"/home/drone/Autonomous-Drone-Platform/Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
            level=logging.DEBUG,
            format='%(asctime)s:%(levelname)s:%(message)s',
        )
        self.logger = logging.getLogger()

        self.test_mode = True if test_mode == 'test' else False
        self.logger.info(f'Test Mode: {self.test_mode}')

        self.throttle_scale = 0.5
        self.throttle_cutoff = 1012
        self.sample_rate = 50

        if not self.test_mode:
            self.logger.info('Initializing Motors')
            self.motors = Motors()
            self.motor_tests_pass = self.motors.test_motors()
            if not self.motor_tests_pass:
                self.logger.error("Motor Tests Failed")
            else:
                self.logger.info("Motor Tests Passed")

            self.logger.info("Initializing RX")
            self.rx = RX()
            self.logger.info("Successfully initialized RX")

        self.logger.info("Initializing ZED")
        self.zed = ZedModel()
        self.logger.info("Successfully initialized ZED")

        self.logger.info("Initializing LQR")
        self.lqr = LQR()
        self.logger.info(f"LQR gain K shape: {self.lqr.K.shape}")

        self.logger.info("FlightController initialized")


    def outputSpeeds(self, speeds: list):
        reordered_speeds = [
            speeds[0],
            speeds[3],
            speeds[1],
            speeds[2],
        ]
        self.motors.output_speeds(reordered_speeds)


    def run(self):
        self.logger.info('Entering main event loop')
        while True:
            # RC input
            rx_data = self.rx.read()

            # Full 12-state vector from ZED (grabs one camera frame)
            x = self.zed.get_state()
            if x is None:
                continue  # skip frame if ZED grab failed

            # Hover baseline from throttle stick
            throttle_norm   = (rx_data[2] - 1000) / 1000
            hover           = 100 * self.throttle_scale * throttle_norm

            # Reference state: only attitude setpoints from sticks; everything else → 0
            x_ref = np.zeros(12)
            x_ref[6]  = self.MAX_TILT_RAD * (rx_data[0] - 1500) / 500   # phi   (roll, rad)
            x_ref[7]  = self.MAX_TILT_RAD * (rx_data[1] - 1500) / 500   # theta (pitch, rad)
            x_ref[11] = self.MAX_YAW_RATE * (rx_data[3] - 1500) / 500   # r     (yaw rate, rad/s)

            # LQR correction: [T_cmd, roll_cmd, pitch_cmd, yaw_cmd] in motor-speed units
            u = self.lqr.calc(x_ref, x)
            T_cmd, roll_cmd, pitch_cmd, yaw_cmd = u

            # Motor mixing (matches existing hardware layout)
            mSpeeds = [
                hover + T_cmd + pitch_cmd + roll_cmd - yaw_cmd,  # motor 0: front-right
                hover + T_cmd - pitch_cmd + roll_cmd + yaw_cmd,  # motor 1: rear-right
                hover + T_cmd - pitch_cmd - roll_cmd - yaw_cmd,  # motor 2: rear-left
                hover + T_cmd + pitch_cmd - roll_cmd + yaw_cmd,  # motor 3: front-left
            ]

            if rx_data[2] > self.throttle_cutoff:
                self.outputSpeeds(mSpeeds)
            else:
                self.motors.zero_throttle()


    def close(self):
        self.motors.zero_throttle()
        self.zed.close()
