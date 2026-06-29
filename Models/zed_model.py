import pyzed.sl as sl
import math
import time
import logging
import numpy as np
from datetime import datetime


class ZedModel:
    def __init__(self, log=False):
        self.log = log
        if self.log:
            logging.basicConfig(
                filename=f"../Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
                level=logging.DEBUG,
                format='%(asctime)s:%(levelname)s:%(message)s',
            )
            self.logger = logging.getLogger()

        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER

        # Close if left open from a previous session
        self.close()

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            if self.log:
                self.logger.error(f"Failed to open camera: {err}")
            self.zed.close()
            exit(1)

        tracking_params = sl.PositionalTrackingParameters(_init_pos=sl.Transform())
        err = self.zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            if self.log:
                self.logger.warning(f"Failed to enable tracking: {err}")
            self.zed.close()
            exit(1)

        self.zed_pose = sl.Pose()
        self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
        self._prev_pos = self.zed_pose.get_translation(sl.Translation()).get()
        self._prev_time = time.time()

    def close(self):
        if self.zed.is_opened():
            self.zed.disable_spatial_mapping()
            self.zed.close()
            if self.log:
                self.logger.info("Camera closed")

    def get_config(self):
        return self.zed.get_camera_information()

    def get_quaternion(self):
        sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            return sensors_data.get_imu_data().get_pose().get_orientation().get()
        if self.log:
            self.logger.warning("IMU data unavailable")
        return None

    def get_euler(self) -> dict:
        """Returns {'roll', 'pitch', 'yaw'} in radians."""
        x, y, z, w = self.get_quaternion()
        roll  = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        pitch = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        yaw   = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def get_euler_in_degrees(self) -> dict:
        return {k: math.degrees(v) for k, v in self.get_euler().items()}

    def get_pos_global(self) -> np.ndarray | None:
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
            return self.zed_pose.get_translation(sl.Translation()).get()
        return None

    def get_pos_relative(self) -> np.ndarray | None:
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.CAMERA)
            return self.zed_pose.get_translation(sl.Translation()).get()
        return None

    def get_angular_velocity(self) -> list:
        """
        Returns body angular rates [p, q, r] (roll, pitch, yaw) in rad/s.

        ZED RIGHT_HANDED_Y_UP axis mapping:
          raw[0] → X axis → pitch rate (q)
          raw[1] → Y axis → yaw rate   (r)
          raw[2] → Z axis → roll rate  (p)

        Roll and pitch are negated to match the flight-controller sign convention.
        ZED reports angular velocity in deg/s.
        """
        sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            v = sensors_data.get_imu_data().get_angular_velocity()
            return [-math.radians(v[2]), -math.radians(v[0]), math.radians(v[1])]
        return [0.0, 0.0, 0.0]

    def get_state(self) -> np.ndarray | None:
        """
        Grabs one camera frame and returns the full 12-element state vector:
          [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]

        Y is altitude (ZED RIGHT_HANDED_Y_UP). Angles in radians.
        Roll and pitch (and their rates) are negated relative to raw ZED output
        to match the flight-controller sign convention.
        """
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None

        now = time.time()
        dt = now - self._prev_time
        self._prev_time = now

        self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
        pos = self.zed_pose.get_translation(sl.Translation()).get()
        vel = (pos - self._prev_pos) / dt if dt > 0 else np.zeros(3)
        self._prev_pos = pos.copy()

        sensors_data = sl.SensorsData()
        self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
        imu = sensors_data.get_imu_data()

        q = imu.get_pose().get_orientation().get()
        x, y, z, w = q[0], q[1], q[2], q[3]
        roll  = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        pitch = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        yaw   = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))

        v = imu.get_angular_velocity()
        p, q_rate, r = -math.radians(v[2]), -math.radians(v[0]), math.radians(v[1])

        return np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            -roll, -pitch, yaw,
            p, q_rate, r,
        ])
