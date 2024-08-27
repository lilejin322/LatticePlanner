from typing import List
from protoclass.VehicleState import VehicleState
from protoclass.LocalizationEstimate import LocalizationEstimate
from protoclass.Chassis import Chassis
from common.Vec2d import Vec2d
from protoclass.Pose import Pose, Quaternion
from common.Status import Status
from protoclass.Header import ErrorCode
from logging import Logger
from config import FLAGS_reverse_heading_vehicle_state, FLAGS_use_navigation_mode, FLAGS_enable_map_reference_unify, \
                   FLAGS_state_transform_to_com_reverse, FLAGS_state_transform_to_com_drive
from copy import deepcopy
import math
import numpy as np
from scipy.spatial.transform import Rotation

logger = Logger("VehicleStateProvider")

class VehicleStateProvider:
    """
    The class of vehicle state.
    It includes basic information and computation about the state of the vehicle.
    """

    def __init__(self):
        """
        Constructor
        """

        self._vehicle_state: VehicleState = None
        self._original_localization: LocalizationEstimate = None

    def Update(self, *args):

        if isinstance(args[0], LocalizationEstimate) and isinstance(args[1], Chassis):
            """
            Constructor by information of localization and chassis.

            :param localization: Localization information of the vehicle.
            :param chassis: Chassis information of the vehicle.
            """

            localization: LocalizationEstimate = args[0]
            chassis: Chassis= args[1]
            self._original_localization = localization
            if not self.ConstructExceptLinearVelocity(localization):
                msg = "Fail to update because ConstructExceptLinearVelocity error.localization:\n" + f"{localization}"
                return Status(ErrorCode.LOCALIZATION_ERROR, msg)
            if localization.measurement_time is not None:
                self._vehicle_state.timestamp = localization.measurement_time
            elif localization.header.timestamp_sec is not None:
                self._vehicle_state.timestamp = localization.header.timestamp_sec
            elif chassis.header is not None and chassis.header.timestamp_sec is not None:
                logger.error("Unable to use location timestamp for vehicle state. Use chassis time instead.")
                self._vehicle_state.timestamp = chassis.header.timestamp_sec
        
            if chassis.gear_location is not None:
                self._vehicle_state.gear = chassis.gear_location
            else:
                self._vehicle_state.gear = Chassis.GearPosition.GEAR_NONE

            if chassis.speed_mps is not None:
                self._vehicle_state.linear_velocity = chassis.speed_mps
                if not FLAGS_reverse_heading_vehicle_state and self._vehicle_state.gear == Chassis.GearPosition.GEAR_REVERSE:
                    self._vehicle_state.linear_velocity = -self._vehicle_state.linear_velocity

            if chassis.steering_percentage is not None:
                self._vehicle_state.steering_percentage = chassis.steering_percentage
            
            kEpsilon = 0.1
            if abs(self._vehicle_state.linear_velocity) < kEpsilon:
                self._vehicle_state.kappa = 0.0
            else:
                self._vehicle_state.kappa = self._vehicle_state.angular_velocity / self._vehicle_state.linear_velocity
            
            self._vehicle_state.driving_mode = chassis.driving_mode

            return Status.OK()

        if isinstance(args[0], str) and isinstance(args[1], str):
            """
            Update VehicleStateProvider instance by protobuf files.

            :param str localization_file: the localization protobuf file.
            :param str chassis_file: The chassis protobuf file
            """

            localization_file: str = args[0]
            chassis_file: str = args[1]

            # This function has not been implemented in C++ source code
            raise NotImplementedError

    @property
    def timestamp(self) -> float:
        """
        Return the timestamp
        """
        
        return self._vehicle_state.timestamp

    @property
    def pose(self) -> Pose:
        """
        Return the pose of the vehicle

        :returns: The pose of the vehicle
        :rtype: Pose
        """

        return self._vehicle_state.pose

    @property
    def original_pose(self) -> Pose:
        """
        Return the original pose of the vehicle

        :returns: The original pose of the vehicle
        :rtype: Pose
        """

        return self._original_localization.pose

    @property
    def x(self) -> float:
        """
        Get the x-coordinate of vehicle position.
        :returns: The x-coordinate of vehicle position.
        :rtype: float
        """

        return self._vehicle_state.x

    @property
    def y(self) -> float:
        """
        Get the y-coordinate of vehicle position.
        
        :returns: The y-coordinate of vehicle position.
        :rtype: float
        """

        return self._vehicle_state.y

    @property
    def z(self):
        """
        Get the z coordinate of vehicle position.
        
        :returns: The z-coordinate of vehicle position.
        :rtype: float
        """

        return self._vehicle_state.z

    @property
    def kappa(self) -> float:
        """
        Get the kappa of vehicle position.
        the positive or negative sign is decided by the vehicle heading vector
        along the path

        :returns: The kappa of vehicle position.
        :rtype: float
        """

        return self._vehicle_state.kappa

    @property
    def roll(self) -> float:
        """
        Get the vehicle roll angle.

        :returns: The euler roll angle.
        :rtype: float
        """

        return self._vehicle_state.roll

    @property
    def pitch(self) -> float:
        """
        Get the vehicle pitch angle.
        
        :returns: The euler pitch angle.
        :rtype: float
        """

        return self._vehicle_state.pitch

    @property
    def yaw(self) -> float:
        """
        As of now, use the heading instead of yaw angle.
        Heading angle with East as zero, yaw angle has North as zero

        :returns: The euler yaw angle.
        """
        
        return self._vehicle_state.yaw

    @property
    def heading(self) -> float:
        """
        Get the heading of vehicle position, which is the angle
        between the vehicle's heading direction and the x-axis.

        :returns: The angle between the vehicle's heading direction and the x-axis.
        :rtype: float
        """

        return self._vehicle_state.heading

    @property
    def linear_velocity(self) -> float:
        """
        Get the vehicle's linear velocity.
        
        :returns: The vehicle's linear velocity.
        """
        
        return self._vehicle_state.linear_velocity

    @property
    def angular_velocity(self) -> float:
        """
        Get the vehicle's angular velocity.
        
        :returns: The vehicle's angular velocity.
        """

        return self._vehicle_state.angular_velocity

    @property
    def linear_acceleration(self) -> float:
        """
        Get the vehicle's linear acceleration.
        
        :returns: The vehicle's linear acceleration.
        :rtype: float
        """

        return self._vehicle_state.linear_acceleration

    @property
    def gear(self) -> float:
        """
        Get the vehicle's gear position.
        
        :returns: The vehicle's gear position.
        :rtype: float
        """

        return self._vehicle_state.gear

    @property
    def steering_percentage(self) -> float:
        """
        Get the vehicle's steering angle.
        
        :returns: The vehicle's steering angle.
        :rtype: float
        """

        return self._vehicle_state.steering_percentage

    def set_linear_velocity(self, linear_velocity: float) -> None:
        """
        Set the vehicle's linear velocity.
        
        :param float linear_velocity: The value to set the vehicle's linear velocity.
        """

        self._vehicle_state.linear_velocity = linear_velocity

    def EstimateFuturePosition(self, t: float) -> Vec2d:
        """
        Estimate future position from current position and heading,
        along a period of time, by constant linear velocity,
        linear acceleration, angular velocity.

        :param float t: The length of time period.
        :returns: The estimated future position in time t.
        :rtype: Vec2d
        """

        vec_distance = np.array([0.0, 0.0, 0.0])
        v: float = self._vehicle_state.linear_velocity
        # Predict distance travel vector
        if np.abs(self._vehicle_state.angular_velocity) < 0.0001:
            vec_distance[0] = 0.0
            vec_distance[1] = v * t
        else:
            angular_velocity = self._vehicle_state.angular_velocity
            vec_distance[0] = -v / angular_velocity * (1.0 - np.cos(angular_velocity * t))
            vec_distance[1] = np.sin(angular_velocity * t) * v / angular_velocity

        # If we have rotation information, take it into consideration.
        if self._vehicle_state.pose.orientation is not None:
            orientation = self._vehicle_state.pose.orientation
            quaternion = Rotation.from_quat([orientation.qx, orientation.qy, orientation.qz, orientation.qw])
            pos_vec = np.array([self._vehicle_state.x, self._vehicle_state.y, self._vehicle_state.z])
            future_pos_3d = quaternion.apply(vec_distance) + pos_vec
            return Vec2d(future_pos_3d[0], future_pos_3d[1])

        # If no valid rotation information provided from localization,
        # return the estimated future position without rotation.
        return Vec2d(vec_distance[0] + self._vehicle_state.x,
                     vec_distance[1] + self._vehicle_state.y)

    def ComputeCOMPosition(self, rear_to_com_distance: float) -> Vec2d:
        """
        Compute the position of center of mass(COM) of the vehicle,
        given the distance from rear wheels to the center of mass.
        
        :param float rear_to_com_distance: Distance from rear wheels to the vehicle's center of mass.
        :returns: The position of the vehicle's center of mass.
        :rtype: Vec2d
        """

        # set length as distance between rear wheel and center of mass.
        v = np.array([0.0, 0.0, 0.0])
        if (FLAGS_state_transform_to_com_reverse and self._vehicle_state.gear == Chassis.GearPosition.GEAR_REVERSE) or \
           (FLAGS_state_transform_to_com_drive and self._vehicle_state.gear == Chassis.GearPosition.GEAR_DRIVE):
            v = np.array([0.0, rear_to_com_distance, 0.0])
        else:
            v = np.array([0.0, 0.0, 0.0])
        pos_vec = np.array([self._vehicle_state.x, self._vehicle_state.y, self._vehicle_state.z])
        # Initialize the COM position without rotation
        com_pos_3d = v + pos_vec

        # If we have rotation information, take it into consideration.
        if self._vehicle_state.pose.orientation is not None:
            orientation = self._vehicle_state.pose.orientation
            quaternion = Rotation.from_quat([orientation.qx, orientation.qy, orientation.qz, orientation.qw])
            # Update the COM position with rotation
            com_pos_3d = quaternion.apply(v) + pos_vec
        
        return Vec2d(com_pos_3d[0], com_pos_3d[1])

    @property
    def vehicle_state(self) -> VehicleState:
        """
        Return the current vehicle state.

        :returns: The current vehicle state.
        :rtype: VehicleState
        """

        return self._vehicle_state

    def ConstructExceptLinearVelocity(self, localization: LocalizationEstimate) -> bool:
        """
        Construct the vehicle state except for the linear velocity.

        :param LocalizationEstimate localization: Localization estimate.
        :returns: A boolean indicating success.
        :rtype: bool
        """

        if localization.pose is None:
            logger.error("Invalid localization input.")
            return False
        
        # skip localization update when it is in use_navigation_mode.
        if FLAGS_use_navigation_mode:
            logger.debug("Skip localization update when it is in use_navigation_mode.")
            return True

        self._vehicle_state.pose = deepcopy(localization.pose)
        if localization.pose.position is not None:
            self._vehicle_state.x = localization.pose.position.x
            self._vehicle_state.y = localization.pose.position.y
            self._vehicle_state.z = localization.pose.position.z
        
        orientation: Quaternion = localization.pose.orientation
        
        if localization.pose.heading is not None:
            self._vehicle_state.heading = localization.pose.heading
        else:
            self._vehicle_state.heading = QuaternionToHeading(orientation.qw, orientation.qx,
                                                              orientation.qy, orientation.qz)

        if FLAGS_enable_map_reference_unify:
            if localization.pose.angular_velocity_vrf is None:
                logger.error("localization.pose().angular_velocity_vrf must not be None when FLAGS_enable_map_reference_unify is true.")
                return False
            self._vehicle_state.angular_velocity = localization.pose.angular_velocity_vrf.z

            if localization.pose.linear_acceleration_vrf is None:
                logger.error("localization.pose().linear_acceleration_vrf must not be None when FLAGS_enable_map_reference_unify is true.")
                return False

            self._vehicle_state.linear_acceleration = localization.pose.linear_acceleration_vrf.y
        else:
            if localization.pose.angular_velocity is None:
                logger.error("localization.pose() has no angular velocity.")
                return False
            self._vehicle_state.angular_velocity = localization.pose.angular_velocity.z

            if localization.pose.linear_acceleration is None:
                logger.error("localization.pose() has no linear acceleration.")
                return False
            self._vehicle_state.linear_acceleration = localization.pose.linear_acceleration.y
        if localization.pose.euler_angles is not None:
            self._vehicle_state.roll = localization.pose.euler_angles.y
            self._vehicle_state.pitch = localization.pose.euler_angles.x
            self._vehicle_state.yaw = localization.pose.euler_angles.z
        else:
            self._vehicle_state.roll = math.atan2(2 * (orientation.qw * orientation.qy - orientation.qx * orientation.qz),
                                                  2 * (orientation.qw**2 + orientation.qz**2) - 1)
            self._vehicle_state.pitch = math.asin(2 * (orientation.qw * orientation.qx + orientation.qy * orientation.qz))
            self._vehicle_state.yaw = math.atan2(2 * (orientation.qw * orientation.qz - orientation.qx * orientation.qy),
                                                 2 * (orientation.qw**2 + orientation.qy**2) - 1)

        return True
