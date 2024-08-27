from typing import List
from protoclass.VehicleState import VehicleState
from protoclass.LocalizationEstimate import LocalizationEstimate
from protoclass.Chassis import Chassis
from common.Vec2d import Vec2d

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
            raise NotImplementedError
        
        if isinstance(args[0], str) and isinstance(args[1], str):
            """
            Update VehicleStateProvider instance by protobuf files.

            :param str localization_file: the localization protobuf file.
            :param str chassis_file: The chassis protobuf file
            """

            localization_file: str = args[0]
            chassis_file: str = args[1]
            raise NotImplementedError

    def timestamp(self) -> float:
        """
        Return the timestamp
        """
        
        raise NotImplementedError

    def pose(self) -> Pose:
        """
        Return the pose of the vehicle

        :returns: The pose of the vehicle
        :rtype: Pose
        """

        raise NotImplementedError

    def original_pose(self) -> Pose:
        """
        Return the original pose of the vehicle

        :returns: The original pose of the vehicle
        :rtype: Pose
        """

        raise NotImplementedError

    def x(self) -> float:
        """
        Get the x-coordinate of vehicle position.
        :returns: The x-coordinate of vehicle position.
        :rtype: float
        """

        raise NotImplementedError

    def y(self) -> float:
        """
        Get the y-coordinate of vehicle position.
        
        :returns: The y-coordinate of vehicle position.
        :rtype: float
        """

        raise NotImplementedError

    def z(self):
        """
        Get the z coordinate of vehicle position.
        
        :returns: The z-coordinate of vehicle position.
        :rtype: float
        """

        raise NotImplementedError

    def kappa(self) -> float:
        """
        Get the kappa of vehicle position.
        the positive or negative sign is decided by the vehicle heading vector
        along the path

        :returns: The kappa of vehicle position.
        :rtype: float
        """

        raise NotImplementedError

    def roll(self) -> float:
        """
        Get the vehicle roll angle.
        
        :returns: The euler roll angle.
        :rtype: float
        """

        raise NotImplementedError

    def pitch(self) -> float:
        """
        Get the vehicle pitch angle.
        
        :returns: The euler pitch angle.
        :rtype: float
        """

        raise NotImplementedError

    def yaw(self) -> float:
        """
        As of now, use the heading instead of yaw angle.
        Heading angle with East as zero, yaw angle has North as zero

        :returns: The euler yaw angle.
        """
        
        raise NotImplementedError

    def heading(self) -> float:
        """
        Get the heading of vehicle position, which is the angle
        between the vehicle's heading direction and the x-axis.

        :returns: The angle between the vehicle's heading direction and the x-axis.
        :rtype: float
        """

        raise NotImplementedError

    def linear_velocity(self) -> float:
        """
        Get the vehicle's linear velocity.
        
        :returns: The vehicle's linear velocity.
        """
        
        raise NotImplementedError

    def angular_velocity(self) -> float:
        """
        Get the vehicle's angular velocity.
        
        :returns: The vehicle's angular velocity.
        """

        raise NotImplementedError

    def linear_acceleration(self) -> float:
        """
        Get the vehicle's linear acceleration.
        
        :returns: The vehicle's linear acceleration.
        :rtype: float
        """

        raise NotImplementedError

    def gear(self) -> float:
        """
        Get the vehicle's gear position.
        
        :returns: The vehicle's gear position.
        :rtype: float
        """

        raise NotImplementedError

    def steering_percentage(self) -> float:
        """
        Get the vehicle's steering angle.
        
        :returns: The vehicle's steering angle.
        :rtype: float
        """

        raise NotImplementedError

    def set_linear_velocity(self, linear_velocity: float) -> None:
        """
        Set the vehicle's linear velocity.
        
        :param float linear_velocity: The value to set the vehicle's linear velocity.
        """

        raise NotImplementedError

    def EstimateFuturePosition(self, t: float) -> Vec2d:
        """
        Estimate future position from current position and heading,
        along a period of time, by constant linear velocity,
        linear acceleration, angular velocity.

        :param float t: The length of time period.
        :returns: The estimated future position in time t.
        :rtype: Vec2d
        """

        raise NotImplementedError

    def ComputeCOMPosition(self, rear_to_com_distance: float) -> Vec2d:
        """
        Compute the position of center of mass(COM) of the vehicle,
        given the distance from rear wheels to the center of mass.
        
        :param float rear_to_com_distance: Distance from rear wheels to the vehicle's center of mass.
        :returns: The position of the vehicle's center of mass.
        :rtype: Vec2d
        """

        raise NotImplementedError

    def vehicle_state(self) -> VehicleState:
        """
        Return the current vehicle state.

        :returns: The current vehicle state.
        :rtype: VehicleState
        """

        raise NotImplementedError

    def ConstructExceptLinearVelocity(self, localization: LocalizationEstimate) -> bool:
        """
        Construct the vehicle state except for the linear velocity.

        :param LocalizationEstimate localization: Localization estimate.
        :returns: A boolean indicating success.
        :rtype: bool
        """

        raise NotImplementedError
