from dataclasses import dataclass, field
from typing import Optional
from protoclass.PointENU import PointENU
from protoclass.ADCTrajectory import Point3D
import math

@dataclass
class Quaternion:
    """
    A unit quaternion that represents a spatial rotation. See the link below for details.
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    The scalar part qw can be omitted. In this case, qw should be calculated by
    qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
    """

    qx: Optional[float] = field(default=math.nan)
    qy: Optional[float] = field(default=math.nan)
    qz: Optional[float] = field(default=math.nan)
    qw: Optional[float] = field(default=math.nan)

@dataclass
class Pose:
    """
    Pose class, oriented from protobuf message
    """

    # Position of the vehicle reference point (VRP) in the map reference frame.
    # The VRP is the center of rear axle.
    position: Optional[PointENU] = None
    
    # A quaternion that represents the rotation from the IMU coordinate
    # (Right/Forward/Up) to the
    # world coordinate (East/North/Up).
    orientation: Optional[Quaternion] = None
    
    # Linear velocity of the VRP in the map reference frame.
    # East/north/up in meters per second.
    linear_velocity: Optional[Point3D] = None
    
    # Linear acceleration of the VRP in the map reference frame.
    # East/north/up in meters per square second.
    linear_acceleration: Optional[Point3D] = None
    
    # Angular velocity of the vehicle in the map reference frame.
    # Around east/north/up axes in radians per second.
    angular_velocity: Optional[Point3D] = None
    
    # Heading
    # The heading is zero when the car is facing East and positive when facing
    # North.
    heading: Optional[float] = None
    
    # Linear acceleration of the VRP in the vehicle reference frame.
    # Right/forward/up in meters per square second.
    linear_acceleration_vrf: Optional[Point3D] = None
    
    # Angular velocity of the VRP in the vehicle reference frame.
    # Around right/forward/up axes in radians per second.
    angular_velocity_vrf: Optional[Point3D] = None
    
    # Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.
    # in world coordinate (East/North/Up)
    # The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.
    # The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.
    # The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
    # The direction of rotation follows the right-hand rule.
    euler_angles: Optional[Point3D] = None
