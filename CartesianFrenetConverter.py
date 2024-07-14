import math
from typing import List, Tuple

class CartesianFrenetConverter:
    """
    CartesianFrenetConverter class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

    @staticmethod
    def cartesian_to_frenet(rs:float, rx:float, ry:float, rtheta:float, rkappa:float, rdkappa:float,
                            x:float, y:float, v:float, a:float, theta:float, kappa:float) -> Tuple[List[float], List[float]]:
        """
        Convert a Cartesian coordinate to a Frenet coordinate

        :param rs: matched point s
        :param rx: matched point x
        :param ry: matched point y
        :param rtheta: matched point theta
        :param rkappa: matched point kappa
        :param rdkappa: matched point dkappa
        :param x: cartesian_state point x
        :param y: cartesian_state point y
        :param v: cartesian_state point v
        :param a: cartesian_state point a
        :param theta: cartesian_state point theta
        :param kappa: cartesian_state point kappa
        :returns: Frenet state
        :rtype: Tuple[List[float], List[float]]
        """

        dx = x - rx
        dy = y - ry

        cos_theta_r = math.cos(rtheta)
        sin_theta_r = math.sin(rtheta)

        cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
        d = math.copysign(math.sqrt(dx * dx + dy * dy), cross_rd_nd)

        delta_theta = theta - rtheta
        tan_delta_theta = math.tan(delta_theta)
        cos_delta_theta = math.cos(delta_theta)

        one_minus_kappa_r_d = 1 - rkappa * d
        d_prime = one_minus_kappa_r_d * tan_delta_theta

        kappa_r_d_prime = rdkappa * d + rkappa * d_prime
        d_double_prime = -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta * (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa)

        s = rs
        s_prime = v * cos_delta_theta / one_minus_kappa_r_d

        delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
        s_double_prime = (a * cos_delta_theta - s_prime * s_prime * (d_prime * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d

        return [s, s_prime, s_double_prime], [d, d_prime, d_double_prime]

    @staticmethod
    def frenet_to_cartesian(rs: float, rx: float, ry: float, rtheta: float, rkappa: float, rdkappa: float, \
                            s_condition: List[float], d_condition: List[float]) -> Tuple[float, float, float, float, float, float]:
        """
        Convert a Frenet coordinate to a Cartesian coordinate

        :param rs: matched point s
        :param rx: matched point x
        :param ry: matched point y
        :param rtheta: matched point theta
        :param rkappa: matched point kappa
        :param rdkappa: matched point dkappa
        :param s_condition: Frenet state
        :param d_condition: Frenet state
        :returns: Cartesian state (x, y, theta, kappa, v, a)
        :rtype: Tuple[float, float, float, float, float, float]
        """

        assert abs(rs - s_condition[0]) < 1.0e-6, "The reference point s and s_condition[0] don't match"

        cos_theta_r: float = math.cos(rtheta)
        sin_theta_r: float = math.sin(rtheta)

        x: float = rx - sin_theta_r * d_condition[0]
        y: float = ry + cos_theta_r * d_condition[0]

        one_minus_kappa_r_d: float = 1 - rkappa * d_condition[0]

        tan_delta_theta: float = d_condition[1] / one_minus_kappa_r_d
        delta_theta: float = math.atan2(d_condition[1], one_minus_kappa_r_d)
        cos_delta_theta: float = math.cos(delta_theta)

        theta: float = CartesianFrenetConverter.NormalizeAngle(delta_theta + rtheta)

        kappa_r_d_prime: float = rdkappa * d_condition[0] + rkappa * d_condition[1]

        kappa: float = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta * cos_delta_theta) / one_minus_kappa_r_d + rkappa) * cos_delta_theta / one_minus_kappa_r_d

        d_dot: float = d_condition[1] * s_condition[1]
        v: float = math.sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] + d_dot * d_dot)

        delta_theta_prime: float = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa

        a: float = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta + s_condition[1] * s_condition[1] / cos_delta_theta * (d_condition[1] * delta_theta_prime - kappa_r_d_prime)

        return x, y, theta, kappa, v, a

    @staticmethod
    def NormalizeAngle(angle: float) -> float:
        """
        Normalize the angle to [-pi, pi]

        :param float angle: the angle to normalize
        :returns: the normalized angle
        :rtype: float
        """

        a: float = math.fmod(angle + math.pi, 2 * math.pi)
        if a < 0.0:
            a += 2 * math.pi
        return a - math.pi
