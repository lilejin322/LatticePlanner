import math
from typing import List, Tuple

class CartesianFrenetConverter:

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
