class PathPoint:
    """
    class PathPoint
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, kappa=0.0, dkappa=0.0, s=0.0):
        """
        Constructor

        :param x: x coordinate
        :param y: y coordinate
        :param theta: heading angle
        :param kappa: curvature
        :param dkappa: derivative of curvature
        :param s: accumulated s value
        """

        self.x = x
        self.y = y
        self.theta = theta
        self.kappa = kappa
        self.dkappa = dkappa
        self.s = s
