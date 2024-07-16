from typing import List, override
from common.curve1d.PolynomialCurve1d import PolynomialCurve1d
from copy import deepcopy

class QuinticPolynomialCurve1d(PolynomialCurve1d):
    """
    QuinticPolynomialCurve1d class
    1D quintic polynomial curve:
    (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)
    """

    def __init__(self, *args):
        """
        Constructor
        """

        if len(args) == 3:
            start, end, param = args
            self.__init__(start[0], start[1], start[2], end[0], end[1], end[2], param)
        elif len(args) == 7:
            super().__init__()
            x0, dx0, ddx0, x1, dx1, ddx1, param = args
            self._param = param
            self.start_condition: List[float] = [x0, dx0, ddx0]
            self.end_condition: List[float] = [x1, dx1, ddx1]
            self._coef = [0.0] * 6
            self.ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param)
        elif len(args) == 1 and isinstance(args[0], QuinticPolynomialCurve1d):
            other: QuinticPolynomialCurve1d = args[0]
            self.__dict__ = deepcopy(other.__dict__)
        else:
            raise ValueError("Invalid number of arguments")
    
    @override
    def Evaluate(self, order: int, p: float) -> float:
        """
        Evaluate the curve at p

        :param int order: Order of derivative
        :param float p: Parameter
        :returns: Value of the curve at p
        :rtype: float
        """

        if order == 0:
            return ((((self._coef[5] * p + self._coef[4]) * p + self._coef[3]) * p + self._coef[2]) * p + self._coef[1]) * p + self._coef[0]
        elif order == 1:
            return (((5.0 * self._coef[5] * p + 4.0 * self._coef[4]) * p + 3.0 * self._coef[3]) * p + 2.0 * self._coef[2]) * p + self._coef[1]
        elif order == 2:
            return (((20.0 * self._coef[5] * p + 12.0 * self._coef[4]) * p) + 6.0 * self._coef[3]) * p + 2.0 * self._coef[2]
        elif order == 3:
            return (60.0 * self._coef[5] * p + 24.0 * self._coef[4]) * p + 6.0 * self._coef[3]
        elif order == 4:
            return 120.0 * self._coef[5] * p + 24.0 * self._coef[4]
        elif order == 5:
            return 120.0 * self._coef[5]
        else:
            return 0.0

    @staticmethod
    @override
    def Order() -> int:
        """
        Return the order of the curve

        :returns: Order of the curve
        :rtype: int
        """

        return 5
    
    @override
    def Coef(self, order: int) -> float:
        """
        Get the coefficient for the given order

        :param int order: Order of the coefficient
        :returns: Coefficient for the given order
        :rtype: float
        """

        assert order < 6 and order >= 0, "Order must be in [0, 5]"
        return self._coef[order]
    
    @override
    def ToString(self) -> str:
        """
        Convert the curve to a string

        :returns: String representation of the curve
        :rtype: str
        """

        return f"{'\t'.join(map(str, self._coef))}\t{self._param}\n"
    
    def ComputeCoefficients(self, x0: float, dx0: float, ddx0: float, x1: float, dx1: float, ddx1: float, p: float) -> None:
        """
        Compute the coefficients of the curve

        :param float x0: param x0
        :param float dx0: param dx0
        :param float ddx0: param ddx0
        :param float x1: param x1
        :param float dx1: param dx1
        :param float ddx1: param ddx1
        :param float p: param p
        """

        assert p > 0.0, "Param must be positive"

        self._coef[0] = x0
        self._coef[1] = dx0
        self._coef[2] = 0.5 * ddx0

        p2: float = p ** 2
        p3: float = p ** 3

        # the direct analytical method is at least 6 times faster than using matrix
        # inversion.
        c0: float = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3
        c1: float = (dx1 - ddx0 * p - dx0) / p2
        c2: float = (ddx1 - ddx0) / p

        self._coef[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2)
        self._coef[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p
        self._coef[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2

    @override
    def ParamLength(self) -> float:
        """
        Get the param
        It is weird that this method is called 'Length' but it returns the param

        :returns: Param
        :rtype: float
        """

        return self._param
