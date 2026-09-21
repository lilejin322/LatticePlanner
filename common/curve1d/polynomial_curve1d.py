"""
Polynomial curve1D submodule
"""
from abc import abstractmethod
from common.curve1d.curve1d import Curve1d

class PolynomialCurve1d(Curve1d):
    """
    PolynomialCurve1d class
    base type for polynomial curves in different orders
    """
    _param: float = 0.0

    def __init__(self) -> None:
        """
        Constructor
        """
        super().__init__()
        self._param: float = 0.0

    @abstractmethod
    def Coef(self, order: int) -> float:
        """
        Coef method
        Note that this is an abstract method and must be implemented in the derived class

        :param int order: the order
        :returns: the coef
        :rtype: float
        """
        raise NotImplementedError

    @abstractmethod
    def Order(self) -> int:
        """
        Order method
        Note that this is an abstract method and must be implemented in the derived class

        :returns: the order
        :rtype: int
        """
        raise NotImplementedError
