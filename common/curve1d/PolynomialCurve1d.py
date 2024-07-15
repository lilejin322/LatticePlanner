from abc import abstractmethod
from common.curve1d import Curve1d

class PolynomialCurve1d(Curve1d):
    """
    PolynomialCurve1d class
    base type for polynomial curves in different orders
    """

    def __init__(self):
        """
        Constructor
        """

        super.__init__()
        self._param = 0.0

    @abstractmethod
    def Coef(order: int) -> float:
        """
        Coef method
        Note that this is an abstract method and must be implemented in the derived class
        """

        raise NotImplementedError

    @abstractmethod
    def Order() -> int:
        """
        Order method
        Note that this is an abstract method and must be implemented in the derived class
        """

        raise NotImplementedError
