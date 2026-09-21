"""
trajectory curve1D abstract class
"""
from abc import ABC, abstractmethod

class Curve1d(ABC):
    """
    Curve1d class
    Base type for various types of 1-dimensional curves
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        super().__init__()

    @abstractmethod
    def Evaluate(order: int, param: float) -> float:
        """
        Evaluate the curve at the given order and parameter
        Note that this is an abstract method and must be implemented in the derived class

        :param int order: the order of the curve to evaluate
        :param float param: the corresponding parameter
        :returns: the evaluated value
        :rtype: float
        """
        raise NotImplementedError

    @abstractmethod
    def ParamLength() -> float:
        """
        Get the length of the parameter
        Note that this is an abstract method and must be implemented in the derived class

        :returns: the length of the parameter
        :rtype: float
        """
        raise NotImplementedError

    @abstractmethod
    def __str__() -> str:
        """
        Convert the curve to a string
        Note that this is an abstract method and must be implemented in the derived class

        :returns: the string representation of the curve
        :rtype: str
        """
        raise NotImplementedError
