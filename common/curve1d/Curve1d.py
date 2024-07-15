from abc import ABC, abstractmethod

class Curve1d(ABC):
    """
    Curve1d class
    Base type for various types of 1-dimensional curves
    """

    def __init__(self):
        """
        Constructor
        """

        super.__init__()

    @abstractmethod
    def Evaluate(order: int, param: float) -> float:
        """
        Evaluate the curve at the given order and parameter
        Note that this is an abstract method and must be implemented in the derived class
        """

        raise NotImplementedError

    @abstractmethod
    def ParamLength() -> float:
        """
        Get the length of the parameter
        Note that this is an abstract method and must be implemented in the derived class
        """

        raise NotImplementedError

    @abstractmethod
    def ToString() -> str:
        """
        Convert the curve to a string
        Note that this is an abstract method and must be implemented in the derived class
        """

        raise NotImplementedError
