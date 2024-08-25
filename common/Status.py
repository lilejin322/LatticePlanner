from protoclass.Header import ErrorCode, StatusPb

class Status:
    """
    A general class to denote the return status of an API call. It
    can either be an OK status for success, or a failure status with error
    message and error code enum.
    """

    def __init__(self, code=ErrorCode.OK, msg=""):
        """
        Create a status with the specified error code and msg as a
        human-readable string containing more detailed information.

        :param ErrorCode code: the error code.
        :param str msg: the message associated with the error.
        """

        self._code = code
        self._msg = msg

    @staticmethod
    def OK() -> 'Status':
        """
        Generate a success status.

        :returns: a success status
        :rtype: Status
        """

        return Status()

    def ok(self) -> bool:
        """
        Check whether the return status is OK.

        :returns: True if the code is ErrorCode.OK, False otherwise
        :rtype: bool
        """

        return self._code == ErrorCode.OK

    @property
    def code(self) -> ErrorCode:
        """
        Get the error code.

        :returns: the error code
        :rtype: ErrorCode
        """

        return self._code

    def __eq__(self, other: 'Status') -> bool:
        """
        Defines the logic of testing if two Status are equal.

        :returns: True if both the code and the message are equal.
        :rtype: bool
        """

        return (self._code == other._code) and (self._msg == other._msg)

    def __ne__(self, other: 'Status') -> bool:
        """
        Defines the logic of testing if two Status are unequal.

        :returns: True if the Status are not equal.
        :rtype: bool
        """

        return not self.__eq__(other)

    @property
    def error_message(self) -> str:
        """
        Returns the error message of the status, empty if the status is OK.

        :returns: the error message
        :rtype: str
        """

        return self._msg

    def __str__(self) -> str:
        """
        Returns a string representation in a readable format.

        :returns: the string "OK" if success, the internal error message otherwise.
        :rtype: str
        """

        if self.ok():
            return "OK"

        return f"{self._code.name}: {self._msg}"

    def save(self, status_pb: StatusPb) -> None:
        """
        Save the error_code and error message to protobuf.

        :param status_pb: the Status protobuf that will store the message.
        """

        if status_pb is None:
            return
        status_pb.error_code = self._code
        if self._msg:
            status_pb.msg = self._msg
