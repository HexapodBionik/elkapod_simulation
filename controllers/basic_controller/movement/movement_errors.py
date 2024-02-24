
class MovementCoreInitializationError(Exception):
    def __init__(self, message: str):
        super().__init__(message)


class MovementCoreRuntimeError(Exception):
    def __init__(self, message: str):
        super().__init__(message)