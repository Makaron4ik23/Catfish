class DroneSDKError(Exception):
    pass


class ConnectionError(DroneSDKError):
    pass


class TimeoutError(DroneSDKError):
    pass


class MAVSDKError(DroneSDKError):
    pass


class GazeboError(DroneSDKError):
    pass


class CameraError(DroneSDKError):
    pass


class LEDError(DroneSDKError):
    pass
