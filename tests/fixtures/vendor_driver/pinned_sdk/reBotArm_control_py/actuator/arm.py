class Controller:
    pass


class RobotArm:
    def __init__(self, channel):
        self._channel = channel

    def _make_controller(self):
        if self._channel.startswith("/dev/tty"):
            return Controller.from_dm_serial(self._channel, 921600)
        return Controller(self._channel)
