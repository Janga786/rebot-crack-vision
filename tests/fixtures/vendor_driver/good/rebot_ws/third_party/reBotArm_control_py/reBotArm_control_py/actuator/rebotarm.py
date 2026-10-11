class RobotArm:
    def _connect_serial(self):
        return Controller.from_dm_serial(self._channel, 921600)
