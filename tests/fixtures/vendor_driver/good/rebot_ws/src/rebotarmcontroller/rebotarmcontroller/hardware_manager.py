from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.controllers import ArmEndPos
from reBotArm_control_py.actuator.gripper import load_cfg


class HardwareManager:
    def connect(self) -> None:
        self._endpos_ctrl.start()
        self._seed_endpos_target()
        self._arm.mode_pos_vel()
        self._ctrl.enable_all()

    def init_gripper(self, cfg_path: str) -> None:
        load_cfg(cfg_path)
        ctrl.enable_all()

    def shutdown(self) -> None:
        self.safe_park()
        self.disable()

    def _seed_endpos_target(self) -> None:
        pass

    def safe_park(self) -> None:
        pass

    def set_gripper_target(self, position: float) -> None:
        pass
