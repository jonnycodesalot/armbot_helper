# Props to https://github.com/kevinhughes27/TensorKart for helping figure out some of the button constants for the XBox controller
from inputs import get_gamepad
import math
import threading
import time
import joint_motion
from xarm import version


# Mapping for a button.
# code - the code as returned by python input library when the button is pressed
# name - the display name of the button
class Mapping:
    def __init__(self, code, name):
        self.code = code
        self.name = name
        self.value = 0.0
        self.divisor = 1.0
        self.deadzone = 0.0

# Mapping for a joystick; allows nonzero values for deadzone and divisor
# divisor - the number we divide the value by to normalize to a 0-to-1 range
# deadzone - the area around 0 that we treat as 0 (after divisor is applied)


class JoyMapping(Mapping):
    def __init__(self, code, name, divisor, deadzone):
        Mapping.__init__(self, code, name)
        self.divisor = divisor
        self.deadzone = deadzone


class XboxController:

    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    # Expirimentally determined
    JOY_DEAD_ZONE = 0.05
    TRIG_DEAD_ZONE = 0.0
    button_mappings = [Mapping('BTN_EAST', 'B Button'),
                       Mapping('BTN_WEST', 'X Button'),
                       Mapping('BTN_NORTH', 'Y Button'),
                       Mapping('BTN_SOUTH', 'A Button'),
                       Mapping('BTN_TL', 'Left Bumper'),
                       Mapping('BTN_RL', 'Right Bumper'),
                       Mapping('BTN_START', 'Start Button'),
                       Mapping('BTN_SELECT', 'Select Button'),
                       Mapping('ABS_HAT0X', 'Pad X'),
                       Mapping('ABS_HAT0Y', 'Pad Y'),
                       JoyMapping('ABS_X', 'Left Joy X', MAX_JOY_VAL, JOY_DEAD_ZONE),  # NOQA
                       JoyMapping('ABS_Y', 'Left Joy Y', MAX_JOY_VAL, JOY_DEAD_ZONE),  # NOQA
                       JoyMapping('ABS_RY', 'Right Joy Y', MAX_JOY_VAL, JOY_DEAD_ZONE),  # NOQA
                       JoyMapping('ABS_RX', 'Right Joy X', MAX_JOY_VAL, JOY_DEAD_ZONE),  # NOQA
                       JoyMapping('ABS_RZ', 'Right Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE),  # NOQA
                       JoyMapping('ABS_Z', 'Left Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE),  # NOQA
                       ]

    def __init__(self):
        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        joint_motion.RobotMain.pprint(
            'xArm-Python-SDK Version:{}'.format(version.__version__))
        self.arm = joint_motion.XArmAPI('192.168.1.187', baud_checkset=False)
        self.robot_main = joint_motion.RobotMain(self.arm)
        self.first_motion = False
        self._state_changed_callback = False
        self._count_changed_callback = False
        self._error_warn_changed_callback = False

    def _check_code(self, code, label):
        if code != 0:
            self.alive = False
            ret1 = self.arm.get_state()
            ret2 = self.arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(
                label, code, self.arm.connected, self.arm.state, self.arm.error_code, ret1, ret2))
        return True

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                for mapping in self.button_mappings:
                    if event.code == mapping.code:
                        _angle_speed = 35
                        _angle_acc = 350
                        code = 0

                        self.arm.release_error_warn_changed_callback(
                            self._error_warn_changed_callback)
                        self.arm.release_state_changed_callback(
                            self._state_changed_callback)
                        if hasattr(self.arm, 'release_count_changed_callback'):
                            self.arm.release_count_changed_callback(
                                self._count_changed_callback)
                        # Figure out the normalized value
                        new_value = event.state/mapping.divisor
                        # Apply the deadzone
                        if (new_value > -mapping.deadzone and new_value < mapping.deadzone):
                            new_value = 0
                        if (mapping.value == new_value):
                            # no change
                            continue
                        # cache the new value
                        mapping.value = new_value
                        # Print it out
                        print(
                            f"Button {mapping.name} value {mapping.value}")
                        if (new_value == 1):
                            if (self.first_motion):
                               # code = self.arm.set_servo_angle(angle=[-18.4, 36.4, 69.6, 0.0, 33.2, -18.4], speed=_angle_speed, mvacc=_angle_acc, wait=True, radius=0.0)
                                code = self.arm.set_arm_position(
                                    x=241, y=-23, z=352, speed=30)
                                self.first_motion = False
                            else:
                                code = self.arm.set_arm_position(
                                    x=241, y=-23, z=362, speed=30)
                                self.first_motion = True
                            if not self._check_code(code, 'set_servo_angle'):
                                return


if __name__ == '__main__':
    joy = XboxController()
    # For now, just keep the program alive 5 minutes.  Ultimately we'll probably do some other processing here.
    time.sleep(300)
