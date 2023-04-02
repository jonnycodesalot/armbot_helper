# Props to https://github.com/kevinhughes27/TensorKart for helping figure out some of the button constants for the XBox controller
from inputs import get_gamepad
import math
import threading
import datetime
import time
import joint_motion
import enum
import overrides
from xarm import version


# Enums for the various arm APIs
class ArmState(enum.IntEnum):
    SPORT_STATE = 0
    PAUSE_STATE = 3
    STOP_STATE = 4


class ArmMode(enum.IntEnum):
    POSITION_CONTROL_MODE = 0
    SERVO_MOTION_MODE = 1
    JOINT_TEACHING_MODE = 2
    CART_TEACHING_MODE_INVALID_ = 3
    JOINT_VELO_CONTROL_MODE = 4
    CART_VELO_CONTROL_MODE = 5
    JOINT_ONLINE_TRAJ_PLAN_MODE = 6
    CART_ONLINE_TRAJ_PLAN_MODE = 7


def empty():
    return

# Mapping for a button.
# code - the code as returned by python input library when the button is pressed
# name - the display name of the button


class Control():
    def __init__(self, name, id):
        self.name = name
        self.id = id

    # Called when the button is pressed.  Derived object implements relevant logic

    def notify_update(self, value):
        pass

    def notify_interval(self):
        pass

# Possible actions for a button


class ButtonAction(enum.IntEnum):
    # Action occurs when pressed
    PRESS = 0
    # Action occurs when released
    RELEASE = 1
    # Action occurs BUTTON_HOLD_MS after the button is held down.
    # Using HOLD_DO_ONCE or HOLD_REPEAT will override the action set on PRESS
    HOLD_DO_ONCE = 2
    HOLD_REPEAT = 3
    NUM_ACTIONS = 4


class Button(Control):
    # Number of intervals a button must be held to be satisfy ButtonAction.HOLD
    BUTTON_HOLD_INTERVAL_COUNT = 10

    def __init__(self, name, id):
        super().__init__(self, name, id)
        self.state = 0
        self.handlers = [None] * ButtonAction.NUM_ACTIONS
        self.intervals_since_update = 0
        self.press_hold_action_performed = False

    # Set a handler for a specific button action
    def set_handler(self, button_action, handler):
        self.handlers[button_action] = handler

    @overrides
    def notify_update(self, value):
        self.state = value
        handler = None
        # figure out what handler to use,
        if value == 0:
            # Clear the hold metadata
            self.press_hold_action_performed = False
        # Reset our hold interval counter
        self.intervals_since_update = 0

    @overrides
    def notify_interval(self):
        handler = None
        # increment intervals since update
        self.intervals_since_update = self.intervals_since_update+1
        if self.state == 1:
            if self.intervals_since_update >= self.BUTTON_HOLD_INTERVAL_COUNT:
                # The button is being held down.  Do the hold actions
                if self.press_hold_action_performed == False:
                    self.press_hold_action_performed = True
                    handler = self.handlers[ButtonAction.HOLD_DO_ONCE]
                if handler == None:
                    # No hold do once action - do the repeat action
                    handler = self.handlers[ButtonAction.HOLD_REPEAT]
            elif self.intervals_since_update == 1:
                handler = self.handlers[ButtonAction.PRESS]
        elif self.intervals_since_update == 1:  # state == 0
            handler = self.handlers[ButtonAction.RELEASE]
        if handler != None:
            handler()


class AnalogAxis(Control):
    def __init__(self, name, id):
        super().__init__(self, name, id)
        self.position = 0.0


class Mapping:
    def __init__(self, code, name, function):
        self.code = code
        self.name = name
        self.value = 0.0
        self.divisor = 1.0
        self.deadzone = 0.0
        self.call = function

# Mapping for a joystick; allows nonzero values for deadzone and divisor
# divisor - the number we divide the value by to normalize to a 0-to-1 range
# deadzone - the area around 0 that we treat as 0 (after divisor is applied)


class JoyMapping(Mapping):
    def __init__(self, code, name, divisor, deadzone, function):
        Mapping.__init__(self, code, name, function)
        self.divisor = divisor
        self.deadzone = deadzone


current_pose = [241, -23, 352, 180, 0, 0]
pose_delta = [0, 0, 0, 0, 0, 0]
my_speed = 30
my_mvacc = 2000


class XboxController:

    def increment_x(self, value):
        pose_delta[0] = value
        self.it_changed = True
        # return self.arm.set_servo_cartesian(current_pose, speed=my_speed, mvacc=my_mvacc)
        # return self.arm.vc_set_cartesian_velocity(speeds=[2,0,0,0,0,0],is_radian=False,duration=2)
        # return self.arm.set_position(x=-10, y=0, z=0, speed=30, relative=True, wait=False)

    def joy_x(self, value):
        pose_delta[0] = value
        self.it_changed = True
        # return self.arm.set_servo_cartesian(current_pose, speed=my_speed, mvacc=my_mvacc)

    def increment_y(self, value):
        pose_delta[1] = value
        self.it_changed = True
       # return self.arm.set_servo_cartesian(current_pose, speed=my_speed, mvacc=my_mvacc)

    def increment_z(self, value):
        pose_delta[2] = value
        self.it_changed = True
        # return self.arm.set_servo_cartesian(current_pose, speed=my_speed, mvacc=my_mvacc)

    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    # Expirimentally determined
    JOY_DEAD_ZONE = 0.05
    TRIG_DEAD_ZONE = 0.0
    button_mappings = [Mapping('BTN_EAST', 'B Button', increment_z),
                       Mapping('BTN_WEST', 'X Button', increment_x),
                       Mapping('BTN_NORTH', 'Y Button', increment_y),
                       Mapping('BTN_SOUTH', 'A Button', empty),
                       Mapping('BTN_TL', 'Left Bumper', empty),
                       Mapping('BTN_RL', 'Right Bumper', empty),
                       Mapping('BTN_START', 'Start Button', empty),
                       Mapping('BTN_SELECT', 'Select Button', empty),
                       Mapping('ABS_HAT0X', 'Pad X', empty),
                       Mapping('ABS_HAT0Y', 'Pad Y', empty),
                       JoyMapping('ABS_X', 'Left Joy X', MAX_JOY_VAL, JOY_DEAD_ZONE, joy_x),  # NOQA
                       # JoyMapping('ABS_Y', 'Left Joy Y', MAX_JOY_VAL, JOY_DEAD_ZONE,empty),  # NOQA
                       JoyMapping('ABS_RY', 'Right Joy Y', MAX_JOY_VAL, JOY_DEAD_ZONE, empty),  # NOQA
                       JoyMapping('ABS_RX', 'Right Joy X', MAX_JOY_VAL, JOY_DEAD_ZONE, empty),  # NOQA
                       JoyMapping('ABS_RZ', 'Right Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE, empty),  # NOQA
                       JoyMapping('ABS_Z', 'Left Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE, empty),  # NOQA
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
        self.arm.set_mode(mode=ArmMode.POSITION_CONTROL_MODE)
        self.arm.set_position(*current_pose, wait=True)
        self.arm.set_mode(mode=ArmMode.SERVO_MOTION_MODE)
        self.arm.set_state(ArmState.SPORT_STATE)
        self.arm.motion_enable(enable=True)
        self.it_changed = False

    def _check_code(self, code, label):
        if code != 0:
            self.alive = False
            ret1 = self.arm.get_state()
            ret2 = self.arm.get_err_warn_code()
            print('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(
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
                        # if (new_value != 0):
                        # code = self.arm.set_servo_angle(angle=[-18.4, 36.4, 69.6, 0.0, 33.2, -18.4], speed=_angle_speed, mvacc=_angle_acc, wait=True, radius=0.0)
                        if (mapping.call != empty):
                            code = mapping.call(self, new_value)
                            self.first_motion = False
                            if not self._check_code(code, 'set_servo_cartesian'):
                                return
                        # if (new_value == 0):


if __name__ == '__main__':
    joy = XboxController()
    # For now, just keep the program alive 5 minutes.  Ultimately we'll probably do some other processing here.
    # needs_update = True
    while (True):
        time.sleep(0.05)
        if (pose_delta != [0, 0, 0, 0, 0, 0]):
            for i in range(6):
                current_pose[i] += pose_delta[i]
            code = joy.arm.set_servo_cartesian(
                current_pose, speed=my_speed, mvacc=my_mvacc)
            if not joy._check_code(code, 'set_servo_cartesian'):
                exit()
