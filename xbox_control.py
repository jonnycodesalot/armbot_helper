# Props to https://github.com/kevinhughes27/TensorKart for helping figure out some of the button constants for the XBox controller
from inputs import get_gamepad
import math
import threading
import datetime
import time
import joint_motion
import enum
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

    # Returns true of this control captures the event, false otherwise
    def notify_event(self, event_id, value):
        if self.id == event_id:
            self.notify_update(value)
            return True
        return False

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


class Handler:
    def notify_interval(self):
        pass


class Button(Control, Handler):
    # Number of intervals a button must be held to be satisfy ButtonAction.HOLD
    BUTTON_HOLD_INTERVAL_COUNT = 10

    def __init__(self, name, id):
        super().__init__(name, id)
        self.state = 0
        self.handlers = [None] * ButtonAction.NUM_ACTIONS
        # Start at 1 so we don't get fake release operations
        self.intervals_since_update = 1
        self.press_hold_action_performed = False

    # Set a handler for a specific button action
    def set_handler(self, button_action, handler):
        self.handlers[button_action] = handler

    def notify_update(self, value):
        self.state = value
        handler = None
        # figure out what handler to use,
        if value == 0:
            # Clear the hold metadata
            self.press_hold_action_performed = False
        # Reset our hold interval counter
        self.intervals_since_update = 0

    def notify_interval(self):
        handler = None
        output_prefix = ""
        # increment intervals since update
        self.intervals_since_update = self.intervals_since_update+1
        if self.state == 1:
            if self.intervals_since_update >= self.BUTTON_HOLD_INTERVAL_COUNT:
                # The button is being held down.  Do the hold actions
                if self.press_hold_action_performed == False:
                    self.press_hold_action_performed = True
                    handler = self.handlers[ButtonAction.HOLD_DO_ONCE]
                    output_prefix = "Held (once)"
                if handler == None:
                    # No hold do once action - do the repeat action
                    handler = self.handlers[ButtonAction.HOLD_REPEAT]
                    output_prefix = "Held (repeat)"
            elif self.intervals_since_update == 1:
                handler = self.handlers[ButtonAction.PRESS]
                output_prefix = "Pressed"
        elif self.intervals_since_update == 1:  # state == 0
            handler = self.handlers[ButtonAction.RELEASE]
            output_prefix = "Released"
        if handler != None:
            print(output_prefix, self.name)
            handler()


class CartesianAxis(enum.IntEnum):
    AXIS_X = 0
    AXIS_Y = 1
    AXIS_Z = 2
    AXIS_ROLL = 3
    AXIS_PITCH = 4
    AXIS_YAW = 5


class ServoCartesianXyz(Handler):
    def __init__(self, name, controller, initial_x, initial_y, initial_z):
        self.name = name
        self.coordinates = starting_pose
        self.controller = controller
        self.updated = False

    def add_delta(self, axis, value):
        self.coordinates[axis] += value
        print(self.name, "Position:", self.coordinates)
        self.updated = True

    def notify_interval(self):
        if (self.updated):
            self.updated = False
            new_pose = [self.coordinates[CartesianAxis.AXIS_X], self.coordinates[CartesianAxis.AXIS_Y],
                        self.coordinates[CartesianAxis.AXIS_Z], self.coordinates[CartesianAxis.AXIS_ROLL],
                        self.coordinates[CartesianAxis.AXIS_PITCH], self.coordinates[CartesianAxis.AXIS_YAW]]
            code = self.controller.arm.set_servo_cartesian(
                new_pose, speed=my_speed, mvacc=my_mvacc)
            if not self.controller._check_code(code, 'set_servo_cartesian'):
                exit()


class AnalogAxis(Control):
    ANALOG_DEAD_ZONE = 0.12

    def __init__(self, name, id, initial_value, encoder_divisor, position_multiplier):
        super().__init__(id=id, name=name)
        self.value = initial_value
        self.delta = 0
        self.encoder_divisor = encoder_divisor
        self.handler = None
        self.position_multiplier = position_multiplier

    # Set a handler for a specific button action
    def set_handler(self, handler):
        self.handler = handler

    def notify_update(self, value):
        normalized = value/self.encoder_divisor
        if (normalized < self.ANALOG_DEAD_ZONE and normalized > -self.ANALOG_DEAD_ZONE):
            self.delta = 0.0
        else:
            self.delta = normalized * self.position_multiplier

    def notify_interval(self):
        if self.delta != 0.0:
            if (self.handler != None):
                self.handler(delta=self.delta*2.0)


class JoyAnalogAxis(AnalogAxis):
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self, name, id,  initial_value, multiplier):
        super().__init__(name, id, initial_value, self.MAX_JOY_VAL, multiplier)


class TriggerAnalogAxis(AnalogAxis):
    MAX_TRIGGER_VAL = math.pow(2, 8)

    def __init__(self, name, id,  initial_value, multiplier):
        super().__init__(name, id, initial_value, self.MAX_TRIGGER_VAL, multiplier)


class TriggerServoControllerAxis(TriggerAnalogAxis):
    def __init__(self, name,  id, servo_cartesian_xyz, axis, multiplier):
        super().__init__(
            name,  id, initial_value=servo_cartesian_xyz.coordinates[axis], multiplier=multiplier)
        super().set_handler(
            handler=lambda delta: servo_cartesian_xyz.add_delta(axis, delta))


class JoyServoControllerAxis(JoyAnalogAxis):
    def __init__(self, name, id,  servo_cartesian_xyz, axis, multiplier):
        super().__init__(
            name, id, initial_value=servo_cartesian_xyz.coordinates[axis], multiplier=multiplier)
        super().set_handler(
            handler=lambda delta: servo_cartesian_xyz.add_delta(axis, delta))


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


starting_pose = [241, -23, 352, 180, 0, 0]
pose_delta = [0, 0, 0, 0, 0, 0]
my_speed = 30
my_mvacc = 5000


class XboxController:

    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    # Expirimentally determined
    JOY_DEAD_ZONE = 0.05
    TRIG_DEAD_ZONE = 0.0

    # button_mappings = [Mapping('BTN_EAST', 'B Button', increment_z),
    #                Mapping('BTN_WEST', 'X Button', increment_x),
    #                Mapping('BTN_NORTH', 'Y Button', increment_y),
    #                Mapping('BTN_SOUTH', 'A Button', empty),
    #                Mapping('BTN_TL', 'Left Bumper', empty),
    #                Mapping('BTN_RL', 'Right Bumper', empty),
    #                Mapping('BTN_START', 'Start Button', empty),
    #                Mapping('BTN_SELECT', 'Select Button', empty),
    #                Mapping('ABS_HAT0X', 'Pad X', empty),
    #                Mapping('ABS_HAT0Y', 'Pad Y', empty),
    #                JoyMapping('ABS_X', 'Left Joy X', MAX_JOY_VAL, JOY_DEAD_ZONE, joy_x),  # NOQA
    #                # JoyMapping('ABS_Y', 'Left Joy Y', MAX_JOY_VAL, JOY_DEAD_ZONE,empty),  # NOQA
    #                JoyMapping('ABS_RY', 'Right Joy Y', MAX_JOY_VAL, JOY_DEAD_ZONE, empty),  # NOQA
    #                JoyMapping('ABS_RX', 'Right Joy X', MAX_JOY_VAL, JOY_DEAD_ZONE, empty),  # NOQA
    #                JoyMapping('ABS_RZ', 'Right Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE, empty),  # NOQA
    #                JoyMapping('ABS_Z', 'Left Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE, empty),  # NOQA
    #                ]

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
        self.arm.set_state(ArmState.SPORT_STATE)
        self.arm.set_position(*starting_pose, wait=True)
        self.arm.set_mode(mode=ArmMode.SERVO_MOTION_MODE)
        self.arm.set_state(ArmState.SPORT_STATE)
        self.arm.motion_enable(enable=True)
        self.it_changed = False
        self.cart = ServoCartesianXyz(
            "Robot position", self, starting_pose[0], starting_pose[1], starting_pose[2])

    #   JoyMapping('ABS_RZ', 'Right Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE, empty),  # NOQA
    #                    JoyMapping('ABS_Z', 'Left Trigger', MAX_TRIG_VAL, TRIG_DEAD_ZONE, empty),  # NOQA
        self.buttons = [Button(id='BTN_EAST', name='B'),
                        Button(id='BTN_WEST', name='X'),
                        Button(id='BTN_TL', name='Left Bumper'),
                        Button(id='BTN_TR', name='Right Bumper'),
                        JoyServoControllerAxis(
                            id='ABS_X', name="Left Joy X", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_X, multiplier=2.0),
                        JoyServoControllerAxis(
                            id='ABS_Y', name="Left Joy Y", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Y, multiplier=2.0),
                        JoyServoControllerAxis(
                            id='ABS_RX', name="Right Joy X", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_ROLL, multiplier=0.5),
                        JoyServoControllerAxis(
                            id='ABS_RY', name="Right Joy Y", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_PITCH, multiplier=0.5),
                        # JoyServoControllerAxis(
                        #     id='ABS_RY', name="Right Joy Y", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Z),
                        TriggerServoControllerAxis(
                            id='ABS_RZ', name="Trig Right", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Z, multiplier=2.0),
                        TriggerServoControllerAxis(
                            id='ABS_Z', name="Trig Left", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Z, multiplier=-2.0)
                        ]
        self.buttons[0].set_handler(
            ButtonAction.PRESS, lambda: print("Pressed B action"))
        self.buttons[0].set_handler(
            ButtonAction.RELEASE, lambda: print("Released B action"))
        self.buttons[1].set_handler(
            ButtonAction.PRESS, lambda: print("Pressed X action"))
        self.buttons[1].set_handler(
            ButtonAction.HOLD_DO_ONCE, lambda: print("Holding X action"))
        self.buttons[1].set_handler(
            ButtonAction.HOLD_REPEAT, lambda: print("Holding X Repeat action"))
        self.buttons[1].set_handler(
            ButtonAction.RELEASE, lambda: print("Released X action"))
        self.buttons[2].set_handler(
            ButtonAction.PRESS, lambda: self.cart.add_delta(CartesianAxis.AXIS_YAW, -1.0))
        self.buttons[3].set_handler(
            ButtonAction.PRESS, lambda: self.cart.add_delta(CartesianAxis.AXIS_YAW, 1.0))
        self.buttons[2].set_handler(
            ButtonAction.HOLD_REPEAT, lambda: self.cart.add_delta(CartesianAxis.AXIS_YAW, -1.0))
        self.buttons[3].set_handler(
            ButtonAction.HOLD_REPEAT, lambda: self.cart.add_delta(CartesianAxis.AXIS_YAW, 1.0))

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
                for button in self.buttons:
                    if button.notify_event(event.code, event.state):
                        break
                # for mapping in self.button_mappings:
                #     if event.code == mapping.code:
                #         _angle_speed = 35
                #         _angle_acc = 350
                #         code = 0

                #         self.arm.release_error_warn_changed_callback(
                #             self._error_warn_changed_callback)
                #         self.arm.release_state_changed_callback(
                #             self._state_changed_callback)
                #         if hasattr(self.arm, 'release_count_changed_callback'):
                #             self.arm.release_count_changed_callback(
                #                 self._count_changed_callback)
                #         # Figure out the normalized value
                #         new_value = event.state/mapping.divisor
                #         # Apply the deadzone
                #         if (new_value > -mapping.deadzone and new_value < mapping.deadzone):
                #             new_value = 0
                #         if (mapping.value == new_value):
                #             # no change
                #             continue
                #         # cache the new value
                #         mapping.value = new_value
                #         # Print it out
                #         print(
                #             f"Button {mapping.name} value {mapping.value}")
                #         # if (new_value != 0):
                #         # code = self.arm.set_servo_angle(angle=[-18.4, 36.4, 69.6, 0.0, 33.2, -18.4], speed=_angle_speed, mvacc=_angle_acc, wait=True, radius=0.0)
                #         if (mapping.call != empty):
                #             code = mapping.call(self, new_value)
                #             self.first_motion = False
                #             if not self._check_code(code, 'set_servo_cartesian'):
                #                 return
                #         # if (new_value == 0):


if __name__ == '__main__':
    joy = XboxController()
    # For now, just keep the program alive 5 minutes.  Ultimately we'll probably do some other processing here.
    # needs_update = True
    while (True):
        time.sleep(0.02)
        for button in joy.buttons:
            button.notify_interval()
        joy.cart.notify_interval()
        # if (pose_delta != [0, 0, 0, 0, 0, 0]):
        #     for i in range(6):
        #         current_pose[i] += pose_delta[i]
        #     code = joy.arm.set_servo_cartesian(
        #         current_pose, speed=my_speed, mvacc=my_mvacc)
        #     if not joy._check_code(code, 'set_servo_cartesian'):
        #         exit()
