# Props to https://github.com/kevinhughes27/TensorKart for helping figure out some of the button constants for the XBox controller
from inputs import get_gamepad
import math
import threading
import datetime
import time
import joint_motion
import enum
from xarm import version


# Enums for the various Lite6 API constants, so
# there aren't magic numbers all over the place
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

#


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

    # Set a handler for a specific button action, the function
    # that gets called when that button action is performed.
    def set_handler(self, button_action, handler):
        self.handlers[button_action] = handler

    # when an update comes in from the controller, this wll be called with the
    # value of the button.  It will update our internal state
    def notify_update(self, value):
        self.state = value
        handler = None
        # figure out what handler to use,
        if value == 0:
            # Clear the hold metadata
            self.press_hold_action_performed = False
        # Reset our hold interval counter
        self.intervals_since_update = 0

    # When an arm action interval occurs, this will be called to move the arm
    # in accordance with the latest result from calls to notify_update
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


# An enum for all the possible axes of motion that could be changed.
# Note that the rotational ones aren't actually cartesian, so theres some
# misnomer action here.
class CartesianAxis(enum.IntEnum):
    AXIS_X = 0
    AXIS_Y = 1
    AXIS_Z = 2
    AXIS_ROLL_X = 3  # or ROLL for set_servo_cartesian
    AXIS_ROLL_Y = 4  # or PITCH for set_servo_cartesian
    AXIS_ROLL_Z = 5  # or YAW for set_servo_cartesian
    AXIS_ANGLE_4 = 6
    AXIS_ANGLE_5 = 7
    AXIS_ANGLE_6 = 8

# ServoCartesianXyz is an object to track the current desired position of the
# Robot.  Basically, it's a set of numbers for all the axes the robot can move in,
# and handlers that can be in response to button presses to manipulate those values.
# Button motions from joys are incremental, so basically this is keeping track of the
# current result of all the incremental changes, in every axis.


class ServoCartesianXyz(Handler):
    def __init__(self, name, controller):
        self.name = name
        self.coordinates = starting_pose[0:6]
        self.controller = controller
        self.updated = False

    def add_delta(self, axis, value):
        if (axis >= CartesianAxis.AXIS_ANGLE_4):
            # To control individual joint angles, we have to do some of the math directly, because the function we
            # want to use to control the robot (set_servo_cartesian) won't accept joint angles, only roll/pitch/yaw.  So we
            # calculate via inverse kinematics what our current joint positions are, modify those to account for
            # the control request, then feed them back in to forward kinematics to get the corresponding roll/pitch/yaw.
            # Then we can feed that into the actual control functions.
            current_pose = [self.coordinates[CartesianAxis.AXIS_X], self.coordinates[CartesianAxis.AXIS_Y],
                            self.coordinates[CartesianAxis.AXIS_Z], self.coordinates[CartesianAxis.AXIS_ROLL_X],
                            self.coordinates[CartesianAxis.AXIS_ROLL_Y], self.coordinates[CartesianAxis.AXIS_ROLL_Z]]
            joint_positions_with_err = self.controller.arm.get_inverse_kinematics(
                current_pose)
            if joint_positions_with_err[0] != 0:
                return
                # raise RuntimeError("Inverse kinematics failed")
            joint_positions = joint_positions_with_err[1]
            print(f"Joint Positions: {joint_positions}")
            joint_index = axis - CartesianAxis.AXIS_ANGLE_4 + 3
            joint_positions[joint_index] += value
            new_loc_with_err = self.controller.arm.get_forward_kinematics(
                joint_positions)
            if new_loc_with_err[0] != 0:
                raise RuntimeError("Forward kinematics failed")
            self.coordinates = new_loc_with_err[1]
        else:
            self.coordinates[axis] += value
        print(self.name, "Position:", self.coordinates)
        self.updated = True

    def notify_interval(self):
        if (self.updated):
            self.updated = False
            new_pose = [self.coordinates[CartesianAxis.AXIS_X], self.coordinates[CartesianAxis.AXIS_Y],
                        self.coordinates[CartesianAxis.AXIS_Z], self.coordinates[CartesianAxis.AXIS_ROLL_X],
                        self.coordinates[CartesianAxis.AXIS_ROLL_Y], self.coordinates[CartesianAxis.AXIS_ROLL_Z]]
            # For ROLL_X/Y/Z instead of ROLL/PITCH/YAW, use set_servo_cartesian_aa.  Cannot be used with AXIS_ANGLE_XXX
            code = self.controller.arm.set_servo_cartesian(
                new_pose, speed=my_speed, mvacc=my_mvacc)
            if not self.controller._check_code(code, 'set_servo_cartesian'):
                exit()

# Common functionality for both types of analog axis. This is further specified for Joy axis
# and analog Trigger axes


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

# These two classes smoosh references to our set_servo_cartesian object onto
# the implementations above


class TriggerServoControllerAxis(TriggerAnalogAxis):
    def __init__(self, name,  id, servo_cartesian_xyz, axis, multiplier):
        super().__init__(
            name,  id, initial_value=starting_pose[axis], multiplier=multiplier)
        super().set_handler(
            handler=lambda delta: servo_cartesian_xyz.add_delta(axis, delta))


class JoyServoControllerAxis(JoyAnalogAxis):
    def __init__(self, name, id,  servo_cartesian_xyz, axis, multiplier):
        super().__init__(
            name, id, initial_value=starting_pose[axis], multiplier=multiplier)
        super().set_handler(
            handler=lambda delta: servo_cartesian_xyz.add_delta(axis, delta))

# Base mapping. Further derived from below.


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


# Starting position of the robot, where the order of the numbers corresponds
# the order of axes in CartesianAxis.
starting_pose = [241, -23, 352, 180, 0, 0, 0, 0, 0]
# Experimentally determined values ot be sent to the Lite6 arm functions
my_speed = 30
my_mvacc = 5000


class XboxController:

    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    # Expirimentally determined.
    JOY_DEAD_ZONE = 0.05
    TRIG_DEAD_ZONE = 0.0

    # This is no longer used for the actual button mappings, but I'm keeping it here
    # because it tells me the codenames for all the buttons on the controller
    # that I'm might care about (BTN_EAST means B Button, and so on)
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
        self.arm.clean_error()
        self.arm.set_mode(mode=ArmMode.POSITION_CONTROL_MODE)
        self.arm.set_state(ArmState.SPORT_STATE)
        self.arm.set_position(*starting_pose[0:6], wait=True)
        self.arm.set_mode(mode=ArmMode.SERVO_MOTION_MODE)
        self.arm.set_state(ArmState.SPORT_STATE)
        self.arm.motion_enable(enable=True)
        self.it_changed = False
        self.cart = ServoCartesianXyz(
            "Robot position", self)

        # Defines all the buttons that we care about. Every "Button" is just an analog on/off button,
        # while JoyServerControllerAxis defines an analog axis that can have more complicated
        # behaviors assigned to the Joy.  Similar for the TriggerServoControllerAxis that can
        # be linked to analog triggers.
        self.buttons = [Button(id='BTN_NORTH', name='Y'),
                        Button(id='BTN_WEST', name='X'),
                        Button(id='BTN_TL', name='Left Bumper'),
                        Button(id='BTN_TR', name='Right Bumper'),
                        Button(id='BTN_SOUTH', name='A'),
                        # Every analog axis has to specify the id/code for the button, the button's pretty name,
                        # a reference to the ServoCartesian object we'll be updating, the axis on that ServoCartesian
                        # object that the axis applies to, and a multiplier that should be applied to the button values
                        # when updating the axis value.
                        JoyServoControllerAxis(
                            id='ABS_X', name="Left Joy X", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_X, multiplier=1.0),
                        JoyServoControllerAxis(
                            id='ABS_Y', name="Left Joy Y", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Y, multiplier=1.0),
                        JoyServoControllerAxis(
                            id='ABS_RX', name="Right Joy X", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_ANGLE_4, multiplier=1.0),
                        JoyServoControllerAxis(
                            id='ABS_RY', name="Right Joy Y", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_ANGLE_5, multiplier=1.0),
                        # JoyServoControllerAxis(
                        #     id='ABS_RY', name="Right Joy Y", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Z),
                        # Trigger ones work basically the same as the Joy ones - they're just using different deadzone constants inside.
                        TriggerServoControllerAxis(
                            id='ABS_RZ', name="Trig Right", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Z, multiplier=2.0),
                        TriggerServoControllerAxis(
                            id='ABS_Z', name="Trig Left", servo_cartesian_xyz=self.cart, axis=CartesianAxis.AXIS_Z, multiplier=-2.0)
                        ]

        # I fully admit this is sloppy, but this is where actions are set for the digital buttons deviced above. The index of
        # buttons is just an ordinal  - need to neaten this up somehow
        self.buttons[0].set_handler(
            ButtonAction.PRESS, lambda: self.arm.close_lite6_gripper())
        self.buttons[1].set_handler(
            ButtonAction.PRESS, lambda: self.arm.open_lite6_gripper())
        self.buttons[2].set_handler(
            ButtonAction.PRESS, lambda: self.cart.add_delta(CartesianAxis.AXIS_ANGLE_6, -1.0))
        self.buttons[3].set_handler(
            ButtonAction.PRESS, lambda: self.cart.add_delta(CartesianAxis.AXIS_ANGLE_6, 1.0))
        self.buttons[2].set_handler(
            ButtonAction.HOLD_REPEAT, lambda: self.cart.add_delta(CartesianAxis.AXIS_ANGLE_6, -1.0))
        self.buttons[3].set_handler(
            ButtonAction.HOLD_REPEAT, lambda: self.cart.add_delta(CartesianAxis.AXIS_ANGLE_6, 1.0))
        self.buttons[4].set_handler(
            ButtonAction.PRESS, lambda: self.arm.stop_lite6_gripper())

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


if __name__ == '__main__':
    joy = XboxController()
    # For now, just keep the program alive 5 minutes.  Ultimately we'll probably do some other processing here.
    # needs_update = True
    while (True):
        time.sleep(0.02)
        for button in joy.buttons:
            button.notify_interval()
        joy.cart.notify_interval()
