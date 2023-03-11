# Props to https://github.com/kevinhughes27/TensorKart for helping figure out some of the button constants for the XBox controller
from inputs import get_gamepad
import math
import threading
import time


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

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                for mapping in self.button_mappings:
                    if event.code == mapping.code:
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


if __name__ == '__main__':
    joy = XboxController()
    # For now, just keep the program alive 5 minutes.  Ultimately we'll probably do some other processing here.
    time.sleep(300)