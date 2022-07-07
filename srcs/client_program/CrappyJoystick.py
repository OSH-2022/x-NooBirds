
from donkeycar.parts.controller import Joystick, JoystickController


class CrappyJoystick(Joystick):
    #An interface to a physical joystick available at /dev/input/js0
    def __init__(self, *args, **kwargs):
        super(CrappyJoystick, self).__init__(*args, **kwargs)

            
        self.button_names = {
            0x133 : 'x',
            0x130 : 'y',
            0x131 : 'b',
            0x132 : 'a',
        }


        self.axis_names = {
            0x1 : 'y1',
            0x0 : 'x1',
            0x2 : 'x2',
            0x5 : 'y2',
        }



class CrappyJoystickController(JoystickController):
    #A Controller object that maps inputs to actions
    def __init__(self, *args, **kwargs):
        super(CrappyJoystickController, self).__init__(*args, **kwargs)


    def init_js(self):
        #attempt to init joystick
        try:
            self.js = CrappyJoystick(self.dev_fn)
            self.js.init()
        except FileNotFoundError:
            print(self.dev_fn, "not found.")
            self.js = None
        return self.js is not None


    def init_trigger_maps(self):
        #init set of mapping from buttons to function calls
            
        self.button_down_trigger_map = {
            'x' : self.emergency_stop,
        }


        self.axis_trigger_map = {
            'x1' : self.set_steering,
            'y2' : self.set_throttle,
        }


