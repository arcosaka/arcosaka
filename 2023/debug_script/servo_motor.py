# -*- coding: utf-8 -*-
from pyPS4Controller.controller import Controller
from pyPS4Controller.event_mapping.DefaultMapping import DefaultMapping
import pigpio
import Adafruit_PCA9685

SERVO_FREQ = 60
SERVO_MIN = 200
SERVO_MID = 400
SERVO_MAX = 600

ON = 1
OFF = 0 

class ServoMortor(object):
    """
    サーボモータークラス
    """

    def __init__(self):
        # initialize move_servo
        self.pwm = Adafruit_PCA9685.PCA9685(0x41)
        self.pwm.set_pwm_freq(SERVO_FREQ)

    # end __init__


    def move_servo_pulse(self, channel, pulse):
        """
        サーボモーターパルス
        """

        if pulse > SERVO_MAX:
            pulse = SERVO_MAX
        elif pulse < SERVO_MIN:
            pulse = SERVO_MIN
        else:
            pass

        self.pwm.set_pwm(channel, 0, int(pulse))
    # end move_servo_pulse

# end ServoMortor





class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.servo = ServoMortor()
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF
        
        self.servo.move_servo_pulse(0, SERVO_MIN)
        self.servo.move_servo_pulse(1, SERVO_MIN)


    def on_x_press(self):
        print("on_x_press")


    def on_x_release(self):
        print("on_x_release")


    def on_triangle_press(self):
        print("on_triangle_press")


    def on_triangle_release(self):
        print("on_triangle_release")


    def on_circle_press(self):
        print("on_circle_press")


    def on_circle_release(self):
        print("on_circle_release")


    def on_square_press(self):
        print("on_square_press")


    def on_square_release(self):
        print("on_square_release")


    def on_L1_press(self):
        print("on_L1_press")


    def on_L1_release(self):
        print("on_L1_release")


    def on_L2_press(self, value):
        print("on_L2_press: {}".format(value))


    def on_L2_release(self):
        print("on_L2_release")


    def on_R1_press(self):
        print("on_R1_press")


    def on_R1_release(self):
        print("on_R1_release")


    def on_R2_press(self, value):
        print("on_R2_press: {}".format(value))


    def on_R2_release(self):
        print("on_R2_release")


    def on_up_arrow_press(self):
        print("on_up_arrow_press")
        self.on_up_arrow_press_value = ON
        self.servo.move_servo_pulse(0, SERVO_MAX)
        

    def on_up_down_arrow_release(self):
        print("on_up_down_arrow_release")
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        self.servo.move_servo_pulse(0, SERVO_MIN)


    def on_down_arrow_press(self):
        print("on_down_arrow_press")
        self.on_down_arrow_press_value = ON
        #self.servo.move_servo_pulse(0, SERVO_MIN)


    def on_left_arrow_press(self):
        print("on_left_arrow_press")
        self.on_left_arrow_press_value = ON
        self.servo.move_servo_pulse(1, SERVO_MAX)


    def on_left_right_arrow_release(self):
        print("on_left_right_arrow_release")
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF
        self.servo.move_servo_pulse(1, SERVO_MIN)


    def on_right_arrow_press(self):
        print("on_right_arrow_press")
        self.on_right_arrow_press_value = ON
        #self.servo.move_servo_pulse(1, SERVO_MIN)


    def on_L3_up(self, value):
        print("on_L3_up: {}".format(value))


    def on_L3_down(self, value):
        print("on_L3_down: {}".format(value))


    def on_L3_left(self, value):
        print("on_L3_left: {}".format(value))


    def on_L3_right(self, value):
        print("on_L3_right: {}".format(value))


    def on_L3_y_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        print("on_L3_y_at_rest")


    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        print("on_L3_x_at_rest")


    def on_L3_press(self):
        """L3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        print("on_L3_press")


    def on_L3_release(self):
        """L3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        print("on_L3_release")


    def on_R3_up(self, value):
        print("on_R3_up: {}".format(value))


    def on_R3_down(self, value):
        print("on_R3_down: {}".format(value))


    def on_R3_left(self, value):
        print("on_R3_left: {}".format(value))


    def on_R3_right(self, value):
        print("on_R3_right: {}".format(value))


    def on_R3_y_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        print("on_R3_y_at_rest")


    def on_R3_x_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        print("on_R3_x_at_rest")


    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        print("on_R3_press")


    def on_R3_release(self):
        """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        print("on_R3_release")


    def on_options_press(self):
        print("on_options_press")


    def on_options_release(self):
        print("on_options_release")


    def on_share_press(self):
        """this event is only detected when connecting without ds4drv"""
        print("on_share_press")


    def on_share_release(self):
        """this event is only detected when connecting without ds4drv"""
        print("on_share_release")


    def on_playstation_button_press(self):
        """this event is only detected when connecting without ds4drv"""
        print("on_playstation_button_press")


    def on_playstation_button_release(self):
        """this event is only detected when connecting without ds4drv"""
        print("on_playstation_button_release")



controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
#controller.debug = True
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)
