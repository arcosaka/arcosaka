from pyPS4Controller.controller import Controller
from pyPS4Controller.event_mapping.DefaultMapping import DefaultMapping
import pigpio


PWM_PIN = [12, 16, 20, 5, 21, 6, 19, 26, 25] #PWM IN1,IN2
UP = 16
DOWN = 19
LEFT = 5
RIGHT = 21
UPPERLEFT = 12
UPPERRIGHT = 20
LOWERLEFT = 6
LOWERRIGHT = 26
NEUTRAL = 25 
FREQ = 100
RANGE = 100

ON = 1
OFF = 0
        
# 実行前にsudo pigpiodを忘れずに
pi = pigpio.pi()
for j in range(9):
    pi.set_mode(PWM_PIN[j], pigpio.OUTPUT)
    pi.set_PWM_frequency(PWM_PIN[j], FREQ)  #PWMの周波数(Hz)を指定。デフォルトは800。
    pi.set_PWM_range(PWM_PIN[j], RANGE)  #PWM値(μs)の最大値を指定。指定可能な値は25〜40000。デフォルトは255。
pi.set_mode(18, pigpio.OUTPUT)

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        for j in range(9):
            pi.set_PWM_dutycycle(PWM_PIN[j], 0)
        pi.set_PWM_dutycycle(NEUTRAL, 100)
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF


    def on_x_press(self):
        print("on_x_press")
        pi.write(13, 1)


    def on_x_release(self):
        print("on_x_release")
        pi.write(13, 0)


    def on_triangle_press(self):
        print("on_triangle_press")
        pi.write(23, 1)


    def on_triangle_release(self):
        print("on_triangle_release")
        pi.write(23, 0)


    def on_circle_press(self):
        print("on_circle_press")
        pi.write(24, 1)


    def on_circle_release(self):
        print("on_circle_release")
        pi.write(24, 0)


    def on_square_press(self):
        print("on_square_press")
        pi.write(22, 1)


    def on_square_release(self):
        print("on_square_release")
        pi.write(22, 0)


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
        self.on_up_arrow_press_value = ON
        print("on_up_arrow_press")
        if (self.on_left_arrow_press_value == ON):
            pi.set_PWM_dutycycle(UPPERLEFT, 100)
            pi.set_PWM_dutycycle(LEFT, 0)
        elif (self.on_right_arrow_press_value == ON):
            pi.set_PWM_dutycycle(UPPERRIGHT, 100)
            pi.set_PWM_dutycycle(RIGHT, 0)
        else:
            pi.set_PWM_dutycycle(UP, 100)
        pi.set_PWM_dutycycle(NEUTRAL, 0)
        

    def on_up_down_arrow_release(self):
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        print("on_up_down_arrow_release")
        pi.set_PWM_dutycycle(UP, 0)
        pi.set_PWM_dutycycle(DOWN, 0)
        pi.set_PWM_dutycycle(UPPERLEFT, 0)
        pi.set_PWM_dutycycle(UPPERRIGHT, 0)
        pi.set_PWM_dutycycle(LOWERLEFT, 0)
        pi.set_PWM_dutycycle(LOWERRIGHT, 0)
        if (self.on_left_arrow_press_value == ON):
            pi.set_PWM_dutycycle(LEFT, 100)
        elif (self.on_right_arrow_press_value == ON):
            pi.set_PWM_dutycycle(RIGHT, 100)
        else:
            pi.set_PWM_dutycycle(NEUTRAL, 100)


    def on_down_arrow_press(self):
        self.on_down_arrow_press_value = ON
        print("on_down_arrow_press")
        if (self.on_left_arrow_press_value == ON):
            pi.set_PWM_dutycycle(LOWERLEFT, 100)
            pi.set_PWM_dutycycle(LEFT, 0)
        elif (self.on_right_arrow_press_value == ON):
            pi.set_PWM_dutycycle(LOWERRIGHT, 100)
            pi.set_PWM_dutycycle(RIGHT, 0)
        else:
            pi.set_PWM_dutycycle(DOWN, 100)
        pi.set_PWM_dutycycle(NEUTRAL, 0)


    def on_left_arrow_press(self):
        self.on_left_arrow_press_value = ON
        print("on_left_arrow_press")
        if (self.on_up_arrow_press_value == ON):
            pi.set_PWM_dutycycle(UPPERLEFT, 100)
            pi.set_PWM_dutycycle(UP, 0)
        elif (self.on_down_arrow_press_value == ON):
            pi.set_PWM_dutycycle(LOWERLEFT, 100)
            pi.set_PWM_dutycycle(DOWN, 0)
        else:
            pi.set_PWM_dutycycle(LEFT, 100)
        pi.set_PWM_dutycycle(NEUTRAL, 0)


    def on_left_right_arrow_release(self):
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF
        print("on_left_right_arrow_release")
        pi.set_PWM_dutycycle(LEFT, 0)
        pi.set_PWM_dutycycle(RIGHT, 0)
        pi.set_PWM_dutycycle(UPPERLEFT, 0)
        pi.set_PWM_dutycycle(UPPERRIGHT, 0)
        pi.set_PWM_dutycycle(LOWERLEFT, 0)
        pi.set_PWM_dutycycle(LOWERRIGHT, 0)
        if (self.on_up_arrow_press_value == ON):
            pi.set_PWM_dutycycle(UP, 100)
        elif (self.on_down_arrow_press_value == ON):
            pi.set_PWM_dutycycle(DOWN, 100)
        else:
            pi.set_PWM_dutycycle(NEUTRAL, 100)


    def on_right_arrow_press(self):
        self.on_right_arrow_press_value = ON
        print("on_right_arrow_press")
        if (self.on_up_arrow_press_value == ON):
            pi.set_PWM_dutycycle(UPPERRIGHT, 100)
            pi.set_PWM_dutycycle(UP, 0)
        elif (self.on_down_arrow_press_value == ON):
            pi.set_PWM_dutycycle(LOWERRIGHT, 100)
            pi.set_PWM_dutycycle(DOWN, 0)
        else:
            pi.set_PWM_dutycycle(RIGHT, 100)
        pi.set_PWM_dutycycle(NEUTRAL, 0)


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
