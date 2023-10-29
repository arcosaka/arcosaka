# -*- coding: utf-8 -*-
from pyPS4Controller.controller import Controller
from pyPS4Controller.event_mapping.DefaultMapping import DefaultMapping
import pigpio

MOTORA = [16,20] #PWM AIN1,AIN2
MOTORB = [19,26] #PWM BIN1,BIN2
FREQ = 100
RANGE = 100
ON = 1
OFF = 0

class DCMotor(object):

    def __init__(self, pin0, pin1):
        self.PINS = [pin0,pin1]
        self.pi = pigpio.pi()
        print("PIN " + str(self.PINS[0]) + ":" + str(self.PINS[1]))
        for j in range(2):
            self.pi.set_mode(self.PINS[j], pigpio.OUTPUT)  #pigpioで制御するピンの指定
            self.pi.set_PWM_frequency(self.PINS[j], FREQ)  #PWMの周波数(Hz)を指定。デフォルトは800。
            self.pi.set_PWM_range(self.PINS[j], RANGE)  #PWM値(μs)の最大値を指定。指定可能な値は25〜40000。デフォルトは255。
            self.pi.set_PWM_dutycycle(self.PINS[j], 0)


    def changeDuty(self,value):
        if controller.on_up_arrow_press_value == ON:  #正転
            self.pi.set_PWM_dutycycle(self.PINS[0], 95)
            self.pi.set_PWM_dutycycle(self.PINS[1], 0)
        elif controller.on_down_arrow_press_value == ON:  #逆転
            self.pi.set_PWM_dutycycle(self.PINS[0], 0)
            self.pi.set_PWM_dutycycle(self.PINS[1], 95)
        else:
            self.pi.set_PWM_dutycycle(self.PINS[0], 0)
            self.pi.set_PWM_dutycycle(self.PINS[1], 0)



class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.motor_a = DCMotor(MOTORA[0], MOTORA[1])
        self.motor_b = DCMotor(MOTORB[0], MOTORB[1])
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF


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
        self.on_down_arrow_press_value = OFF
        self.motor_a.changeDuty(self.on_up_arrow_press_value)
        self.motor_b.changeDuty(self.on_down_arrow_press_value)
        

    def on_up_down_arrow_release(self):
        print("on_up_down_arrow_release")
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        self.motor_a.changeDuty(self.on_up_arrow_press_value)
        self.motor_b.changeDuty(self.on_down_arrow_press_value)


    def on_down_arrow_press(self):
        print("on_down_arrow_press")
        self.on_down_arrow_press_value = ON
        self.on_up_arrow_press_value = OFF
        self.motor_a.changeDuty(self.on_up_arrow_press_value)
        self.motor_b.changeDuty(self.on_down_arrow_press_value)


    def on_left_arrow_press(self):
        print("on_left_arrow_press")
        self.on_left_arrow_press_value = ON


    def on_left_right_arrow_release(self):
        print("on_left_right_arrow_release")
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF


    def on_right_arrow_press(self):
        print("on_right_arrow_press")
        self.on_right_arrow_press_value = ON


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
