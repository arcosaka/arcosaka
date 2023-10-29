# -*- coding: utf-8 -*-
from pyPS4Controller.controller import Controller
from pyPS4Controller.event_mapping.DefaultMapping import DefaultMapping
import pigpio

#COMMON
ON = 1
OFF = 0

#FOOT
FOOT_MOTORA1 = 16   #FRONT_LEFT
FOOT_MOTORA2 = 20   #FRONT_LEFT
FOOT_MOTORB1 = 19   #FRONT_RIGHT
FOOT_MOTORB2 = 26   #FRONT_RIGHT
FOOT_MOTORC1 = 6   #REAR_LEFT
FOOT_MOTORC2 = 13   #REAR_LEFT
FOOT_MOTORD1 = 14   #REAR_RIGHT
FOOT_MOTORD2 = 7   #REAR_RIGHT
FOOT_FREQ = 100
FOOT_RANGE = 100
FOOT_DUTY = 50


class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
        # initialize gpio
        self.pi = pigpio.pi()

        #FOOT
        self.FOOT_PINS = [FOOT_MOTORA1,FOOT_MOTORA2,FOOT_MOTORB1,FOOT_MOTORB2,FOOT_MOTORC1,FOOT_MOTORC2,FOOT_MOTORD1,FOOT_MOTORD2]
        for j in range(len(self.FOOT_PINS)):
            self.pi.set_mode(self.FOOT_PINS[j], pigpio.OUTPUT)  #pigpioで制御するピンの指定
            self.pi.set_PWM_frequency(self.FOOT_PINS[j], FOOT_FREQ)  #PWMの周波数(Hz)を指定。デフォルトは800。
            self.pi.set_PWM_range(self.FOOT_PINS[j], FOOT_RANGE)  #PWM値(μs)の最大値を指定。指定可能な値は25〜40000。デフォルトは255。
            self.pi.set_PWM_dutycycle(self.FOOT_PINS[j], OFF)

#FOOT
    def on_up_arrow_press(self):
        print("on_up_arrow_press")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, OFF)

    def on_down_arrow_press(self):
        print("on_down_arrow_press")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, FOOT_DUTY)

    def on_up_down_arrow_release(self):
        print("on_up_down_arrow_release")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, OFF)

    def on_left_arrow_press(self):
        print("on_left_arrow_press")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, FOOT_DUTY)

    def on_right_arrow_press(self):
        print("on_right_arrow_press")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, OFF)

    def on_left_right_arrow_release(self):
        print("on_left_right_arrow_release")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, OFF)

    def on_L3_left(self, value):
        print("on_L3_left: {}".format(value))
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, OFF)

    def on_L3_right(self, value):
        print("on_L3_right: {}".format(value))
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, FOOT_DUTY)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, FOOT_DUTY)

    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        print("on_L3_x_at_rest")
        self.pi.set_PWM_dutycycle(FOOT_MOTORA1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORA2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORB2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORC2, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD1, OFF)
        self.pi.set_PWM_dutycycle(FOOT_MOTORD2, OFF)

if __name__ == "__main__":
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
