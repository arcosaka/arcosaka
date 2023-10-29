import time
import threading
from pyPS4Controller.controller import Controller
import Adafruit_PCA9685
import pigpio

#COMMON
ON = 1
OFF = 0

#ARM
ARM_MOTORA = 11 
ARM_MOTORB = 8
ARM_ENBL = 18
ARM_FREQ = 300
ARM_DUTY= 50

#HAND
HAND_MOTORA = 0
HAND_MOTORB = 1
HAND_FREQ = 60
HAND_MINA = 200
HAND_MAXA = 600
HAND_MINB = 200
HAND_MAXB = 600

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
FOOT_DUTY = 90


class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.up_running = False
        self.output_up_thread = None
        self.down_running = False
        self.output_down_thread = None

        
        # initialize gpio
        self.pi = pigpio.pi()

        #ARM
        self.pi.set_mode(ARM_MOTORA, pigpio.OUTPUT)
        self.pi.set_mode(ARM_MOTORB, pigpio.OUTPUT)
        self.pi.set_mode(ARM_ENBL, pigpio.OUTPUT)
        self.pi.write(ARM_MOTORA, OFF)
        self.pi.write(ARM_MOTORB, OFF)
        self.pi.write(ARM_ENBL, OFF)
        self.wait_hl = (1.0 / ARM_FREQ * (ARM_DUTY / 100.0))
        self.wait_lh = (1.0 / ARM_FREQ * (1 - (ARM_DUTY / 100.0)))
        
        #HAND
        self.pwm = Adafruit_PCA9685.PCA9685(0x41)
        self.pwm.set_pwm_freq(HAND_FREQ)

        #FOOT
        self.FOOT_PINS = [FOOT_MOTORA1,FOOT_MOTORA2,FOOT_MOTORB1,FOOT_MOTORB2,FOOT_MOTORC1,FOOT_MOTORC2,FOOT_MOTORD1,FOOT_MOTORD2]
        for j in range(len(self.FOOT_PINS)):
            self.pi.set_mode(self.FOOT_PINS[j], pigpio.OUTPUT)  #pigpioで制御するピンの指定
            self.pi.set_PWM_frequency(self.FOOT_PINS[j], FOOT_FREQ)  #PWMの周波数(Hz)を指定。デフォルトは800。
            self.pi.set_PWM_range(self.FOOT_PINS[j], FOOT_RANGE)  #PWM値(μs)の最大値を指定。指定可能な値は25〜40000。デフォルトは255。
            self.pi.set_PWM_dutycycle(self.FOOT_PINS[j], OFF)
        
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF


#ARM
    def on_R3_up(self, value):
        print("on_R3_up: {}".format(value))
        if not self.up_running:
            self.up_running = True
            self.output_up_thread = threading.Thread(target=self.start_up_output)
            self.output_up_thread.start()

    def on_R3_down(self, value):
        print("on_R3_down: {}".format(value))
        if not self.down_running:
            self.down_running = True
            self.output_down_thread = threading.Thread(target=self.start_down_output)
            self.output_down_thread.start()

    def on_R3_y_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        print("on_R3_y_at_rest")
        if self.output_up_thread is not None:
            self.up_running = False
            self.output_up_thread.join()
        if self.output_down_thread is not None:
            self.down_running = False
            self.output_down_thread.join()
        self.pi.write(ARM_MOTORA, OFF)
        self.pi.write(ARM_MOTORB, OFF)
        self.pi.write(ARM_ENBL, OFF)

    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        print("on_R3_press")
        if self.output_up_thread is not None:
            self.up_running = False
            self.output_up_thread.join()
        if self.output_down_thread is not None:
            self.down_running = False
            self.output_down_thread.join()
        self.pi.write(ARM_MOTORA, OFF)
        self.pi.write(ARM_MOTORB, OFF)
        self.pi.write(ARM_ENBL, OFF)

    def start_up_output(self):
        while self.up_running:
            print("出力中...")
            self.pi.write(ARM_ENBL, ON)
            
            self.pi.write(ARM_MOTORA, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(ARM_MOTORB, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(ARM_MOTORA, OFF)
            time.sleep(self.wait_lh/2)
            self.pi.write(ARM_MOTORB, OFF)
            time.sleep(self.wait_lh/2)

    def start_down_output(self):
        while self.down_running:
            print("出力中...")
            self.pi.write(ARM_ENBL, ON)
            
            self.pi.write(ARM_MOTORB, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(ARM_MOTORA, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(ARM_MOTORB, OFF)
            time.sleep(self.wait_lh/2)
            self.pi.write(ARM_MOTORA, OFF)
            time.sleep(self.wait_lh/2)


#HAND
    def on_L1_press(self):
        print("on_L1_press")
        self.pwm.set_pwm(HAND_MOTORA, 0, int(HAND_MINA))

    def on_L2_press(self, value):
        print("on_L2_press: {}".format(value))
        self.pwm.set_pwm(HAND_MOTORA, 0, int(HAND_MAXA))

    def on_R1_press(self):
        print("on_R1_press")
        self.pwm.set_pwm(HAND_MOTORB, 0, int(HAND_MINB))

    def on_R2_press(self, value):
        print("on_R2_press: {}".format(value))
        self.pwm.set_pwm(HAND_MOTORB, 0, int(HAND_MAXB))


#FOOT
    def on_up_arrow_press(self):
        print("on_up_arrow_press")
        if ( self.on_down_arrow_press_value == OFF
        and self.on_left_arrow_press_value == OFF
        and self.on_right_arrow_press_value == OFF ):
            self.on_up_arrow_press_value = ON
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
        if ( self.on_up_arrow_press_value == OFF
        and self.on_left_arrow_press_value == OFF
        and self.on_right_arrow_press_value == OFF ):
            self.on_down_arrow_press_value = ON        
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
        self.on_up_arrow_press_value = OFF
        self.on_down_arrow_press_value = OFF
        
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
        if ( self.on_up_arrow_press_value == OFF
        and self.on_down_arrow_press_value == OFF
        and self.on_right_arrow_press_value == OFF ):
            self.on_left_arrow_press_value = ON
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
        if ( self.on_up_arrow_press_value == OFF
        and self.on_down_arrow_press_value == OFF
        and self.on_left_arrow_press_value == OFF ):
            self.on_right_arrow_press_value = ON
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
        self.on_left_arrow_press_value = OFF
        self.on_right_arrow_press_value = OFF
        
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
        if ( self.on_up_arrow_press_value == OFF
        and self.on_down_arrow_press_value == OFF
        and self.on_left_arrow_press_value == OFF
        and self.on_right_arrow_press_value == OFF ):
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
        if ( self.on_up_arrow_press_value == OFF
        and self.on_down_arrow_press_value == OFF
        and self.on_left_arrow_press_value == OFF
        and self.on_right_arrow_press_value == OFF ):
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
