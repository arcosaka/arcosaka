#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
モーター制御
"""

import time
import pigpio

# Import the PCA9685 module.
import Adafruit_PCA9685

from mortor_consts import \
    ADDR_PWM, \
    DCROTATE, \
    DC_FREQ, \
    DC_DUTY, \
    DC_PLUS, \
    SERVO_MIN, \
    SERVO_MAX, \
    STEP_1PULSE, \
    STEP_FREQ, \
    STEP_DUTY

class MortorClass(object):
    """
    モータークラス
    """
    def __init__(self, port_a_cw=16, port_a_ccw=20, port_b_cw=19, port_b_ccw=26):
        # initialize gpio
        self.pic = pigpio.pi()

        self.port_a_cw = port_a_cw
        self.port_a_ccw = port_a_ccw
        self.port_b_cw = port_b_cw
        self.port_b_ccw = port_b_ccw

        # initialize move_dc
        self.dcduty_a_o = 0
        self.dcduty_b_o = 0

        # initialize move_servo
        self.pwm = Adafruit_PCA9685.PCA9685(ADDR_PWM)
        self.pwm.set_pwm_freq(60)

    def endfnc(self):
        """
        終了処理
        """
        self.move_dc_duty(self.port_a_cw, self.port_a_ccw, 0, 0)
        self.move_dc_duty(self.port_b_cw, self.port_b_ccw, 0, 0)
        self.pwm.set_all_pwm(0, 0) # 全モーターPWM解除

    def move_dc(self, rotate_a, rotate_b, limit=True):
        """
        DCモーター(なまし付き)
        """

        #とりあえず初期化
        dcduty_a = 0
        dcduty_b = 0

        while True:
            #Duty計算
            if rotate_a != DCROTATE.STOP:
                dcduty_a = self.dcduty_a_o + (DC_PLUS * rotate_a)
            elif self.dcduty_a_o > 0:
                dcduty_a = self.dcduty_a_o - DC_PLUS
            elif self.dcduty_a_o < 0:
                dcduty_a = self.dcduty_a_o + DC_PLUS

            if rotate_b != DCROTATE.STOP:
                dcduty_b = self.dcduty_b_o + (DC_PLUS * rotate_b)
            elif self.dcduty_b_o > 0:
                dcduty_b = self.dcduty_b_o - DC_PLUS
            elif self.dcduty_b_o < 0:
                dcduty_b = self.dcduty_b_o + DC_PLUS

            #上下限ガード
            if limit:
                if dcduty_a > DC_DUTY:
                    dcduty_a = DC_DUTY
                if dcduty_a < -DC_DUTY:
                    dcduty_a = -DC_DUTY
                if dcduty_b > DC_DUTY:
                    dcduty_b = DC_DUTY
                if dcduty_b < -DC_DUTY:
                    dcduty_b = -DC_DUTY

            if dcduty_a > 100:
                dcduty_a = 100
            if dcduty_a < -100:
                dcduty_a = -100

            if dcduty_b > 100:
                dcduty_b = 100
            if dcduty_b < -100:
                dcduty_b = -100

            # 前回値と同じなら処理を抜ける
            if dcduty_a == self.dcduty_a_o and dcduty_b == self.dcduty_b_o:
                break

            dcduty_a_cw = 0
            dcduty_a_ccw = 0
            if dcduty_a > 0:
                dcduty_a_cw = dcduty_a
            elif dcduty_a < 0:
                dcduty_a_ccw = dcduty_a

            dcduty_b_cw = 0
            dcduty_b_ccw = 0
            if dcduty_b > 0:
                dcduty_b_cw = dcduty_b
            elif dcduty_b < 0:
                dcduty_b_ccw = dcduty_b

            print("cw  A = %d" % dcduty_a_cw)
            print("ccw A = %d" % dcduty_a_ccw)
            print("cw  B = %d" % dcduty_b_cw)
            print("ccw B = %d" % dcduty_b_ccw)

            if self.port_a_cw >= 0 and self.port_a_ccw >= 0:
                self.move_dc_duty(self.port_a_cw, self.port_a_ccw, dcduty_a_cw, dcduty_a_ccw)

            if self.port_b_cw >= 0 and self.port_b_ccw >= 0:
                self.move_dc_duty(self.port_b_cw, self.port_b_ccw, dcduty_b_cw, dcduty_b_ccw)

            #今回値を保存
            self.dcduty_a_o = dcduty_a
            self.dcduty_b_o = dcduty_b
            
            time.sleep(0.05)

    def move_dc_duty(self, port_cw, port_ccw, dcduty_cw, dcduty_ccw):
        """
        DCモーターデューティー
        """
        #上下限ガード
        if dcduty_cw > 100:
            dcduty_cw = 100
        if dcduty_cw < 0:
            dcduty_cw = 0
        if dcduty_ccw > 100:
            dcduty_ccw = 100
        if dcduty_ccw < 0:
            dcduty_ccw = 0

        if dcduty_cw != 0 and dcduty_ccw != 0:
            dcduty_cw = 0
            dcduty_ccw = 0

        #新しい値で出力
        self.pic.set_PWM_frequency(port_cw, DC_FREQ)
        self.pic.set_PWM_frequency(port_ccw, DC_FREQ)

        self.pic.set_PWM_range(port_cw, 100)
        self.pic.set_PWM_range(port_ccw, 100)

        self.pic.set_PWM_dutycycle(port_cw, dcduty_cw)
        self.pic.set_PWM_dutycycle(port_ccw, dcduty_ccw)

    def move_servo(self, channel, power, limit=True):
        """
        サーボモーター
        """
        # 上下限ガード
        if power < 0:
            power = 0
        if power > 100:
            power = 100

        # 割合から
        pulse = ((SERVO_MAX - SERVO_MIN) * (power / 100.0)) + SERVO_MIN
        self.move_servo_pulse(channel, pulse, limit)

    def move_servo_pulse(self, channel, pulse, limit=True):
        """
        サーボモーターパルス
        """
        print("pulse = %d" % pulse)

        if limit:
            if pulse < SERVO_MIN:
                pulse = SERVO_MIN
            if pulse > SERVO_MAX:
                pulse = SERVO_MAX

        self.pwm.set_pwm(channel, 0, int(pulse))

    def move_step(self, port_a, port_b, distance):
        """
        ステッピングモーター
        """
        # 周波数とDuty比から1パルスの待機時間を計算
        step = int(distance / STEP_1PULSE)
        self.move_step_step(port_a, port_b, step)

    def move_step_step(self, port_a, port_b, step):
        """
        ステッピングモーターステップ数入力
        """
        print("step      = %d" % step)
        wait_hl = (STEP_FREQ * (STEP_DUTY / 100.0)) / 2.0
        wait_lh = (STEP_FREQ / 2.0) - wait_hl
        wait_hl /= 1000.0
        wait_lh /= 1000.0
        
        print("STEP_FREQ = %d" % STEP_FREQ)
        print("STEP_DUTY = %d" % STEP_DUTY)
        print("wait_hl   = %d" % wait_hl)
        print("wait_lh   = %d" % wait_lh)
        self.pic.write(18, pigpio.HIGH)

        for i in range(step):
            if step > 0:
                self.pic.write(port_a, pigpio.HIGH)
                print("port_a ON")
                time.sleep(wait_hl)
                self.pic.write(port_b, pigpio.HIGH)
                print("port_b ON")
                time.sleep(wait_hl)
                self.pic.write(port_a, pigpio.LOW)
                print("port_a OFF")
                time.sleep(wait_lh)
                self.pic.write(port_b, pigpio.LOW)
                print("port_b OFF")
                time.sleep(wait_lh)

            elif step < 0:
                self.pic.write(port_b, pigpio.HIGH)
                print("port_b ON")
                time.sleep(wait_hl)
                self.pic.write(port_a, pigpio.HIGH)
                print("port_a ON")
                time.sleep(wait_hl)
                self.pic.write(port_b, pigpio.LOW)
                print("port_b OFF")
                time.sleep(wait_lh)
                self.pic.write(port_a, pigpio.LOW)
                print("port_a OFF")
                time.sleep(wait_lh)
            print("pulse     = %d/%d" % (i,step))

        self.pic.write(18, pigpio.LOW)
        self.pic.write(port_a, pigpio.LOW)
        self.pic.write(port_b, pigpio.LOW)
        print("move_step_step end")
