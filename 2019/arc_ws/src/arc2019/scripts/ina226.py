#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
電流電圧測定IC
"""

from subprocess import call
import time
import smbus

from ina226_consts import *

class Ina226Class(object):
    """
    電流電圧測定IC値取得クラス
    """
    def __init__(self):
        self.is_enable = False
        try:
            self.i2c = smbus.SMBus(1)
            #ICの設定
            setdata = (SET_RST << 12) + (SET_AVG << 9) + \
                (SET_VBUSCT << 6) + (SET_VSHCT << 3) + SET_MODE
            setdata = ((setdata << 8) & 0xFF00) + (setdata >> 8)
            self.i2c.write_word_data(I2C_INA226, ADDR_S, setdata)
            #キャリブレーションレジスタの設定
            # 0.00512/((0.0015[mΩ])*0.001)
            setdata = int(0.00512/((BATT_R)*0.001))
            setdata = ((setdata << 8) & 0xFF00) + (setdata >> 8)
            self.i2c.write_word_data(I2C_INA226, ADDR_R, setdata)
            self.is_enable = True
            #1回目は変な値をとるときが多いので...
            self.read_v()
            self.read_i()

        finally:
            self.is_battlow = False
            self.cnt_battlow = 0
            self.v_ave = 0
            self.i_ave = 0
            self.i_sgm = 0
    #end __init__

    def read_v(self):
        """
        電圧読み取り
        """
        volt = 0.0
        if self.is_enable:
            word = self.i2c.read_word_data(I2C_INA226, ADDR_V) & 0xFFFF
            result = ((word << 8) & 0xFF00) + (word >> 8)
            volt = result * 1.25 / 1000

        return volt
    #end read_v

    def read_i(self):
        """
        電流読み取り
        """
        curr = 0.0
        if self.is_enable:
            word = self.i2c.read_word_data(I2C_INA226, ADDR_I) & 0xFFFF
            curr = ((word << 8) & 0xFF00) + (word >> 8)
        return curr
    #end read_i

    def read_vi_loop(self):
        """
        電流電圧読み取りループ
        """
        v_aves = []
        i_aves = []
        while self.is_enable:
            v_now = self.read_v()
            v_aves.append(v_now)
            if v_aves.count() > 100:
                del v_aves[0]
            self.v_ave = sum(v_aves) / v_aves.count()

            i_now = self.read_i() / 1000.0
            self.i_sgm += i_now
            i_aves.append(i_now)
            if i_aves.count() > 100:
                del i_aves[0]
            self.i_ave = sum(i_aves) / i_aves.count()

            print "NOW={:.2f}[V]\t".format(v_now) + "AVE={:.2f}[V]\t".format(self.v_ave) + \
                "NOW={:.4f}[A]\t".format(i_now) + "AVE={:.4f}[A]\t".format(self.i_ave) + \
                "SGM={:.4f}[Asec]\t".format(self.i_sgm)

            if self.v_ave < BATT_VLOW:
                self.cnt_battlow += 1
                print "電圧低下(" + str(self.cnt_battlow) + ")"
            elif self.i_ave > BATT_IHI:
                #self.cnt_battlow += 1
                print "電流異常(" + str(self.cnt_battlow) + ")"
            else:
                self.cnt_battlow = 0

            if self.cnt_battlow > BATT_ERR:
                print "1分後にシャットダウンします"
                call('sudo shutdown -h 1', shell=True)
                self.is_enable = False
            else:
                pass

            time.sleep(1)
    #end read_vi_loop

if __name__ == '__main__':
    inac = Ina226Class()
    inac.read_vi_loop()
