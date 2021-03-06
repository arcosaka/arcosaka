#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import volcurmeas

CYCLES = 60 #処理周波数
# 定数などの定義ファイルimport


# class  定義
class VOLCURMEAS_DEV(object):
    """
    電圧電流測定ICの動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        self.cyclecount = 0
        # 受信作成
        #self.sub_client  = rospy.Subscriber('main', main, self.mainCallback, queue_size=1)
        # 送信作成
        self.pub_volcurmeas = rospy.Publisher('volcurmeas', volcurmeas, queue_size=100)
        # messageのインスタンスを作る
        self.msg_volcurmeas = volcurmeas()

        #GPIOの初期設定
        #self.pi = pigpio.pi()
        #self.pi.set_mode(PIN_EMG, pigpio.INPUT)

#--------------------
# 受信コールバック
#    def mainCallback(self, main_msg):
#        """
#        mainの受信コールバック
#        """
#        #print("led a" + str(main_msg.led_a_value)) 
#        self.pi.set_PWM_dutycycle(PIN_A,main_msg.led_a_value)
#        self.pi.set_PWM_dutycycle(PIN_B,main_msg.led_b_value)
#--------------------
    def main(self):
        # メッセージを発行する
        self.msg_volcurmeas.volt = 7.4
        self.msg_volcurmeas.cur = 0.5
        self.pub_volcurmeas.publish(self.msg_volcurmeas)

def volcurmeas_py():
    # 初期化宣言 : このソフトウェアは"led_py_node"という名前
    rospy.init_node('volcurmeas_py_node', anonymous=True)
    # インスタンスの作成 
    volcurmeas_dev = VOLCURMEAS_DEV()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[volcurmeas] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        volcurmeas_dev.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        volcurmeas_py()

    except rospy.ROSInterruptException:
        print("[volcurmeas] end")
