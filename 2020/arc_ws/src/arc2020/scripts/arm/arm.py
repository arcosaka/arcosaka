#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio
import time

# モーター制御用
import mortor

# 自分で定義したmessageファイルから生成されたモジュール
#受信
from arc2020.msg import main

#送信
from arc2020.msg import arm

# 定数などの定義ファイルimport
from arm_consts import *

CYCLES = 60 

# OPERATION
OPE_START = 1
OPE_PAUSE = 2
OPE_STOP  = 0

# MOVE_STATUS
MOVE_INPROGRESS = 0
MOVE_COMPLETE = 1
MOVE_IDLE = 2
MOVE_INPROGRESS_PAUSE = 3

LIMIT_WIDTH_SYSTEM_ORDER = 273
LIMIT_WIDTH_CM    = 45
UNE_WIDTH_CM      = 30
SYSTEMORDER_PER_CM = LIMIT_WIDTH_SYSTEM_ORDER/LIMIT_WIDTH_CM

ARM_OFFSET = (LIMIT_WIDTH_SYSTEM_ORDER - (UNE_WIDTH_CM*SYSTEMORDER_PER_CM))/2

class Arm(object):
    """
    全体の動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        # 受信作成
        self.sub_main  = rospy.Subscriber('main', main, self.mainCallback, queue_size=1)
        
        # 送信作成
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        
        # messageのインスタンスを作る
        self.msg_arm = arm()
        
        # 送信メッセージ初期化
        self.clearMsg()
        
        # MortorClass        
        self.svmc = mortor.ServoMortorClass(DEBUG_ARM)
        self.svmc.move_servo(CHANNEL_HAND, 80)
        
        self.stmc = mortor.StepMortorClass(DEBUG_ARM, (PORT_HANDV_A, PORT_HANDV_B), (LIM_HANDV_MIN, LIM_HANDV_MAX))
        time.sleep(1)
        self.stmc.move_posinit_step()
        

        self.move_status = MOVE_IDLE
        self.operation = OPE_PAUSE
        self.y_index = -1
        
        
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        self.msg_arm.drillstatus = 1
        self.msg_arm.retorgstatus = 0
        
#--------------------
# 受信コールバック
    def mainCallback(self, main_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ受信
        self.ridge_width = main_msg.arm_ridge_width
        self.drill_width = main_msg.arm_drill_width
        
        self.operation = main_msg.arm_drill_req


#--------------------
# ステッピングモータ動作関数
    def movestep(self):
        handy = ARM_OFFSET + (self.drill_width[self.y_index] * SYSTEMORDER_PER_CM)
        if handy < 0 :
            handy = 0
        elif handy > LIMIT_WIDTH_SYSTEM_ORDER :
            handy = LIMIT_WIDTH_SYSTEM_ORDER
        
        self.stmc.move_step(handy)  # y軸移動
        time.sleep(2)


#--------------------
# サーボモータ動作関数
    def moveservo(self):
        self.svmc.move_servo(CHANNEL_HAND, 45)  # z軸移動
        time.sleep(0.5)
        self.svmc.move_servo(CHANNEL_HAND, 80)  # z軸移動
        time.sleep(2)


#--------------------
# メイン関数
    def main(self):
        #print("arm:"+str(self.move_status)+","+str(self.stepstatus)+","+str(self.msg_arm.drill_req_step)+","+str(self.servostatus))
        if self.move_status == MOVE_IDLE:
            # -> INPROGRESS
            if self.operation == OPE_START:
                self.move_status = MOVE_INPROGRESS
                self.y_index = self.y_index + 1
                self.msg_arm.drillstatus = 0
        
        # INPROGRESS
        elif self.move_status == MOVE_INPROGRESS:
            # -> COMPLETE
            #self.msg_arm.drillstatus = 0
            self.movestep()
            self.moveservo()
            self.move_status = MOVE_COMPLETE
        
        # INPROGRESS_PAUSE
        elif self.move_status == MOVE_INPROGRESS_PAUSE:
            # -> INPROGRESS
            if self.operation == OPE_START:
                self.move_status = MOVE_INPROGRESS
        
        # COMPLETE
        elif self.move_status == MOVE_COMPLETE:
            # -> IDLE
            self.msg_arm.drillstatus = 1
            if self.operation == OPE_PAUSE:
                self.move_status = MOVE_IDLE
        
        # メッセージを発行する
        self.pub_arm.publish(self.msg_arm)
        
#--------------------
def arm_py():
    # 初期化宣言 : このソフトウェアは"arm_py_node"という名前
    rospy.init_node('arm_py_node', anonymous=True)
    # インスタンスの作成 
    arm = Arm()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[arm] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        arm.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        arm_py()

    except rospy.ROSInterruptException:
        print("[arm] end")
