#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio
import time

# 自分で定義したmessageファイルから生成されたモジュール
#受信
from arc2020.msg import webapp
from arc2020.msg import arm
from arc2020.msg import foot
from arc2020.msg import database
from arc2020.msg import emgstp
from arc2020.msg import volcurmeas

#送信
from arc2020.msg import main


# 定数などの定義ファイルimport
CYCLES = 60

USERNAME = "aaa"
PASSWORD = "bbb"
mainlist = ["ニンジン","トマト","レタス"]
sublist = ["バジル","春菊","ほうれん草"]
type = [0,0,1,0,0]
y = [5,5,10,15,15]
x = [10,20,15,10,20]
COUNT = len(x)

class Main(object):
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
        # webapp ⇒ main
        self.sub_webapp  = rospy.Subscriber('webapp', webapp, self.webappCallback, queue_size=1)
        
        # arm ⇒ main
        self.sub_arm  = rospy.Subscriber('arm', arm, self.armCallback, queue_size=1)
        
        # foot ⇒ main
        self.sub_foot  = rospy.Subscriber('foot', foot, self.footCallback, queue_size=1)
        
        # database ⇒ main
        self.sub_database  = rospy.Subscriber('database', database, self.databaseCallback, queue_size=1)

        # emgstp ⇒ main
        self.sub_emgstp  = rospy.Subscriber('emgstp', emgstp, self.emgstpCallback, queue_size=1)

        # volcurmeas ⇒ main
        self.sub_volcurmeas  = rospy.Subscriber('volcurmeas', volcurmeas, self.volcurmeasCallback, queue_size=1)


        # 送信作成
        # main ⇒ 各モジュール
        self.pub_main = rospy.Publisher('main', main, queue_size=100)
        self.msg_main = main()

        # 送信メッセージ・受信バッファ初期化
        self.clearMsg()
        
        self.footcount = 0
        self.armcount = 0
        self.status = 0
        self.movestatus_o = 1
        self.drillstatus_o = 1
        
#--------------------
# 送信メッセージ・受信バッファ初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        # main ⇒ foot
        self.msg_main.foot_operation = 1
        for i in range(COUNT):
            self.msg_main.foot_x_cordinate.append(y[i])
        self.msg_main.foot_mode = 0
        self.msg_main.foot_motor_l = 0
        self.msg_main.foot_motor_r = 0
        
        # main ⇒ arm
        self.msg_main.arm_retorg_req = 0
        self.msg_main.arm_drill_req = 2
        for i in range(COUNT):
            self.msg_main.arm_drill_width.append(x[i])
        self.msg_main.arm_ridge_width = 0
        self.msg_main.arm_ridge_height = 0
        
        # main ⇒ database
        self.msg_main.database_inforeq = 0
        self.msg_main.database_vegetable_num1 = 0
        self.msg_main.database_vegetable_num2 = 0
        self.msg_main.database_ridge_width = 0
        self.msg_main.database_ridge_length = 0
        
        # main ⇒ webapp
        self.msg_main.webapp_loginpermit = 0
        for i in range(3):
            self.msg_main.webapp_mainseedlist.append(mainlist[i])
        for i in range(3):
            self.msg_main.webapp_subseedlist.append(sublist[i])
        for i in range(COUNT):
            self.msg_main.webapp_seed_type.append(type[i])
        for i in range(COUNT):
            self.msg_main.webapp_coordinates_x.append(x[i])
        for i in range(COUNT):
            self.msg_main.webapp_coordinates_y.append(y[i])
        self.msg_main.webapp_batterycharge = 100
        self.msg_main.webapp_mileage = 0
        self.msg_main.webapp_completion = 0
        self.msg_main.webapp_operatingstate = 0
        
        
        # webapp ⇒ main
        self.username = 0
        self.password = 0
        self.inforeq = 0
        self.mainseed = 0
        self.subseed = 0
        self.ridge_length = 0
        self.ridge_width = 0
        self.ridge_height = 0
        self.blueprintreq = 0
        self.movestartreq = 0
        self.movestopreq = 0
        self.continueselect = 0
        
        # arm ⇒ main
        self.drillstatus = 1
        self.retorgstatus = 0
        
        # foot ⇒ main
        self.movestatus = 1
        
        # database ⇒ main
        #for i in range(3):
        #    self.mainseedlist[i] = mainlist[i]
        #for i in range(3):
        #    self.subseedlist[i] = sublist[i]
        #for i in range(COUNT):
        #    self.seed_type[i] = type[i]
        #for i in range(COUNT):
        #    self.coordinates_x[i] = x[i]
        #for i in range(COUNT):
        #    self.coordinates_y[i] = y[i]
        
        # emgstp ⇒ main
        self.emgstopstate = 0
        
        # volcurmeas ⇒ main
        self.batterycharge = 0


#--------------------
# 受信コールバック
    def webappCallback(self, webapp_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ取得
        self.username = webapp_msg.username
        self.password = webapp_msg.password
        self.inforeq = webapp_msg.inforeq
        self.mainseed = webapp_msg.mainseed
        self.subseed = webapp_msg.subseed
        self.ridge_length = webapp_msg.ridge_length
        self.ridge_width = webapp_msg.ridge_width
        self.ridge_height = webapp_msg.ridge_height
        self.blueprintreq = webapp_msg.blueprintreq
        self.movestartreq = webapp_msg.movestartreq
        self.movestopreq = webapp_msg.movestopreq
        self.continueselect = webapp_msg.continueselect


#--------------------
# 受信コールバック
    def armCallback(self, arm_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ取得
        self.drillstatus = arm_msg.drillstatus
        self.retorgstatus = arm_msg.retorgstatus


#--------------------
# 受信コールバック
    def footCallback(self, foot_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ取得
        self.movestatus = foot_msg.movestatus
        self.msg_main.webapp_mileage = foot_msg.mileage_val


#--------------------
# 受信コールバック
    def databaseCallback(self, database_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ取得
        #self.mainseedlist = database_msg.mainseedlist
        #self.subseedlist = database_msg.subseedlist
        #self.seed_type = database_msg.seed_type
        #self.coordinates_x = database_msg.coordinates_x
        #self.coordinates_y = database_msg.coordinates_y
        pass

#--------------------
# 受信コールバック
    def emgstpCallback(self, emgstp_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ取得
        self.emgstopstate = emgstp_msg.emgstopstate


#--------------------
# 受信コールバック
    def volcurmeasCallback(self, volcurmeas_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ取得
        self.batterycharge = volcurmeas_msg.batterycharge
        
        # バッテリ残量
        #self.msg_main.webapp_batterycharge = self.batterycharge


#--------------------
# メイン関数
    def main(self):
        # パスワード認証
        if self.username == USERNAME and self.password == PASSWORD :
            self.msg_main.webapp_loginpermit = 1
        else :
            self.msg_main.webapp_loginpermit = 0
        
        # メイン・サブ品種・畝情報設定
        # 品種・畝情報取得要求
        if self.inforeq == 1 :
            self.msg_main.database_inforeq = 1
            
            # メイン候補更新
            for i in range(3):
                self.msg_main.webapp_mainseedlist[i] = mainlist[i]
            """
            # メイン品種
            for i in range(30):
                if self.mainseed == self.mainseedlist[i] :
                    self.msg_main.database_vegetable_num1 = i
            """
            # サブ候補更新
            for i in range(3):
                self.msg_main.webapp_subseedlist[i] = sublist[i]
            """
            # サブ品種
            for i in range(30):
                if self.subseed == self.subseedlist[i] :
                    self.msg_main.database_vegetable_num2 = i
            """
            # 畝の幅・長さ・高さ
            self.msg_main.database_ridge_width = self.ridge_width
            self.msg_main.database_ridge_length = self.ridge_length
            self.msg_main.arm_ridge_width = self.ridge_width
            self.msg_main.arm_ridge_height = self.ridge_height
            
        # 穴あけ設計図要求
        if self.blueprintreq == 1 :
            for i in range(COUNT):
                self.msg_main.webapp_seed_type[i] = type[i]
            for i in range(COUNT):
                self.msg_main.webapp_coordinates_y[i] = y[i]
                self.msg_main.foot_x_cordinate[i] = y[i]
            for i in range(COUNT):
                self.msg_main.webapp_coordinates_x[i] = x[i]
                self.msg_main.arm_drill_width[i] = x[i]
        
        # 穴あけ実施
        if self.footcount == COUNT and self.armcount == COUNT :
            self.status = 1
            self.msg_main.foot_operation = 2
            self.msg_main.arm_drill_req = 0
        if self.movestartreq == 1 and self.status == 0 :
            # foot移動指示
            if self.drillstatus == 1 and self.footcount == self.armcount :
                self.msg_main.foot_operation = 0
                self.msg_main.arm_drill_req = 2
                if self.movestatus == 1 and self.movestatus_o == 0 :
                    self.footcount += 1
                    self.msg_main.foot_operation = 1
                self.movestatus_o = self.movestatus
            # arm移動指示
            if self.movestatus == 1 and self.armcount < self.footcount :
                self.msg_main.arm_drill_req = 1
                self.msg_main.foot_operation = 1
                if self.drillstatus == 1 and self.drillstatus_o == 0 :
                    self.armcount += 1
                    self.msg_main.arm_drill_req = 2
                    self.msg_main.webapp_completion = self.armcount
                self.drillstatus_o = self.drillstatus
        #print("self.movestatus:"+str(self.movestatus)+" self.footcount:"+str(self.footcount)+" self.drillstatus:"+str(self.drillstatus)+" self.armcount:"+str(self.armcount))
        # メッセージを発行する
        self.pub_main.publish(self.msg_main)


#--------------------
def main_py():
    # 初期化宣言 : このソフトウェアは"main_py_node"という名前
    rospy.init_node('main_py_node', anonymous=True)
    # インスタンスの作成 
    main = Main()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[main] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        main.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        main_py()

    except rospy.ROSInterruptException:
        print("[main] end")
