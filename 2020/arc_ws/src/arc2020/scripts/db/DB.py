#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

import sqlite
import copy
# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import sql
from arc2020.msg import database
from arc2020.msg import maind

CYCLES = 1 #処理周波数
# 定数などの定義ファイルimport


# class DB 定義
class DB(object):
    """
    DBクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        self.cyclecount = 0
        # 受信作成
        self.sub_client  = rospy.Subscriber('sql', sql, self.insertCallback, queue_size=1)
        self.sub_main  = rospy.Subscriber('maind', maind, self.mainCallback, queue_size=1)
        # 送信作成
        self.pub_database = rospy.Publisher('database', database, queue_size=100)
        # messageのインスタンスを作る
        self.msg_sql = sql()
        self.msg_database = database()
        self.msg_main = maind()
        # dbを作成する
        db_name = 'yasai'
        self.db = sqlite.sqlite(db_name) 
        print("db create by DB")

        ##self.db.selectTable("yasai")
        self.db.selectyasai1()
        # tableを作成する
        # self.table_seed = 'seed'
        # self.sqlite.createTable(self.table_seed)

#--------------------
# 受信コールバック
    def insertCallback(self, msg):
        """
        brainの受信コールバック
        """
        print("table " + msg.table + ",name " + msg.name)

        # msg.tableにmsg.nameを登録する       
        self.db.insert(msg.table,msg.name)

    def mainCallback(self, msg):
        """
        mainの受信コールバック
        """
        print(msg.database_inforeq)
        print(msg.database_vegetable_num1)
        print(msg.database_vegetable_num2)
        print(msg.database_ridge_width)
        print(msg.database_ridge_length)

        if msg.database_inforeq:
           aaa = self.db.selectyasai1()
           self.msg_database.mainseedlist = copy.copy(aaa)

        bbb = ','.join(msg.database_vegetable_num1)
#        print (bbb)
        self.msg_database.subseedlist = self.db.selectyasai2(bbb)
        ccc = ','.join(msg.database_vegetable_num2)
#        print (ccc)
        haichi, yasai1, yasai2, yasai1s, yasai2s = self.db.selectyasaihaichi(bbb,ccc,msg.database_ridge_width,msg.database_ridge_length)
#        print ('printしますよ')

        haba = msg.database_ridge_width
        nagasa = msg.database_ridge_length

        haichi_0  = int(haichi[0][0]) 
        yasai1_ktate  = int(yasai1[0][0]) 
        yasai2_ktate  = int(yasai2[0][0]) 
        yasai1_kazu = int(yasai1s[0][0]) 
        yasai2_kazu = int(yasai2s[0][0]) 
        print (haichi_0)
        print (yasai1_ktate)
        print (yasai2_ktate)
        print (yasai1_kazu)
        print (yasai2_kazu)

        ynagasa = 0
        self.msg_database.coordinates_x = []
        self.msg_database.coordinates_y = []
        self.msg_database.seed_type = []

        yasai_sum = yasai1_kazu + yasai2_kazu

        for num in range(yasai_sum):
            if haichi_0 == 1:
                self.msg_database.coordinates_x.append(haba / 2)

        for num in range(yasai_sum):
            ynagasa += yasai1_ktate
            self.msg_database.coordinates_y.append(ynagasa)

        for num in range(yasai_sum):
            if num % 2 == 0:
                seedtype = 1
            else:
                seedtype = 0
            self.msg_database.seed_type.append(seedtype)

        print('xとyとseed_typeを表します。')

        for x in self.msg_database.coordinates_x:
            print(x)

        for y in self.msg_database.coordinates_y:
            print(y)

        for stype in self.msg_database.seed_type:
            print(stype)


#--------------------
    def main(self):
        # メッセージを発行する
        self.pub_database.publish(self.msg_database)
        print('mainが動いていますよ！！')

def DB_py():
    # 初期化宣言 : このソフトウェアは"DB_py_node"という名前
    rospy.init_node('DB_py_node', anonymous=True)
    # インスタンスの作成 
    _DB = DB()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[DB] start")


    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        _DB.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        DB_py()

    except rospy.ROSInterruptException:
        print("[DB] end")
