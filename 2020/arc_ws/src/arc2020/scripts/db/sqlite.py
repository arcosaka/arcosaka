#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

import sqlite3

class sqlite(object):
    def __init__(self,name_db):
        path = "/root/catkin_ws/src/arc2020/"
        print("db create start")
        # 'dbname'.dbを作成する
        # すでに存在していれば、それにアスセスする。
        self.db = path + name_db + '.db'
        conn = sqlite3.connect(self.db)
        print("db create done")
        # いったん閉じる
        conn.close()

    def open(self):
        # sqliteを操作するカーソルオブジェクトを作成
        self.conn = sqlite3.connect(self.db)
        self.cursor = self.conn.cursor()

    def close(self):
        # データベースへのコネクションを閉じる。(必須)
        self.cursor.close()
        self.conn.close()

    def createTable(self,name_table):
        # db open
        self.open()

        # 'name_table'というtableがなければ作成する
        # 大文字部はSQL文。小文字でも問題ない。
        command = 'CREATE TABLE IF NOT EXISTS '+ name_table + ' ( id INTEGER PRIMARY KEY,name VARCHAR(20),price INTEGER);'
        self.cursor.execute( command )
        self.conn.commit()
        
        # db close
        self.close()

    def insert(self, name_table, name_seed):
        # create table
        self.createTable(name_table)
        # db open
        self.open()
        # 'name_table'の"name"に"name_seed"を入れる
        command = 'INSERT INTO '+ name_table +'(name)' + ' values("'+name_seed+'")'
        self.cursor.execute(command)
        #登録
        self.conn.commit()
    
        # db close
        self.close()

    def selectTable(self,name_table):
        # db open
        self.open()

        # terminalで実行したSQL文と同じようにexecute()に書く
        command = 'SELECT * FROM ' + name_table 
        self.cursor.execute( command )

        # 中身を全て取得するfetchall()を使って、printする。
        print(self.cursor.fetchall())

        # db close
        self.close()

    def selectyasai1(self):
        # db open
        self.open()

        #print 'printしますよ！！'
        # terminalで実行したSQL文と同じようにexecute()に書く
        command = 'SELECT 野菜1 FROM yasai group by 野菜1 having count(野菜1)>1' 
        self.cursor.execute( command )

        # 中身を全て取得するfetchall()を使って、printする。
        moji = self.cursor.fetchall()

        #for rows in moji: 
        #  print rows[0]

        # db close
        # self.close()
        return moji

    def selectyasai2(self,vegetable_num1):
        self.open()

        #print 'printしますよ！！'
        # terminalで実行したSQL文と同じようにexecute()に書く
        command = 'SELECT 野菜2 FROM yasai WHERE 野菜1 = \'' + vegetable_num1 + '\'group by 野菜2'
        self.cursor.execute( command )

        # 中身を全て取得するfetchall()を使って、printする。
        moji = self.cursor.fetchall()

        return moji

    def selectyasaihaichi(self,vegetable_num1,vegetable_num2,ridge_width,ridge_length):
        self.open()

        #print(ridge_width)
        #print(ridge_length)

        command = 'SELECT 配置 FROM yasai WHERE 野菜1 = \'' + vegetable_num1 + '\' AND 野菜2 = \'' + vegetable_num2 + '\' AND 幅 = ' + str(ridge_width) + ' AND 長さ = ' + str(ridge_length)
#        print(command)
        self.cursor.execute( command )
        # 中身を全て取得するfetchall()を使って、printする。
        haichi = self.cursor.fetchall()
#        print (haichi)

        command = 'SELECT 野菜1間隔縦 FROM yasai WHERE 野菜1 = \'' + vegetable_num1 + '\' AND 野菜2 = \'' + vegetable_num2 + '\' AND 幅 = ' + str(ridge_width) + ' AND 長さ = ' + str(ridge_length)
        self.cursor.execute( command )
        # 中身を全て取得するfetchall()を使って、printする。
        yasai1 = self.cursor.fetchall()
#        print (yasai1)

        command = 'SELECT 野菜2間隔縦 FROM yasai WHERE 野菜1 = \'' + vegetable_num1 + '\' AND 野菜2 = \'' + vegetable_num2 + '\' AND 幅 = ' + str(ridge_width) + ' AND 長さ = ' + str(ridge_length)
        self.cursor.execute( command )
        # 中身を全て取得するfetchall()を使って、printする。
        yasai2 = self.cursor.fetchall()
#        print (yasai2)

        command = 'SELECT 野菜1の数 FROM yasai WHERE 野菜1 = \'' + vegetable_num1 + '\' AND 野菜2 = \'' + vegetable_num2 + '\' AND 幅 = ' + str(ridge_width) + ' AND 長さ = ' + str(ridge_length)
        self.cursor.execute( command )
        # 中身を全て取得するfetchall()を使って、printする。
        yasai1s = self.cursor.fetchall()
#        print (yasai1s)

        command = 'SELECT 野菜2の数 FROM yasai WHERE 野菜1 = \'' + vegetable_num1 + '\' AND 野菜2 = \'' + vegetable_num2 + '\' AND 幅 = ' + str(ridge_width) + ' AND 長さ = ' + str(ridge_length)
        self.cursor.execute( command )
        # 中身を全て取得するfetchall()を使って、printする。
        yasai2s = self.cursor.fetchall()
#        print (yasai2s)

        return haichi, yasai1, yasai2, yasai1s, yasai2s

