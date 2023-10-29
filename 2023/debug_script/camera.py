# -*- coding: utf-8 -*-
import cv2

#-------------------------------
# Webカメラ
#-------------------------------
WIDTH = 300
HEIGHT = 250
#ID:0  FPS:5,7.5,10,15  //20
#ID:2  FPS:5    ,10,15      //20
FPS = 5

#-------------------------------
#
#-------------------------------
def cam_set(DEVICE_ID,WIDTH,HEIGHT,FPS):
    # video capture
    cap = cv2.VideoCapture(DEVICE_ID)

    # フォーマット・解像度・FPSの設定
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    # フォーマット・解像度・FPSの取得
    fourcc = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("ID:{} fourcc:{} fps:{}　width:{}　height:{}".format(DEVICE_ID, fourcc, fps, width, height))

    return cap

#-------------------------------
#
#-------------------------------
def decode_fourcc(v):
        v = int(v)
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

#-------------------------------
#
#-------------------------------
def main():
    #--------------------------------------
    DEVICE_ID = 0
    cap1=cam_set(DEVICE_ID,WIDTH,HEIGHT,FPS)

    #--------------------------------------
    DEVICE_ID = 2
    cap2=cam_set(DEVICE_ID,WIDTH,HEIGHT,FPS)
    
    #--------------------------------------
    DEVICE_ID = 4
    cap3=cam_set(DEVICE_ID,WIDTH,HEIGHT,FPS)

    #--------------------------------------
    while True:
        #-----------------------
        # カメラ画像取得
        ret1, frame1 = cap1.read()
        if(frame1 is None):
            continue

        #-----------------------
        # カメラ画像取得
        ret2, frame2 = cap2.read()
        if(frame2 is None):
            continue

        #-----------------------
        # カメラ画像取得
        ret3, frame3 = cap3.read()
        if(frame3 is None):
            continue

        #---------------------------
        im_h=cv2.hconcat([frame1,frame2,frame3])

        #---------------------------
        # 画像表示
        cv2.imshow('frame1', im_h)

        #--------------------------------------
        # キュー入力判定(1msの待機)
        # waitKeyがないと、imshow()は表示できない
        # 'q'をタイプされたらループから抜ける
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    #------------------------------
    # VideoCaptureオブジェクト破棄
    cap1.release()
    cap2.release()
    cap3.release()
    cv2.destroyAllWindows()

#-------------------------------
if __name__ == '__main__':
    main()
