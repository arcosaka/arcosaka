#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

from params import MODE, TARGET, CAMERA
from eye_consts import PUB_RATE, EYE2_DISP
from eyes import Eyes


#=======================================================================================================================
# Display Mode(for Debug)
dispMode  = EYE2_DISP                   # display 1:ON 0:OFF


#=======================================================================================================================
# Connecting Camera
# ラズパイ0を接続する場合の処理
#RP0_Url_L = "192.168.253.2:9000/?action=stream"
#RP0_Cap_L = cv2.VideoCapture( RP0_Url_L )
#print( RP0_Cap_L.isOpened() )
#RP0_Url_R = "192.168.253.2:9001/?action=stream"
#RP0_Cap_R = cv2.VideoCapture( RP0_Url_L )
#print( RP0_Cap_L.isOpened() )

#RP3_Url_L  = "http://osakarp.local:8495/?action=stream"
#RP3_Url_R  = "http://osakarp.local:8494/?action=stream"
#RP3_Cap_L  = cv2.VideoCapture( RP3_Url_L )
#print( RP3_Cap_L.isOpened() )
#RP3_Cap_R  = cv2.VideoCapture( RP3_Url_R )
#print( RP3_Cap_R.isOpened() )

RP3_Url_L  = "http://osakarp.local:8497/?action=stream"
RP3_Url_R  = "http://osakarp.local:8496/?action=stream"
RP3_Cap_L  = cv2.VideoCapture( RP3_Url_L )
print( RP3_Cap_L.isOpened() )
RP3_Cap_R  = cv2.VideoCapture( RP3_Url_R )
print( RP3_Cap_R.isOpened() )

#RP3_Url_L  = "http://osakarp.local:8499/?action=stream"
#RP3_Url_R  = "http://osakarp.local:8498/?action=stream"
#RP3_Cap_L  = cv2.VideoCapture( RP3_Url_L )
#print( RP3_Cap_L.isOpened() )
#RP3_Cap_R  = cv2.VideoCapture( RP3_Url_R )
#print( RP3_Cap_R.isOpened() )


# Setting Camera
capWidth         = 320          # /arcosaka/2020/sh/camera.sh Ref.
capHeight        = 240          # /arcosaka/2020/sh/camera.sh Ref.
capL_CenterX     = 174          # Left  Camera CenterX
capR_CenterX     = 144          # Right Camera CenterX
cap_CenterY      = 119          # Both Camera CenterY
cap_CenterOffset = 5            # Both Camera CenterOffset


# Mapping Param
#imgL = np.zeros( (capHeight, capWidth, 3), np.uint8 )
#imgR = np.zeros( (capHeight, capWidth, 3), np.uint8 )
dispar = 32                     # multiple of a unit = 16
sadws  = 21                     # odd within 5...255


#  Side Buds and Weed Position
sidebudsPosX    = 0
sidebudsPosY    = 0
sidebudsPosDist = 0
weedPosX        = 0
weedPosY        = 0
weedPosDist     = 0


#=======================================================================================================================
# Get Disparity Map
def GetDisparityMap():
    imgGrayL = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )
    imgGrayR = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )

    stereo = cv2.StereoBM_create( numDisparities=dispar, blockSize=sadws )
    return stereo.compute( imgGrayL, imgGrayR )


#=======================================================================================================================
# Get Green Detect
def GetGreenDetect( img ):
    # HSV space
    hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

    # thresh1
    hsv_min = np.array( [ 40,  80, 100] )
    hsv_max = np.array( [ 80, 255, 255] )
    mask1   = cv2.inRange( hsv, hsv_min, hsv_max )

    binary_img = mask1

    if dispMode == 1:    
        cv2.imshow( "Green", binary_img )

    unused_image, contours, unused_hierarchy = cv2.findContours( binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE )
    max_index   = 0
    max_contour = 0

    for i in range( 0, len( contours ) ):
        if len( contours[i] ) > 0:
            tmp = cv2.contourArea( contours[i] )

            # remove small objects
            #if tmp < 10:
            #    if max_contour != 0:
            #        continue

            # Max Green Contours
            if tmp > max_contour:
                max_index    = i
                max_contour  = tmp

    return contours[max_index]


#=======================================================================================================================
def eyes_py(id):
    global sidebudsPosX, sidebudsPosY, dispar, sadws

    # 初期化宣言 : このソフトウェアは"eyes_py_node"という名前
    rospy.init_node('eyes_py_node', anonymous=True)

    # インスタンスの作成 
    eyes = Eyes(id)

    # 処理周期の設定
    r = rospy.Rate(PUB_RATE)

    print("[eyes2] start")
    # ctl + Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():

        #==================================================
        # Loop
        ret0, imgL = RP3_Cap_L.read()
        ret1, imgR = RP3_Cap_R.read()

        if dispMode == 1:
            cv2.imshow( "Left  Camera", imgL )
            cv2.imshow( "Right Camera", imgR )

        #-- Create DiparityMap --
        disparity = GetDisparityMap()

        ###################################################
        #-- Look for SideBuds --
        sbsJpg  = cv2.imread( './SideBuds_1.JPG' , 0 )

        grayImg = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)

        result  = cv2.matchTemplate( grayImg, sbsJpg, cv2.TM_CCOEFF_NORMED )
        unused_minVal, unused_maxVal, unused_minLoc, maxLoc = cv2.minMaxLoc( result )
        top_left     = maxLoc
        w, h         = sbsJpg.shape[::-1]
        bottom_right = ( top_left[0] + w, top_left[1] + h )

        if dispMode == 1:
            cv2.rectangle( imgL, top_left, bottom_right, (0, 255, 0), 1 )
            cv2.imshow( "SideBuds", imgL )

        sidebudsPosX = top_left[0] + (w / 2)
        sidebudsPosY = top_left[1] + (h / 2)

        #-- Cal SideBuds Distance --
        sidebudsPosDist = 6.6 * 62 / 0.003 / disparity[sidebudsPosY][sidebudsPosX]
        print sidebudsPosX, sidebudsPosY, sidebudsPosDist

        ###################################################
        #-- Cal weed Center --
        targetL = GetGreenDetect( imgL )
        targetR = GetGreenDetect( imgR )

        # target Position
        Lx, Ly, Lw, Lh = cv2.boundingRect( targetL )
        Rx, Ry, Rw, Rh = cv2.boundingRect( targetR )

        if dispMode == 1:
            cv2.rectangle( imgL, (Lx, Ly), (Lx + Lw, Ly + Lh), (0, 0, 255), 1 )
            cv2.imshow( "Weed L", imgL )
            cv2.rectangle( imgR, (Rx, Ry), (Rx + Rw, Ry + Rh), (0, 0, 255), 1 )
            cv2.imshow( "Weed R", imgR )

        # Xは左右中心の更に中心
        Lpos = Lx + (Lw / 2)
        Rpos = Rx + (Rw / 2)
        weedPosX = (Lpos + Rpos) / 2
        # Yは片側基準でOK
        weedPosY = Ly + (Lh / 2)

        # from disparity to dist
        # Z = f x T / um / disparity
        weedPosDist = 6.6 * 62 / 0.003 / disparity[weedPosY][weedPosX]
        print weedPosX, weedPosY, weedPosDist

        #==================================================
        # メイン処理
        eyes.main()

        # Sleep処理
        r.sleep()

#=======================================================================================================================
if __name__ == '__main__':
    try:
        eyes_py(CAMERA.SUB)

    except rospy.ROSInterruptException:
        # Stop
        capL.release()
        capR.release()
        cv2.destroyAllWindows()
        print("[eyes2] end")
