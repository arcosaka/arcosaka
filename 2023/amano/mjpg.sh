#!/bin/bash

pkill mjpg_streamer

echo "start MJPG-streamer"

WIDTH=640
HEIGHT=480
RES="${WIDTH}x${HEIGHT}"
FPS=30
QUALITY=10
PORT=49200
WWW_ROOT="/usr/local/share/mjpg-streamer/www"

cameras=""
cmdargs=""
videos=(`ls -1v /dev/video*`)
for video_file in "${videos[@]}"; do
    video_name=${video_file#/dev/}
    video_num=${video_file#/dev/video}
    
    CAMEXIST=`v4l2-ctl -d ${video_file}  --list-inputs | grep -c Name`
    #echo "${video_file}:${CAMEXIST}"
    # 指定されたデバイスが録画できるものの場合
    if [ ${CAMEXIST} != "0" ]; then
        if [ "${cameras}" != "" ]; then
            cameras="${cameras},"
        fi
        cameras="${cameras}${video_num}"
        cmdargs="${cmdargs} -i 'input_uvc.so -d ${video_file} -r ${RES} -f ${FPS}'"
    fi
done
cmdargs="${cmdargs} -o 'output_http.so -w $WWW_ROOT -p $PORT'"
#echo ${cmdargs}

cmdline="mjpg_streamer ${cmdargs} &>> /var/log/stream/video.log &"
echo "----" >> /var/log/stream/video.log
eval ${cmdline}

echo "http://${HOSTNAME}.local:49199/preview.html?camnums=${cameras}"

#python ~/create_html.py

python -m http.server 49199 --directory .//www/ &>> /var/log/stream/pyhttp.log &
