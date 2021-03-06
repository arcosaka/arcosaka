#!/bin/bash
if [ $# -ne 1 ]; then
    echo "起動したいプログラムを指定してください" 1>&2
    exit 1
fi
export ROS_PRGNAME="$1"
echo "ROS_PRGNAME=「${ROS_PRGNAME}」に設定"
if ! pgrep roslaunch > /dev/null; then
    source /opt/ros/melodic/setup.bash
    source ~/arcosaka/2019/${ROS_PRGNAME}/devel/setup.bash
    export ROS_SRCDIR="~/arcosaka/2019/${ROS_PRGNAME}/src"
    export ROS_HOSTNAME=osakarp.local
    export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
    alias "cw=cd ~/arcosaka/2019/${ROS_PRGNAME}/"
    alias "cs=cd ~/arcosaka/2019/${ROS_PRGNAME}/src/"
    alias "cm=cd ~/arcosaka/2019/${ROS_PRGNAME}/ && catkin_make"

    cs
    /opt/ros/melodic/bin/roslaunch ${ROS_PRGNAME} ${ROS_PRGNAME}.launch
else
    echo "roslaunch killing"
    killall -w roslaunch
fi
