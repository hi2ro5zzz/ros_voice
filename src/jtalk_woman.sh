#!/bin/sh
TMP=/tmp/ojtalk_tmp.wav
echo "$1" | open_jtalk -x /var/lib/mecab/dic/open-jtalk/naist-jdic -m ~/catkin_ws/src/ros_voice/src/Voice_mei/mei_normal.htsvoice -ow $TMP
aplay --quiet $TMP
rm -f $TMP