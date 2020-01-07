#!/bin/sh
TMP=/tmp/ojtalk_tmp.wav
echo "$1" | open_jtalk -x /var/lib/mecab/dic/open-jtalk/naist-jdic -m /usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice -ow $TMP
aplay --quiet $TMP
rm -f $TMP