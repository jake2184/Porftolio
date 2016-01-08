#!/bin/bash

width=$1;
height=$2;

ffplay -probesize 32 -video_size ${1}x${2} -pix_fmt rgb24 -vcodec rawvideo -f rawvideo  -i -
