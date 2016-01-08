#!/bin/bash

QUALITY=0 # Best quality, big files
if [[ "$2" ]]; then
    QUALITY=$2
fi

ffmpeg -vsync 0 -i $1 -f mjpeg -qscale:video ${QUALITY} -
