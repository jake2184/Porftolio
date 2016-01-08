#!/bin/bash

# On windows, to enumerate camera sources, do:
#     ffmpeg -list_devices true -f dshow -i dummy

CAMERA="HP HD Webcam";

ffmpeg -f dshow -framerate 10 -i video="${CAMERA}" -f image2pipe -vcodec mjpeg -
