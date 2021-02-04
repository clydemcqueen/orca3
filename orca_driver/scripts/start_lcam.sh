#!/usr/bin/env bash

screen -dmS lcam bash -c "raspivid --nopreview --mode 4 -w 800 -h 600 --framerate 15 --awb auto --brightness 55 --saturation 10 --sharpness 50 --contrast 15  -fl --timeout 0 --output - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.105 port=5601; exec bash"
