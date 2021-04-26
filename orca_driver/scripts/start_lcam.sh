#!/usr/bin/env bash

# Start left camera, runs on zero-left

# Minimal gstreamer install:
# apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad

# To install:           sudo cp /home/pi/orca3/orca_driver/scripts/orca_lcam.service /lib/systemd/system
# To start:             sudo systemctl start orca_lcam.service
# To stop:              sudo systemctl stop orca_lcam.service
# To start on boot:     sudo systemctl enable orca_lcam.service
# To not start on boot: sudo systemctl disable orca_lcam.service

screen -dmS lcam bash -c "raspivid --nopreview --mode 4 -w 800 -h 600 --framerate 15 --awb auto --brightness 55 --saturation 10 --sharpness 50 --contrast 15  -fl --timeout 0 --output - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.27 port=5601; exec bash"
