#!/usr/bin/env bash

# To install:           sudo cp ~/ros2/orca3_ws/src/orca3/orca_driver/scripts/orca_fcam.service /lib/systemd/system
# To start:             sudo systemctl start orca_fcam.service
# To stop:              sudo systemctl stop orca_fcam.service
# To start on boot:     sudo systemctl enable orca_fcam.service
# To not start on boot: sudo systemctl disable orca_fcam.service

screen -dmS orca_fcam bash -c "gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=10.0.0.1 port=5600; exec bash"

# gstreamer notes:
#
# v4l2src pulls data from the camera
#   pulled from video2, which is h264 encoded on the BlueRobotics USB camera
#   do-timestamp=true means that v4l2src provides a timestamp
#
# queue provides a producer/consumer buffer, and makes sure that the producer (v4l2src)
# and consumer (capsfilter and downstream) are on different threads
#
# There’s an implied “capsfilter caps=” in the next step.
# This set of capabilities specifies width, height and framerate, the other caps are negotiated.
#
# h264parse parses the h264 stream and provides additional caps downstream. In this case it provides these caps:
#   stream-format=(string)byte-stream, alignment=(string)au, pixel-aspect-ratio=(fraction)1/1, colorimetry=(string)2:4:7:1, interlace-mode=(string)progressive
#
# rtph264pay encodes h264 into RTP packets
#   config-interval=10 sends SPS and PPS packets every 10s, use -1 to send every IDR frame (whatever that is).
#   10, 1 and -1 all seem to work fine. I picked 10s for now.
#
# With these settings gst-launch is consuming ~13% CPU on the UP board and sending data at 1.4Mbps
