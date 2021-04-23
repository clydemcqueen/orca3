#!/usr/bin/env bash

# Start orca ROS driver

# To install:           sudo cp ~/ros2/orca3_ws/src/orca3/orca_driver/scripts/orca_driver.service /lib/systemd/system
#                       cp ~/ros2/orca3_ws/src/orca3/orca_driver/scripts/setup.bash ~/ros2/orca3_ws
# To start:             sudo systemctl start orca_driver.service
# To stop:              sudo systemctl stop orca_driver.service
# To start on boot:     sudo systemctl enable orca_driver.service
# To not start on boot: sudo systemctl disable orca_driver.service

screen -dmS orca_driver bash -c "cd ~/ros2/orca3_ws; . install/setup.bash; ros2 launch orca_bringup rov_sub_launch.py; exec bash"
