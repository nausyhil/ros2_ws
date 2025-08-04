`rm -rf /tmp/fastrtps_* /dev/shm/fastdds*`

`vi ~/.fastrtps_disable_shm.xml`

`echo $FASTRTPS_DEFAULT_PROFILES_FILE`

`export FASTRTPS_DEFAULT_PROFILES_FILE=~/.fastrtps_disable_shm.xml`

BUILD

Terminal-1:

`~/ros2_ws$ colcon build --symlink-install`

`~/ros2_ws$ source install/setup.bash`

`~/ros2_ws$ ros2 launch my_nav2_launch bringup_launch.py  namespace:=robot1   use_sim_time:=true   autostart:=true   log_level:=debug > log.txt`

Terminal-2

`~/ros2_ws$ source install/setup.bash`

`~/ros2_ws$ ros2 run rviz2 rviz2`

USEFUL REFERENCES
https://github.com/maponarooo/ROS2-Navigation2-Tutorial

FYI Ubuntu Version

`~/ros2_ws$ lsb_release -a
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 24.04.2 LTS
Release:        24.04
Codename:       noble`