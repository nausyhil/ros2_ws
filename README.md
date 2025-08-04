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

```
~/ros2_ws$ lsb_release -a
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 24.04.2 LTS
Release:        24.04
Codename:       noble
```

August 3, 25
Fixed SHM issue with

`rm -rf /tmp/fastrtps_* /dev/shm/fastdds*`

`vi ~/.fastrtps_disable_shm.xml`

```
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
  </profiles>
</dds>
```

`echo $FASTRTPS_DEFAULT_PROFILES_FILE`

`export FASTRTPS_DEFAULT_PROFILES_FILE=~/.fastrtps_disable_shm.xml`

Ref Further Readings on FastDDS documentations:\
https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/transports.html\
https://www.omg.org/spec/DDS/\
https://www.jsoftware.us/vol6/jsw0606-23.pdf\
https://orte.sourceforge.net/rtn08_orte.pdf

BUG?
```
~/ros2_ws$ ros2 node list
WARNING: Be aware that there are nodes in the graph that share an exact name, which can have unintended side effects.
/diff_drive_sim
/diff_drive_sim
/diff_drive_sim
/diff_drive_sim
/diff_drive_sim
/diff_drive_sim
/diff_drive_sim
/rviz
/transform_listener_impl_55fd57757dc0
```

Need to install below so the launch file can start up.
`sudo apt install ros-jazzy-nav2-bringup`

`~/ros2_ws$ ros2 launch my_nav2_launch bringup_launch.py  namespace:=robot1   use_sim_time:=true   autostart:=true  rviz:=true`