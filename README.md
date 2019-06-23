# AMiRo Bridges

This ROS package contains conversions (aka brdiges) from RSB (Robotics Service Bus) to ROS (Robot Operating System) messages and vice versa.
E.g. the robot's outgoing RSB message, like odometry or laserscans, are converted to ROS messages.
Vice verse, ROS based control messages from turtlebot_teleop are converted to RSB messages and send to the robot.


## How To

Make sure that you sourced the setup.bash like 'source devel/setup.bash'. Otherwise ros doesn't know the environment.
There are several parameter you have to adapt to your scenario in the 'start.launch'.

### Parameter start.launch

|         Name         | Default |                                                         Description                                                          |
| -------------------- | ------- | ---------------------------------------------------------------------------------------------------------------------------- |
| rostimenow           | false   | If this is set to true ros::time::now will be used as timestmap otherwise the timestmap will be converted from the rsb data. |
| xboxController       | 0       | To controll the AMiRo with an xboxController                                                                                 |
| laserscaner          | 0       | If you want to use a laserscaner. With laserscan_sick and laserscan_hokuyo you can controll the which laserscan is used.     |
| laserscan_sick       | 0       | Decide to use the sick tim laserscaner.                                                                                      |
| laserscan_hokuyo     | 0       | Decide to use the hokuyo laserscanner.                                                                                       |
| twb                  | 0       | Use the TeleWorkBench data .                                                                                                 |
| robot_localization   | 0       | Use the robot_localization packge from ros which is a kalman-filter. It combines the AMiRo-odometry and the twb-data.        |
| dynamic_tf_with_odom | 0       | This is a dynamic transformer to update the tf from amiro_odom and amiro_base_link with AMiRo odometry data                  |
| dynamic_tf_with_ekf  | 0       | the same as dynamic-tf_with_odom but with the ekf data.                                                                      |
| static_tf_map_odom   | 0       | defines a static transform between map and amiro_odom                                                                        |

As default there are only converter for one AMiRo. If you want to add more AMiRo you have to edit the 'start.launch' and copy a block of the 'amiro.launch' as following and add it under:
```
<include file="$(env MUROX_ROS)/amiro.launch">
  <arg name="amiroNr" value="1"/>
  <arg name="markerId" value="1"/>

  <arg name="laserscan" value="$(arg laserscan)"/>
  <arg name="laserscan_sick" value="$(arg laserscan_sick)"/>
  <arg name="laserscan_hokuyo" value="$(arg laserscan_hokuyo)"/>
  <arg name="camera" value="0"/>

  <arg name="twb" value="$(arg twb)"/>
  <arg name="robot_localization" value="$(arg robot_localization)"/>

  <arg name="rostimenow" value="$(arg rostimenow)"/>

  <arg name="dynamic_tf_with_odom" value="$(arg dynamic_tf_with_odom)"/>
  <arg name="dynamic_tf_with_ekf" value="$(arg dynamic_tf_with_ekf)"/>
  <arg name="static_tf_map_odom" value="$(arg static_tf_map_odom)"/>

  <arg name="xbox_controller" value="$(arg xboxController)"/>
  <arg name="keyboard_controller" value="0"/>

  <arg name="nav_stack" value="0"/>
  <arg name="no_static_map" value="0"/>

</include>
```

### Parameter amiro.launch

|         Name         | Default |                                                   Description                                                   |
| -------------------- | ------- | --------------------------------------------------------------------------------------------------------------- |
| amiroNr              | 1       | Set the amiroId as namespace for all bridges.                                                                   |
| markerId             | 1       | Set the markerId from the twb data.                                                                             |
| laserscan            | 0       | Toggle the laserscaner.                                                                                         |
| laserscan_sick       | 0       | Toggle the sick tim laserscanner.                                                                               |
| laserscan_hokuyo     | 0       | Toggle the hokuyo laserscanner.                                                                                 |
| camera               | 0       | The the vision bridge for the inbuild AMiRo camera.                                                             |
| twb                  | 0       | Toggle the bridge for the twb data.                                                                             |
| robot_localization   | 0       | Toggle the extended kalman filter for AMiRo odometry and twb data.                                              |
| rostimenow           | 0       | Toggle the header stimestamps of each message with rostimenow or the normal rsb timestamps.                     |
| dynamic_tf_with_odom | 0       | This is a dynamic transformer to update the tf from amiro_odom and amiro_base_link with AMiRo odometry data     |
| dynamic_tf_with_ekf  | 0       | the same as dynamic-tf_with_odom but with the ekf data.                                                         |
| static_tf_map_odom   | 0       | defines a static transform between map and amiro_odom                                                           |
| xbox_controller      | 0       | Toggle the bridges to control the AMiRo with an xbox controller.                                                |
| keyboard_controller  | 0       | Toggle the bridges to control the AMiRo with a keyboard.                                                        |
| nav_stack            | 0       | Toggle the ros navigation stack for the AMiRo.                                                                  |
| no_static_map        | 0       | If there is a dynamic map while SLAM'ing this has to be set 0 otherwise if there is a static map set this to 1. |

## Some useful commands

## Preconditions on AMiRo

* Working network connection
* correct `rsb.conf` configuration

AMiRo as host with IP `192.168.3.1`:
```
[transport.spread]
enabled = 0

[transport.inprocess]
enabled = 0

[transport.socket]
enabled = 1
server = 1
port = 55555
host = 192.168.3.1
```

PC as client with IP `192.168.3.2`:
```
[transport.spread]
enabled = 0

[transport.inprocess]
enabled = 0

[transport.socket]
enabled = 1
server = 0
port = 55555
host = 192.168.3.1
```

### Running executables on AMiRo that converst RSB to CAN messages

```
# Set lights
./setLights -c /amiro/lights
# Drive to pose (do not use with actAmiroMotor)
./actTargetPosition -i /amiro/pose
# Drive directly (do not use with actTargetPosition)
./actAmiroMotor -i /amiro/motor
```

### Running bridges on PC

```
# Bridge light messages
rosrun ros_to_rsb_bridge ros_int_multiarray_rst_value_array _ros_listener_topic:=/amiro/lights _rsb_publish_scope:=/amiro/lights
# Bridge pose messages
rosrun ros_to_rsb_bridge ros_geometry_msgs_twist_to_rst_value_array _ros_listener_topic:=/amiro/motor _rsb_publish_scope:=/amiro/motor
# Bridge twist messages
rosrun ros_to_rsb_bridge ros_geometry_msgs_posestamped_to_rst_geometry_pose _ros_listener_topic:=/amiro/pose _rsb_publish_scope:=/amiro/pose
```

### Example commands on PC

* Lightning (Every data looks like `[mode, R, G, B, period]` or `[mode, R_1, G_1, B_1, ..., R_8, G_8, B_8, period]`
  * Initial coloring (nor RGB necessary): `rostopic pub -r 1 /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [0,0]}}'`
  * Specific coloring (Period does not care): `rostopic pub -r 1 /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [1,255,0,0,0,255,0,0,0,255,255,127,0,255,255,0,127,0,127,0,127,127,255,255,255,0]}}'`
  * Green full blink with 500ms (Period does care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [2,0,255,0,500]}}'`
  * Blue left/right blink with 1s (Period does care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [3,0,0,255,1000]}}'`
  * White quad-blink with 1s (Period does care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [4,255,255,255,1000]}}'`
  * Red ring (Period does not care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [5,255,0,0,0]}}'`
  * Red ring-reverse (Period does not care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [6,255,0,0,0]}}'`
  * Initial color blink with 100ms (Period does not care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [7,100]}}'`
  * Specific color (blue/white/red) blink with 100ms (Period does not care): `rostopic pub /amiro/lights amiro_msgs/UInt16MultiArrayStamped '{array: {data: [8,0,0,255,255,255,255,255,0,0,1000]}}'`
  * ... (all other commands but with specific colors)
* Driving
  * Drive a circle with .1m/s and 1rad/s: `rostopic pub /amiro/motor geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"`
  * Drive to relative poistition: `rostopic pub /amiro/pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'`


