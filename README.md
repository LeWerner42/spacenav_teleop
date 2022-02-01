# Spacenav Teleop Node #

## Purpose
This little ROS node (tested with noetic) provides an ROS interface for the well known 3DConnexion SpaceMouse®.
Control inputs from the device are published as `sensor_msgs/Joy` and `geometry_msgs/Twist`.
The `geometry_msgs/Twist` output can be disabled by pushing Button 2 (the right one if you have a SpaceMouse® Compact).
Output topics for `Joy` and `Twist` messages can be set as a ROS parameter (just have a look at the exemplary launch file).
They default to `cmd_vel` for Twist and `spacenav/joy` for `Joy` respectively.
While the output range for the `Joy` messages is limited to [-1...1] a prescaler for the `Twist` output can be set for the linear and rotational axis as shown in the launch file as well.
This enables for example usage of the SpaceNav device for both, controlling the camera in GAZEBO simulator AND driving a simulated (or real) rover.

## Prequesits
This node works with the [spacenavd](http://spacenav.sourceforge.net/) driver.
It can be aquired from packet sources by:
```sh
sudo apt install spacenavd
```
Other than that, just clone the repository into your catkin WS, build it and you are good to go.

## Run
You can run the Node with default parameters by:
```sh
rosrun spacenav_teleop spacenav_teleop
```
* /cmd_vel
* /spacenav/joy

Will then be the published topics.

Otherwise, you can start with parameters for Topic names and `Twist` prescaler values in a launch file: 

```xml
<launch>
  <node pkg="spacenav_teleop" type="spacenav_teleop" name="spacenav_teleop_node" output="screen">
    <param name="twistLinearScaler" value="2"/>
    <param name="twistRotationScaler" value="2"/>
    <param name="twistTopic" value="cmd_vel"/>
    <param name="joyTopic" value="spacenav/joy"/>
  </node>
</launch>
```

## Change to other Space... Devices
While the present Node was written for a SpaceMouse Compact, other SpaceNav devices should work as well.
Axis mapping for `Twist` and `Joy` can be adapted by modifying the indexing as already done for the `Twist` messages:
```C++
    twist_msg.linear.x  =  last_spacenav_axis[2] / hwSpecificScaling * twistLinScale;
    twist_msg.linear.y  = -last_spacenav_axis[0] / hwSpecificScaling * twistLinScale;
    twist_msg.linear.z  =  last_spacenav_axis[1] / hwSpecificScaling * twistLinScale;
    twist_msg.angular.x =  last_spacenav_axis[5] / hwSpecificScaling * twistRotScale;
    twist_msg.angular.y =  last_spacenav_axis[3] / hwSpecificScaling * twistRotScale;
    twist_msg.angular.z =  last_spacenav_axis[4] / hwSpecificScaling * twistRotScale;
```