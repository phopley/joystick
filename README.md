# joystick
ROS node that takes input from a joystick/game pad and outputs current state in a sensor_msgs/joy topic.

## Running the Node

Once you have the node built you can run it with "rosrun joystick joystick_node".

## Node Information
Topics:

* `joy`:  
  Publishes `sensor_msgs/Joy` current state of the joystick/game pad
