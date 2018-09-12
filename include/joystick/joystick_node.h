#ifndef JOYSTICK_NODE_H_
#define JOYSTICK_NODE_H_
/* Joystick Node. Takes input from a joystick/game pad and outputs current state in a sensor_msgs/joy topic.
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <unistd.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <sys/time.h>
//#include <sys/types.h>

class Joystick
{
public:
    Joystick(ros::NodeHandle n, std::string device);
    void process(void);

private:
    ros::NodeHandle nh_;
    ros::Publisher joy_status_pub_;    // Topic to publish the joystick status
    ros::Time time_last_msg_;
    
    sensor_msgs::Joy joyMsgs_;         // Actual message published
    
    int js_;    // Joystick handle
    fd_set set_;
    struct timeval tv_;
    
    int getButtonCount(void);
    int getAxisCount(void);

};

#endif // JOYSTICK_NODE_H_

