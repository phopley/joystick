// Joystick Node. Takes input from a joystick/game pad and outputs current state in a sensor_msgs/joy topic.
// See https://www.kernel.org/doc/Documentation/input/joystick-api.txt
#include <joystick/joystick_node.h>

#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>

// Constructor 
Joystick::Joystick(ros::NodeHandle n, std::string device)
{
    nh_ = n;
    
    // Advertise the topics we publish
    joy_status_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 5);
        
    js_ = open(device.c_str(), O_RDONLY);
    
    if (js_ == -1)
    {
        ROS_ERROR("Problem opening joystick device");
        ros::shutdown();
    }
    else
    {
        int buttons = getButtonCount();
        int axes = getAxisCount();

        joyMsgs_.buttons.resize(buttons);
        joyMsgs_.axes.resize(axes);
        
        ROS_INFO("Joystick number of buttons %d, number of axis %d", buttons, axes);
    }
}

// Process the joystick input
void Joystick::process(void)
{
    js_event event;
        
    FD_ZERO(&set_);
    FD_SET(js_, &set_);
    
    tv_.tv_sec = 0;
    tv_.tv_usec = 250000;
    
    int selectResult = select(js_+1, &set_, NULL, NULL, &tv_);
    
    if(selectResult == -1)
    {
        ROS_ERROR("Error with select joystick call"); // Error
    }
    else if (selectResult)
    {
        // Data available
        if(read(js_, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
        {
            // Joystick probably closed
            ;
        }
        else
        {
            switch (event.type)
            {
                case JS_EVENT_BUTTON:
                case JS_EVENT_BUTTON | JS_EVENT_INIT:
                    // Set the button value                    
                    joyMsgs_.buttons[event.number] = (event.value ? 1 : 0);
                    
                    time_last_msg_ = ros::Time::now();
                    
                    joyMsgs_.header.stamp = time_last_msg_;
                    
                    // We publish a button press right away so they are not missied
                    joy_status_pub_.publish(joyMsgs_);
                    break;
                    
                case JS_EVENT_AXIS:
                case JS_EVENT_AXIS | JS_EVENT_INIT:
                    // Set the axis value 
                    joyMsgs_.axes[event.number] = event.value;
                    
                    // Only publish if time since last regular message as expired
                    if((ros::Time::now() - time_last_msg_).toSec() > 0.1f)                    
                    {
                        time_last_msg_ = ros::Time::now();
                    
                        joyMsgs_.header.stamp = time_last_msg_;
                        
                        // Time to publish
                        joy_status_pub_.publish(joyMsgs_);                 
                    }

                default:                    
                    break;            
            }
        }  
    }
    else
    {
        // No data available, select time expired.
        // Publish message to keep anything alive that needs it
        
        time_last_msg_ = ros::Time::now();
        
        joyMsgs_.header.stamp = time_last_msg_;
        
        // Publish the message
        joy_status_pub_.publish(joyMsgs_);
    }
}

// Returns the number of buttons on the controller or 0 if there is an error.
int Joystick::getButtonCount(void)
{
    int buttons;
    
    if (ioctl(js_, JSIOCGBUTTONS, &buttons) == -1)
    {
        buttons = 0;
    }

    return buttons;
}

// Returns the number of axes on the controller or 0 if there is an error.
int Joystick::getAxisCount(void)
{
    int axes;

    if (ioctl(js_, JSIOCGAXES, &axes) == -1)
    {
        axes = 0;
    }

    return axes;
}

int main(int argc, char **argv)
{
    std::string device;
    
    ros::init(argc, argv, "joystick_node");

    // ros::init() parses argc and argv looking for the := operator.
    // It then modifies the argc and argv leaving any unrecognized command-line parameters for our code to parse.
    // Use command line parameter to set the device name of the joystick or use a default.        
    if (argc > 1)
    {
        device = argv[1];
    }
    else
    {
        device = "/dev/input/js0";
    }
    
    ros::NodeHandle n;    
    Joystick joystick_node(n, device);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());	
	
	// We are not going to use ros::Rate here, the class will use select and 
	// return when it's time to spin and send any messages on the topic
    
    while(ros::ok())
    {
        // Check the joystick for an input and process the data
        joystick_node.process();
        
        ros::spinOnce();
    }
    
    return 0;    
}

