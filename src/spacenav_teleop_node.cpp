#include <stdio.h>
#include "math.h"
#include "string.h"
#include "ros/node_handle.h"
#include "ros/param.h"

#include "spnav.h" // NOLINT

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

int main(int arc, char **argv)
{
    ros::init(arc, argv, "spacenav_teleop");
    ros::NodeHandle nh;

    // default parameters for topic names
    std::string twistTop = "cmd_vel";
    std::string joyTop = "spacenav/joy";

    // default parameters for twist output scaling
    static double twistLinScale = 10.0;
    static double twistRotScale = 10.0;

    // get parameters from the parameter server
    ros::param::get("~/twistTopic", twistTop);
    ros::param::get("~/joyTopic", joyTop);

    ros::param::get("~/twistLinearScaler", twistLinScale);
    ros::param::get("~/twistRotationScaler", twistRotScale);

    // setup publishers for the twist and joy messages
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>(twistTop, 1);
    ros::Publisher pub_joy = nh.advertise<sensor_msgs::Joy>(joyTop, 1);

    sensor_msgs::Joy joystick_msg;
    geometry_msgs::Twist twist_msg;

    // reshape joystick message to match present axis and button numbers on the SpaceMouse
    joystick_msg.axes.resize(6);
    joystick_msg.buttons.resize(2);

    spnav_event sev;
    if (spnav_open() == -1) // opens spacenavd object and checks, if connection was successfull
    {
        ROS_ERROR("Could not open the space navigator device.");
        return 1; // exit program, if spacemouse cannot be opened
    }

    // internally used variables
    static double last_spacenav_axis[6] = {0.0}; // x, y, z, rx, ry, rz
    static int last_spacenav_button[2] = {0};    // button 1, button 2
    static int zeroCnt = 0;                      // counter to reset axis to zero if no motion is detected
    static bool buttonUpdate = false;            // flag to indicate if the button state has changed and need to be handled
    static int mode = 0;                         // 0: twist on, 1: twist off

    ros::Rate loop_rate(1000); // we want to poll our spacenav device at 1kHz
    while (nh.ok())
    {
        joystick_msg.header.stamp = ros::Time::now();

        switch (spnav_poll_event(&sev))
        {
        case 0:                // no new data for...
            if (zeroCnt > 100) // 1/10th of a second at 1khz rate
            {
                for (int i = 0; i < 6; i++) // perform spacemouse zeroing
                {
                    last_spacenav_axis[i] = 0.0;
                }
                zeroCnt = 0; // reset zero counter
            }
            else
            {
                zeroCnt++; // if not yet reached, increase counter
            }
            break;

        case SPNAV_EVENT_MOTION: // spacenavd decides between motion and button events
            last_spacenav_axis[0] = sev.motion.x;
            last_spacenav_axis[1] = sev.motion.y;
            last_spacenav_axis[2] = sev.motion.z;
            last_spacenav_axis[3] = sev.motion.rx;
            last_spacenav_axis[4] = sev.motion.ry;
            last_spacenav_axis[5] = sev.motion.rz;
            break;

        case SPNAV_EVENT_BUTTON:
            buttonUpdate = true;
            last_spacenav_button[sev.button.bnum] = sev.button.press;
            break;

        default:
            ROS_WARN("invalid message from spacenavd");
            break;
        }

        // data aquisition from device complete

        static const double hwSpecificScaling = 350.0; // scales output from spacenav to [-1...0...1]

        joystick_msg.axes[0] = last_spacenav_axis[0] / hwSpecificScaling;
        joystick_msg.axes[1] = last_spacenav_axis[1] / hwSpecificScaling;
        joystick_msg.axes[2] = last_spacenav_axis[2] / hwSpecificScaling;
        joystick_msg.axes[3] = last_spacenav_axis[3] / hwSpecificScaling;
        joystick_msg.axes[4] = last_spacenav_axis[4] / hwSpecificScaling;
        joystick_msg.axes[5] = last_spacenav_axis[5] / hwSpecificScaling;

        joystick_msg.buttons[0] = last_spacenav_button[0];
        joystick_msg.buttons[1] = last_spacenav_button[1];

        // set twist message
        twist_msg.linear.x = last_spacenav_axis[2] / hwSpecificScaling * twistLinScale; // change axis order, so that X is forward, Y is left, Z is up
        twist_msg.linear.y = -last_spacenav_axis[0] / hwSpecificScaling * twistLinScale;
        twist_msg.linear.z = last_spacenav_axis[1] / hwSpecificScaling * twistLinScale;
        twist_msg.angular.x = last_spacenav_axis[5] / hwSpecificScaling * twistRotScale; // equally for rotational axis
        twist_msg.angular.y = last_spacenav_axis[3] / hwSpecificScaling * twistRotScale;
        twist_msg.angular.z = last_spacenav_axis[4] / hwSpecificScaling * twistRotScale;

        if (buttonUpdate) // only check for state change, if the last message was a button message. This ensures that the button state is not changed, if the user is still holding the button down
        {
            buttonUpdate = false;                          // reset flag for button update
            if (mode == 0 && last_spacenav_button[1] == 1) // if it was the correct (right) button was pressed & current mode is 0 -> toggle to mode 1
            {
                mode = 1;
                ROS_INFO("disabled twist output");
            }
            else if (mode == 1 && last_spacenav_button[1] == 1) // otherwise change to 0
            {
                mode = 0;
                ROS_INFO("enabled twist output");
            }
        }

        if (mode == 0) // mode 0 indicates enabled twist output -> publish twist message
        {
            pub_twist.publish(twist_msg);
        }

        // publish joystick message all the time
        pub_joy.publish(joystick_msg);

        loop_rate.sleep();
    }
    spnav_close();
    return 0;
}