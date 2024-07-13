#include <ros/ros.h>
#include "can_interfaces/CtrlCmd.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

can_interfaces::CtrlCmd my_cmd;

void
sub_move_base(const geometry_msgs::Twist::ConstPtr& msg)
{
    my_cmd.ctrl_cmd_velocity = (int16_t)(msg->linear.x / 0.00066);
    my_cmd.ctrl_cmd_steering = (int16_t)(msg->angular.z / 0.06 / M_PI * 180);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "movebase_bridge");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Publisher vel_pub = nh.advertise<can_interfaces::CtrlCmd>("ctrl_cmd", 1);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, sub_move_base);

    while(ros::ok())
    {
        vel_pub.publish(my_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


