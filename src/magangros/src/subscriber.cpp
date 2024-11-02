#include <ros/ros.h>
#include <std_msgs/Float64.h>

void chatterCallback(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I heard: [%.2f]", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/pengumuman", 10, chatterCallback);

    ros::spin();

    return 0;
}   