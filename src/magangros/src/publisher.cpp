#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher pub_data = nh.advertise<std_msgs::Float64>("/pengumuman", 1);

    // Time step
    ros::Rate loop_rate(1);

    // Main
    while (ros::ok())
    {
        static float counter = 0;
        std_msgs::Float64 msg;
        msg.data = ++counter;

        pub_data.publish(msg);
        printf("%f", msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}