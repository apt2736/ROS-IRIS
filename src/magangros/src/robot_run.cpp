#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std;

void turtleSpawner(const string& name, float X, float Y, float angleDegree) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("spawn");
    float angleRads = angleDegree * (M_PI / 180);

    turtlesim::Spawn turtle;
    turtle.request.x = X;
    turtle.request.y = Y;
    turtle.request.name = name;
    turtle.request.theta = angleRads;

    client.call(turtle);
}

void killDefaultTurtle(const string& name) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("kill");
    turtlesim::Kill kill_it;
    kill_it.request.name = name;

    client.call(kill_it);
}

void moveTurtle (float targetX, float targetY) {
    ros::NodeHandle nh;
    ros::Publisher robot_run = nh.advertise<geometry_msgs::Twist>("/turtle_main/cmd_vel", 10);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist move_msg;

    float distance = sqrt(pow(targetX / 100, 2) + pow(targetY / 100, 2));
    float angle = atan2(targetY, targetX);

    move_msg.linear.x = 10 * distance;
    move_msg.angular.z = angle;

    ros::Time startTime = ros::Time::now();
    while (true) {
        ros::Time currentTime = ros::Time::now();
        if ((currentTime - startTime).toSec() < 0.036) {
            robot_run.publish(move_msg);
        } else {
            break;
        }
        loop_rate.sleep();
    }
                
    move_msg.linear.x = 0;
    move_msg.angular.z = 0;
    robot_run.publish(move_msg);
    ros::Duration(0.1).sleep();
}

int main(int argc, char **argv) {   
    ros::init(argc, argv, "Robot");
    ros::NodeHandle nh;
    bool firstMove = true;

    killDefaultTurtle("turtle1");
    turtleSpawner("turtle_main", 1, 1, 90);

    ros::Publisher robot_run = nh.advertise<geometry_msgs::Twist>("/turtle_main/cmd_vel", 10);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist move_msg;

    VideoCapture cap ("/home/apt2736/Downloads/Video.mp4");

    Mat frame;
    Mat frame_hsv;
    Mat threshold_hsv;

    while(ros::ok()) {
        while(ros::ok()) {
            cap >> frame;

            if (frame.empty()) {
                break;
            }

            cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
            inRange(frame_hsv, Scalar(7, 131, 120), Scalar(20, 255, 255), threshold_hsv);

            vector<vector<Point>> contours;
            findContours(threshold_hsv, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            float radius, prevX, prevY;
            Point2f center;
            double maxArea = 0;

            for (size_t i = 0; i < contours.size(); ++i) {
                double area = contourArea(contours[i]);
                if (area > maxArea) {
                    maxArea = area;
                    minEnclosingCircle(contours[i], center, radius);
                } else if ((int)area == (int)maxArea) {
                    continue;
                }
            }

            if (maxArea > 100) {
                float targetX, targetY;

                if(firstMove) {
                    prevX = center.x - 946;
                    prevY = 503 - center.y;
                    firstMove = false;
                    continue;
                }

                targetX = prevX - (center.x - 946);
                targetY = prevY - (503 - center.y);

                prevX = center.x - 946;
                prevY = 503 - center.y;
                
                moveTurtle(targetX, targetY);
            } 

            if (waitKey(30) == 'q') {
                break;
            }

            ros::spinOnce();
        }
    }

    cap.release();
    return 0;
}