//Tugas nomor 2
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <thread>

class letterWrite {
    public:
        ros::NodeHandle nh;
        geometry_msgs::Twist move_msg;
        ros::Publisher robot_run;

        letterWrite(const std::string& name) : nh() {
            robot_run = nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 10);
        }

    void moveTurtleRotate(float angleDegree) {
        ros::Rate rate(10);

        float angleRads = angleDegree * (M_PI / 180);
        float angleVel = 1;
        float durationAngle = abs(angleRads / angleVel);

        if (angleRads > 0) {
            move_msg.angular.z = angleVel;
        } else {
            move_msg.angular.z = -angleVel;
        }

        ros::Time startTime = ros::Time::now();
        while(true) {
            ros::Time currentTime = ros::Time::now();
            if ((currentTime - startTime).toSec() < durationAngle) {
                robot_run.publish(move_msg);
            } else {
                break;
            }
            rate.sleep();
        }

        move_msg.angular.z = 0;
        robot_run.publish(move_msg);
        ros::Duration(0.5).sleep();
    }

    void moveTurtleForward(float direction, float duration) {
        ros::Rate rate(10);

        ros::Time startTime = ros::Time::now();
        move_msg.linear.x = direction;
        while(true) {
            ros::Time currentTime = ros::Time::now();
            if ((currentTime - startTime).toSec() < duration) {
                robot_run.publish(move_msg);
            } else {
                break;
            }
            rate.sleep();
        }

        move_msg.linear.x = 0;
        robot_run.publish(move_msg);
        ros::Duration(0.5).sleep();
    }

    void letters(const std::string& letter) {
        if (letter == "A") {
            moveTurtleForward(1, 3);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 3);
            moveTurtleForward(-1, 1.5);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
        } else if (letter == "S") {
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(90);
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(90);
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
        } else if (letter == "L") {
            moveTurtleForward(1, 1.5);
            moveTurtleForward(-1, 1.5);
            moveTurtleRotate(90);
            moveTurtleForward(1, 3);
        } else if (letter == "M") {
            moveTurtleForward(1, 3);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 3);
            moveTurtleForward(-1, 3);
            moveTurtleRotate(90);
            moveTurtleForward(1, 1);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 3);
        } else if (letter == "P") {
            moveTurtleForward(1, 3);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
            moveTurtleRotate(-90);
            moveTurtleForward(1, 1.5);
        } else if (letter == "Star") {
            for (int i = 0; i < 5; ++i) {
                moveTurtleForward(1, 2);
                moveTurtleRotate(-144);
            }
        }
    }
};

class robotHandler {
    public:
        ros::NodeHandle nh;
        ros::ServiceClient client;

        robotHandler() : nh() {}

    void turtleSpawner(const std::string& name, float X, float Y, float thetaDeg) {
        client = nh.serviceClient<turtlesim::Spawn>("spawn");

        turtlesim::Spawn turtle;
        turtle.request.x = X;
        turtle.request.y = Y;
        turtle.request.name = name;
        turtle.request.theta = thetaDeg * (M_PI / 180);

        client.call(turtle);
    }

    void killDefaultTurtle(const std::string& name) {
        client = nh.serviceClient<turtlesim::Kill>("kill");
        turtlesim::Kill kill_it;
        kill_it.request.name = name;

        client.call(kill_it);
    }
};

void controlTurtle(const std::string& name, const std::string& letter, float x, float y, float angleDegree) {
    robotHandler robotSetup;
    robotSetup.turtleSpawner(name, x, y, angleDegree);
    letterWrite write(name);
    write.letters(letter);
    robotSetup.killDefaultTurtle(name);
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "Robot_nama");

    robotHandler robotSetup;
    robotSetup.killDefaultTurtle("turtle1");

    std::thread r1(controlTurtle, "turtle1", "A", 0.5, 5, 90);
    std::thread r2(controlTurtle, "turtle2", "S", 2.5, 5, 0);
    std::thread r3(controlTurtle, "turtle3", "L", 4.5, 5, 0);
    std::thread r4(controlTurtle, "turtle4", "A", 6.5, 5, 90);
    std::thread r5(controlTurtle, "turtle5", "M", 8.5, 5, 90);
    std::thread r6(controlTurtle, "turtle6", "P", 0.5, 1, 90);
    
    std::thread r7(controlTurtle, "turtle7", "Star", 5, 1, 90);
    std::thread r8(controlTurtle, "turtle8", "Star", 7, 3, 90);
    std::thread r9(controlTurtle, "turtle9", "Star", 9, 1, 90);

    r1.join();
    r2.join();
    r3.join();
    r4.join();
    r5.join();
    r6.join();
    r7.join();
    r8.join();
    r9.join();

    return 0;
}
