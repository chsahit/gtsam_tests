#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#include <math.h>

ros::Publisher posePub;

auto currentDistance = 0;
auto currentAngle = 0;

auto measuredAngle = 0.f;
auto measuredEncoder = 0.f;

void angleCB(const std_msgs::Float32::ConstPtr &angleMsg) {
    measuredAngle = angleMsg->data;
}

void encoderCB(const std_msgs::Float32::ConstPtr &encoderMsg) {
    measuredEncoder = encoderMsg->data;
}

void updatePose() {
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle nh;

    auto encoderSub = nh.subscribe("/encoder", 1, encoderCB);
    auto angleSub = nh.subscribe("/angle", 1, angleCB);
    posePub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        updatePose();
    }
}
