#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <math.h>

ros::Publisher posePub;

auto lastAngle = 0.f;
auto lastEncoder = 0.f;

void angleCB(const std_msgs::Float32::ConstPtr &angleMsg) {
    lastAngle = angleMsg->data;
}

void encoderCB(const std_msgs::Float32::ConstPtr &encoderMsg) {
    lastEncoder = encoderMsg->data;
}

void predictPose() {
    result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle nh;

    auto encoderSub = nh.subscribe("/encoder", 1, encoderCB);
    auto angleSub = nh.subscribe("/angle", 1, angleCB);
    posePub = nh.advertise<geometry_msgs::Pose>("/pose", 1);

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
