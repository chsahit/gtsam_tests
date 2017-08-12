#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

#include <math.h>

ros::Publisher posePub;

auto currentX = 0.f;
auto currentY = 0.f;
auto currentAngle = 0.f;

auto measuredAngle = 0.f;
auto measuredEncoder = 0.f;

long double lastTime = -1;

void updatePose() {
    currentAngle = measuredAngle;
    long double deltaT = 0L;
    if (lastTime == -1) {
        lastTime = ros::Time::now().toSec();
    }
    else {
        long double now = ros::Time::now().toSec();
        deltaT = (now - lastTime);
        lastTime = now;
    }
    currentX += measuredEncoder * deltaT * cos(measuredAngle);
    currentY = measuredEncoder * deltaT * sin(measuredAngle) + currentY;
    ROS_INFO_STREAM("x pos: " << currentX << "\n");
}

void angleCB(const std_msgs::Float32::ConstPtr &angleMsg) {
    measuredAngle = angleMsg->data;
    updatePose();
}

void encoderCB(const std_msgs::Float32::ConstPtr &encoderMsg) {
    measuredEncoder = encoderMsg->data;
    updatePose();
}


void publishPose() {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();

    msg.pose.pose.position.x = currentX;
    msg.pose.pose.position.y = currentY;
    
    tf::Matrix3x3 eulerMat;
    eulerMat.setEulerYPR(measuredAngle, 0, 0);
    tf::Quaternion q_tf;
    eulerMat.getRotation(q_tf);
    msg.pose.pose.orientation.x = q_tf.getX();
    msg.pose.pose.orientation.y = q_tf.getY();
    msg.pose.pose.orientation.z = q_tf.getZ();
    msg.pose.pose.orientation.w = q_tf.getW();

    msg.twist.twist.linear.x = measuredEncoder * cos(measuredAngle);
    msg.twist.twist.linear.y = measuredEncoder * sin(measuredAngle);

    posePub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "model");
    ros::NodeHandle nh;

    auto encoderSub = nh.subscribe("/encoder", 1, encoderCB);
    auto angleSub = nh.subscribe("/angle", 1, angleCB);
    posePub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        publishPose();
   }
}
