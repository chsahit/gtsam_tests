#include <ros/ros.h>

ros::Publisher posePub;

void encoderCB() {

}

void predictPose() {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle nh;

    auto encoderSub = nh.subscribe("/encoder", 1, encoderCB);
    posePub = nh.advertise<<Pose2>>("/pose", 1);

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        predictPose();
    }
}
