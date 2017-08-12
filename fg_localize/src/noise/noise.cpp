#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <random>

ros::Publisher encoderPub;
std::default_random_engine generator;
std::normal_distribution<double> distribution(0, 0.2);

void encoderCB(const std_msgs::Float32::ConstPtr &encoderMsg) {
    auto signal = encoderMsg->data;
    auto noise = distribution(generator);
    std_msgs::Float32 noisy;
    noisy.data = signal + noise;
    encoderPub.publish(noisy);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "noise");
    ros::NodeHandle nh;
    encoderPub = nh.advertise<std_msgs::Float32>("/encoder", 1);
    auto encSub = nh.subscribe("/enc", 1, encoderCB);
    ros::spin();
}
