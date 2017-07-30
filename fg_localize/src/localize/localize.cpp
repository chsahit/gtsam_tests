#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <math.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtsam;

ros::Publisher posePub;

NonlinearFactorGraph graph;
Values initial;
Values result;
int poseNum = 2;

float lastAngle = 0;
float lastEncoder = 0;

void angleCB(const std_msgs::Float32::ConstPtr &angleMsg) {
    lastAngle = angleMsg->data;
}

void encoderCB(const std_msgs::Float32::ConstPtr &encoderMsg) {
    lastEncoder = encoderMsg->data;
}

void updateFactorGraph() {
    Pose2 odometry(lastEncoder * sin(lastAngle), lastEncoder * cos(lastAngle), lastAngle);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(poseNum, poseNum + 1, odometry, odometryNoise));
    initial.insert(poseNum + 1, Pose2(0.0, 0.0, 0.0));
    poseNum++;
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

    Pose2 priorMean(0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
    initial.insert(1, Pose2(0.0, 0.0, 0.0));
    initial.insert(2, Pose2(0.0, 0.0, 0.0));

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO("update factor graph 1");
        updateFactorGraph();
        ros::spinOnce();
        ROS_INFO("update factor graph 2");
        updateFactorGraph();
        ROS_INFO("predictPose");
        predictPose();
        result.print("Final Result:\n");
    }
}
