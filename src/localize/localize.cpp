#include <ros/ros.h>
#include <math.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

ros::Publisher posePub;

gtsam::NonlinearFactorGraph graph;
int nextPose = 2;

float lastAngle = 0;
float lastEncoder = 0;

void angleCB() {
    lastAngle = 2/*angle*/;
}

void encoderCB() {
    lastEncoder = 2/*enc*/;
}

void predictPose() {
    gtsam::Pose2 odometry(lastEncoder * sin(lastAngle), lastEncoder * cos(lastAngle), lastAngle);
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(nextPose - 1, nextPose, odometry, odometryNoise));
    nextPose++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle nh;

    auto encoderSub = nh.subscribe("/encoder", 1, encoderCB);
    auto angleSub = nh.subscribe("/angle", 1, angleCB);
    posePub = nh.advertise<<Pose2>>("/pose", 1);

    gtsam::Pose2 priorMean(0.0, 0.0, 0.0);
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(1, priorMean, priorNoise));

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        predictPose();
    }
}
