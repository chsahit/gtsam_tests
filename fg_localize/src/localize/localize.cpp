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

gtsam::NonlinearFactorGraph graph;
Values initial;
int nextPose = 2;

float lastAngle = 0;
float lastEncoder = 0;

void angleCB(const std_msgs::Float32::ConstPtr &angleMsg) {
    lastAngle = 2/*angle*/;
}

void encoderCB(const std_msgs::Float32::ConstPtr &encoderMag) {
    lastEncoder = 2/*enc*/;
}

void updateFactorGraph() {
    Pose2 odometry(lastEncoder * sin(lastAngle), lastEncoder * cos(lastAngle), lastAngle);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(nextPose - 1, nextPose, odometry, odometryNoise));
    nextPose++;
}

void predictPose() {
    //Values result = LevenbergMarquadtOptimizer(graph, initial).optimize();
    //--result.find(nextPose).end();
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

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        updateFactorGraph();
        ros::spinOnce();
        updateFactorGraph();
        predictPose();
    }
}
