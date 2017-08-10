#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

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

auto poseNum = 1;
auto lastAngle = 0.f;
auto measuredX = 0.f;
auto lastX = 0.f;
auto measuredY = 0.f;
auto lastY = 0.f;
auto speed = 1;

void updateFactorGraph() {
    auto changeX = measuredX - lastX;
    auto changeY = measuredY - lastY;
    Pose2 odometry(changeX, changeY, lastAngle);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));
    graph.add(BetweenFactor<Pose2>(poseNum, poseNum + 1, odometry, odometryNoise));
    initial.insert(poseNum + 1, Pose2(measuredX, measuredY, 0.0));
    poseNum++;
}

void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    lastX = measuredX;
    lastY = measuredY;
    measuredX = odomMsg->pose.pose.position.x;
    measuredY = odomMsg->pose.pose.position.y;
    updateFactorGraph();
}


void predictPose() {
    result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle nh;

    auto odomSub = nh.subscribe("/odom", 1, odomCB);
    posePub = nh.advertise<geometry_msgs::Pose>("/pose", 1);

    Pose2 priorMean(0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
    initial.insert(1, Pose2(0.0, 0.0, 0.0));

    ros::Rate rate(3);
    while (ros::ok()) {
        ros::spinOnce();
        predictPose();
        if (result.size() > 1) {
            auto newestPose = result.at<Pose2>(result.size() - 1);
            ROS_INFO_STREAM("Pose: " << newestPose.x() << " " << newestPose.y());
        }
        rate.sleep();
    }
}
