//
// Created by jonas on 03.04.19.
//

#include "HololensRobotControllerNode.h"
#include <ros/console.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>

HololensRobotControllerNode::HololensRobotControllerNode() : markerPoseCorrectionRate(0.2), tfListener(tfBuffer) {
    ros::NodeHandle nh;
    std::string param;

    this->keepRunning = false;
    double frequency;
    nh.param<double>("/HololensRobotController/markerCorrectionRate", frequency, 0.2);
    this->markerPoseCorrectionRate = ros::Rate(frequency);

    // Get coordinate frame names
    nh.param<std::string>("/HololensRobotController/trimbotMarkerFrameName", this->trimbotMarkerFrameName, "base_marker");
    nh.param<std::string>("/HololensRobotController/hololensWorldFrameName", this->hololensWorldFrameName, "hololens_world");
    nh.param<std::string>("/HololensRobotController/trimbotWorldFrameName", this->trimbotWorldFrameName, "map");
    nh.param<std::string>("/HololensRobotController/trimbotBaseFrameName", this->trimbotBaseFrameName, "base_footprint");

    // Create subscribers and publishers
    nh.param<std::string>("/HololensRobotController/hololensMeshTopic", param, "/hololens/point_cloud");
    this->hololensMeshSub = nh.subscribe(param, 1, &HololensRobotControllerNode::onNewHololensMesh, this);
    nh.param<std::string>("/HololensRobotController/markerPoseTopic", param, "/hololens/marker_pose");
    this->markerPoseSub = nh.subscribe(param, 1, &HololensRobotControllerNode::onNewMarkerPose, this);
    nh.param<std::string>("/HololensRobotController/trimbotMapTopic", param, "/slam/map");
    this->trimbotMapSub = nh.subscribe(param, 1, &HololensRobotControllerNode::onNewTrimbotMap, this);
    nh.param<std::string>("/HololensRobotController/hololensGoalPoseTopic", param, "/hololens/goal");
    this->hololensGoalPoseSub = nh.subscribe(param, 1, &HololensRobotControllerNode::onNewHololensGoal, this);

    nh.param<std::string>("/HololensRobotController/alignedMapTopic", param, "/trimbot/alignedmap");
    this->alignedMapPub = nh.advertise<sensor_msgs::PointCloud2>(param, 1);
    nh.param<std::string>("/HololensRobotController/trimbotGoalPoseTopic", param, "/move_base_simple/goal");
    this->trimbotGoalPosePub = nh.advertise<geometry_msgs::PoseStamped>(param, 1);

    this->pointCloudAlignment.setPublisher(this->alignedMapPub);
    this->publishMarkerFrame();
}

void HololensRobotControllerNode::onNewHololensMesh(const sensor_msgs::PointCloud2ConstPtr& msg) {
    this->pointCloudAlignment.setHololensMap(msg);
}

void HololensRobotControllerNode::onNewTrimbotMap(const sensor_msgs::PointCloud2ConstPtr& msg) {
    this->pointCloudAlignment.setTrimbotMap(msg);
}

void HololensRobotControllerNode::onNewMarkerPose(const geometry_msgs::PoseStampedConstPtr& msg) {
    ROS_DEBUG("Received marker pose.");

    // Extract transformation from msg and invert it
    tf2::Stamped<Eigen::Isometry3d> markerTransformStamped;
    tf2::fromMsg(*msg, markerTransformStamped);
    Eigen::Isometry3d markerTransform = markerTransformStamped.inverse();

    // Publish inverted transformation via TransformBroadcaster
    geometry_msgs::TransformStamped outMsg = tf2::eigenToTransform(markerTransform);
    outMsg.header.stamp = msg->header.stamp;
    //outMsg.header.stamp = ros::Time::now();
    outMsg.header.frame_id = this->trimbotMarkerFrameName;
    outMsg.child_frame_id = this->hololensWorldFrameName;
    tfBroadcaster.sendTransform(outMsg);

    // Get transformation from trimbot world frame to hololens map frame
    try {
        geometry_msgs::TransformStamped prior = tfBuffer.lookupTransform(this->hololensWorldFrameName, this->trimbotWorldFrameName, ros::Time(0), ros::Duration(1.0));
        this->priorTimestamp = msg->header.stamp;
        this->pointCloudAlignment.setPrior(prior);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        // TODO: maybe reset prior estimate instead of keeping the last one?
    }
}

void HololensRobotControllerNode::onNewHololensGoal(const geometry_msgs::PoseStampedConstPtr& msg) {
    ROS_DEBUG("Received hololens goal pose.");
    Eigen::Matrix4d posteriorTransformation;
    tf2::Stamped<Eigen::Isometry3d> goalStamped;
    tf2::fromMsg(*msg, goalStamped);
    Eigen::Matrix4d goal = goalStamped.matrix();
    geometry_msgs::PoseStamped outMsg;

    if(this->pointCloudAlignment.getValidPosterior(posteriorTransformation)) {
        Eigen::Matrix4d transformedGoal = posteriorTransformation.inverse() * goal;
        tf2::Stamped<Eigen::Isometry3d> tmp = tf2::Stamped<Eigen::Isometry3d>(Eigen::Isometry3d(transformedGoal), msg->header.stamp, this->trimbotWorldFrameName);
        outMsg = tf2::toMsg(tmp);
        outMsg.pose.orientation.x = 0.0;
        outMsg.pose.orientation.y = 0.0;
        outMsg.pose.orientation.z = 0.0;
        outMsg.pose.orientation.w = 1.0;

        ROS_DEBUG("Publishing hololens goal pose.");
        this->trimbotGoalPosePub.publish(outMsg);
    }
}

void HololensRobotControllerNode::run() {
    Eigen::Matrix4d posteriorTransformation;
    geometry_msgs::TransformStamped trimbotBaseToWorldTransformMsg;
    Eigen::Matrix4d trimbotBaseToWorld;
    Eigen::Matrix4d correctedMarkerPoseMatrix;
    Eigen::Isometry3d correctedMarkerPose;
    geometry_msgs::TransformStamped outMsg;

    this->keepRunning = true;
    this->pointCloudAlignment.start();

    while(ros::ok() && this->keepRunning) {
        // update marker <-> hololens_world transformation for rviz visualization only
        if (this->pointCloudAlignment.getValidPosterior(posteriorTransformation)) {
            try {
                trimbotBaseToWorldTransformMsg = tfBuffer.lookupTransform(this->trimbotWorldFrameName, this->trimbotBaseFrameName, ros::Time(0), ros::Duration(1.0));
                trimbotBaseToWorld = tf2::transformToEigen(trimbotBaseToWorldTransformMsg).matrix();

                correctedMarkerPoseMatrix = posteriorTransformation * trimbotBaseToWorld * this->trimbotMarkerToBaseTransformation;
                correctedMarkerPose = Eigen::Isometry3d(correctedMarkerPoseMatrix);
                outMsg = tf2::eigenToTransform(correctedMarkerPose);
                outMsg.header.stamp = this->priorTimestamp;
                //outMsg.header.stamp = ros::Time::now();
                outMsg.header.frame_id = this->trimbotMarkerFrameName;
                outMsg.child_frame_id = this->hololensWorldFrameName;

                //ROS_DEBUG("Sending corrected marker pose");
                //tfBroadcaster.sendTransform(outMsg);
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
        }

        ros::spinOnce();
        this->markerPoseCorrectionRate.sleep();
    }
}

void HololensRobotControllerNode::publishMarkerFrame() {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time(0);
    transformStamped.header.frame_id = this->trimbotBaseFrameName;
    transformStamped.child_frame_id = this->trimbotMarkerFrameName;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.7;
    tf2::Quaternion q;
    q.setRPY(0.0, -1.57, 0.0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // Store matrix form transformation
    this->trimbotMarkerToBaseTransformation = tf2::transformToEigen(transformStamped).inverse().matrix();

    stb.sendTransform(transformStamped);
}

void HololensRobotControllerNode::stop() {
    this->keepRunning = false;
    this->pointCloudAlignment.stop();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hololens_robot_controller");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ROS_INFO("Started hololens_robot_controller_node.");

    HololensRobotControllerNode* hrc = new HololensRobotControllerNode();

    hrc->run();
    hrc->stop();
    return 0;
}