//
// Created by jonas on 03.04.19.
//

#ifndef SRC_HOLOLENSROBOTCONTROLLERNODE_H
#define SRC_HOLOLENSROBOTCONTROLLERNODE_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>


#include "PointCloudAlignment.h"

class HololensRobotControllerNode {
private:
    tf2_ros::TransformBroadcaster tfBroadcaster;
    tf2_ros::StaticTransformBroadcaster stb;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Subscriber hololensMeshSub;
    ros::Subscriber trimbotMapSub;
    ros::Subscriber markerPoseSub;
    ros::Subscriber hololensGoalPoseSub;
    ros::Publisher alignedMapPub;
    ros::Publisher trimbotGoalPosePub;

    bool keepRunning;
    std::string trimbotMarkerFrameName;
    std::string hololensWorldFrameName;
    std::string trimbotWorldFrameName;
    std::string trimbotBaseFrameName;

    PointCloudAlignment pointCloudAlignment;

    // due to limitations of the tf2 package, the trimbot ego motion is erroneously applied to hololens_world_frame
    // to remove this error, main thread publishes corrected marker_pose -> hololens_world_frame transformations
    // The markerPoseCorrectionRate sets the update rate of this correction. It's only needed for visualization in rviz, hololens visualization is unaffected!
    ros::Rate markerPoseCorrectionRate;
    ros::Time priorTimestamp;
    Eigen::Matrix4d trimbotMarkerToBaseTransformation;

    void publishMarkerFrame();

public:
    HololensRobotControllerNode();

    void onNewHololensMesh(const sensor_msgs::PointCloud2ConstPtr&);
    void onNewTrimbotMap(const sensor_msgs::PointCloud2ConstPtr&);
    void onNewMarkerPose(const geometry_msgs::PoseStampedConstPtr&);
    void onNewHololensGoal(const geometry_msgs::PoseStampedConstPtr&);
    void run();
    void stop();
};

#endif //SRC_HOLOLENSROBOTCONTROLLERNODE_H
