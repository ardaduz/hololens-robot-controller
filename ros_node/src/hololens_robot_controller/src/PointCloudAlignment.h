//
// Created by jonas on 29.04.19.
//

#ifndef SRC_POINTCLOUDALIGNMENT_H
#define SRC_POINTCLOUDALIGNMENT_H

#include <mutex>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>


class PointCloudAlignment {
private:
    int MIN_POINT_CLOUD_SIZE;
    double RESET_ON_RELATIVE_CHANGE;
    double FITNESS_SCORE_RELAXATION_FACTOR;

    bool keepRunning;
    bool dataChanged;
    bool isPosteriorValid;
    int hololensPointCount;
    int trimbotPointCount;

    ros::Rate rate;
    boost::thread* pThread;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher publisher;

    // Some mutexes for thread safety
    std::mutex pointCloudMutex;
    std::mutex priorMutex;
    std::mutex posteriorMutex;

    // Latest maps
    sensor_msgs::PointCloud2 latestHololensMsg;
    sensor_msgs::PointCloud2 latestTrimbotMsg;

    // Prior and posterior pose estimates
    Eigen::Vector3d priorTranslation;
    Eigen::Quaterniond priorRotation;
    Eigen::Matrix4d priorTransformation;
    Eigen::Matrix4d posteriorTransformation;

    // ICP fitness score
    double minFitnessScore;

    // Aligned and wrapped trimbot point cloud
    sensor_msgs::PointCloud2 alignedTrimbotMsg;

    void align();
    void publish();
    void run();

public:
    PointCloudAlignment();
    bool getValidPosterior(Eigen::Matrix4d&);
    void setHololensMap(const sensor_msgs::PointCloud2ConstPtr&);
    void setTrimbotMap(const sensor_msgs::PointCloud2ConstPtr&);
    void setPublisher(const ros::Publisher& publisher);
    void setPrior(const geometry_msgs::TransformStamped& prior);
    void start();
    void stop();
};


#endif //SRC_POINTCLOUDALIGNMENT_H
