//
// Created by jonas on 29.04.19.
//

#include "PointCloudAlignment.h"
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>

PointCloudAlignment::PointCloudAlignment() : rate(1.0), tfListener(tfBuffer) {
    ros::NodeHandle nh;

    double frequency;
    nh.param<double>("/HololensRobotController/pointCloudAlignmentRate", frequency, 0.2);
    this->rate = ros::Rate(frequency);

    nh.param<int>("/HololensRobotController/minPointCloudSize", this->MIN_POINT_CLOUD_SIZE, 1);
    nh.param<double>("/HololensRobotController/resetOnRelativeChangeOfSize", this->RESET_ON_RELATIVE_CHANGE, 0.25);
    nh.param<double>("/HololensRobotController/fitnessScoreRelaxationFactor", this->FITNESS_SCORE_RELAXATION_FACTOR, 1.1);

    this->keepRunning = false;
    this->dataChanged = false;
    this->hololensPointCount = 0;
    this->trimbotPointCount = 0;
    this->minFitnessScore = std::numeric_limits<double>::max();
    this->posteriorTransformation = Eigen::Matrix4d::Identity();
    this->isPosteriorValid = false;
}

void PointCloudAlignment::align() {
    bool isAligned = false;

    this->pointCloudMutex.lock();
    sensor_msgs::PointCloud2 hololensMsg = this->latestHololensMsg;
    sensor_msgs::PointCloud2 trimbotMsg = this->latestTrimbotMsg;
    double minFitnessScore = this->minFitnessScore;
    this->dataChanged = false;
    this->pointCloudMutex.unlock();

    this->priorMutex.lock();
    Eigen::Matrix4f prior = this->priorTransformation.cast<float>();
    this->priorMutex.unlock();

    ROS_DEBUG("Aligning the two latest point clouds...");

    pcl::PCLPointCloud2 hololensMap;
    pcl::PCLPointCloud2 trimbotMap;
    pcl::PCLPointCloud2 alignedMap;

    pcl_conversions::toPCL(hololensMsg, hololensMap);
    pcl_conversions::toPCL(trimbotMsg, trimbotMap);

    // Conversions from PointCloud2 to PointCloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr hololensMapPCL(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trimbotMapPCL(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedMapPCL(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::fromPCLPointCloud2(hololensMap,*hololensMapPCL);
    pcl::fromPCLPointCloud2(trimbotMap,*trimbotMapPCL);

    // Setting ICP parameters and run alignment
    icp.setMaximumIterations(1000);        //First finish criteria, which we want to avoid
    icp.setMaxCorrespondenceDistance(0.5);  
    icp.setTransformationEpsilon (0.0001);
    icp.setEuclideanFitnessEpsilon(0.0001);

    icp.setInputSource(trimbotMapPCL);
    icp.setInputTarget(hololensMapPCL);
    icp.align(*alignedMapPCL, prior);

    if (icp.hasConverged()) {
        double score = icp.getFitnessScore();

        if(score < minFitnessScore) {
            this->minFitnessScore = score;

            ROS_DEBUG("ICP converged, fitness score %f", score);
            this->posteriorMutex.lock();
            this->posteriorTransformation = icp.getFinalTransformation().cast<double>();
            this->isPosteriorValid = true;
            this->posteriorMutex.unlock();

            isAligned = true;
        } else {
            ROS_INFO("ICP fitness score too high: %f > %f", score, minFitnessScore);
            this->minFitnessScore = std::max(minFitnessScore * this->FITNESS_SCORE_RELAXATION_FACTOR, this->minFitnessScore);
        }
    } else {
        ROS_WARN("ICP has not converged!");
    }

    if(!isAligned) {
        pcl::transformPointCloud(*trimbotMapPCL, *alignedMapPCL, this->posteriorTransformation);
    }

    pcl::toPCLPointCloud2(*alignedMapPCL, alignedMap);
    pcl_conversions::fromPCL(alignedMap, this->alignedTrimbotMsg);
    this->alignedTrimbotMsg.header.frame_id = hololensMsg.header.frame_id;
    this->publish();
}

// If there is a valid posterior transformation, this function writes it into the out parameter and returns true. Returns false otherwise.
bool PointCloudAlignment::getValidPosterior(Eigen::Matrix4d& out) {
    if(this->isPosteriorValid) {
        this->posteriorMutex.lock();
        out = this->posteriorTransformation;
        this->posteriorMutex.unlock();

        return true;
    } else {
        return false;
    }
}

void PointCloudAlignment::publish() {
    // make sure point cloud has no more than 60k points
    // TODO: move magic number into static const or into config!
    if(this->alignedTrimbotMsg.height * this->alignedTrimbotMsg.width <= 60000) {
        this->publisher.publish(this->alignedTrimbotMsg);
    } else {
        ROS_WARN("Aligned point cloud is too large for hololens app and will not be published!");
    }
}

void PointCloudAlignment::run() {
    while (ros::ok() && this->keepRunning)
    {
        if (this->dataChanged && this->trimbotPointCount >= this->MIN_POINT_CLOUD_SIZE && this->hololensPointCount >= this->MIN_POINT_CLOUD_SIZE) {
            this->align();
            this->rate.sleep();
        } else {
            ros::Duration(0.001).sleep();
        }
    }
    this->keepRunning = false;
}

void PointCloudAlignment::setHololensMap(const sensor_msgs::PointCloud2ConstPtr& hololensMsg) {
    this->pointCloudMutex.lock();

    // Reset minFitnessScore if size of point cloud changes by at least 25%
    int pointCount = hololensMsg->width * hololensMsg->height;
    if(this->hololensPointCount * this->RESET_ON_RELATIVE_CHANGE < abs(this->hololensPointCount - pointCount)) {
        ROS_DEBUG("Hololens map size changed greatly, resetting minFitnessScore.");
        this->minFitnessScore = std::numeric_limits<double>::max();
    }

    this->hololensPointCount = pointCount;
    this->latestHololensMsg = *hololensMsg;
    this->dataChanged = true;

    this->pointCloudMutex.unlock();
}

void PointCloudAlignment::setTrimbotMap(const sensor_msgs::PointCloud2ConstPtr& trimbotMsg) {
    this->pointCloudMutex.lock();

    // Reset minFitnessScore if size of point cloud changes by at least 25%
    int pointCount = trimbotMsg->width * trimbotMsg->height;
    if(this->trimbotPointCount * this->RESET_ON_RELATIVE_CHANGE < abs(this->trimbotPointCount - pointCount)) {
        ROS_DEBUG("Trimbot map size changed greatly, resetting minFitnessScore.");
        this->minFitnessScore = std::numeric_limits<double>::max();
    }

    this->trimbotPointCount = pointCount;
    this->latestTrimbotMsg = *trimbotMsg;
    this->dataChanged = true;

    this->pointCloudMutex.unlock();
}

void PointCloudAlignment::setPublisher(const ros::Publisher& publisher) {
    this->publisher = publisher;
}

void PointCloudAlignment::setPrior(const geometry_msgs::TransformStamped& prior) {
    this->priorMutex.lock();
    this->priorTransformation = tf2::transformToEigen(prior).matrix();
    this->priorMutex.unlock();
}

void PointCloudAlignment::start() {
    if (this->publisher != nullptr && !this->keepRunning) {
        this->keepRunning = true;
        this->pThread = new boost::thread(boost::bind(&PointCloudAlignment::run, this));
    }
}

void PointCloudAlignment::stop() {
    if (this->keepRunning) {
        keepRunning = false;
        this->pThread->join();
    }
}