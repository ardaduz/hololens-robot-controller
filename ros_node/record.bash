#!/bin/bash

rosbag record -o ${1}_slim /hololens/marker_pose /hololens/point_cloud /hololens/pose /tf /tf_static /slam/map /trimbot/alignedmap /trimbot/goal /localization_node/localization/currentPose /cmd_vel /move_base/NavfnROS/plan /move_base_simple/goal /move_base/goal /move_base/result /move_base/status
