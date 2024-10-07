/*
 * Copyright (C) 2024, Itamar Mishani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   manipulation_action_space.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Jan 19 2024
 */


#pragma once

// include standard libraries
#include <iostream>
#include <utility>
#include <vector>

// include ROS libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

// include tf to convert euler angles to quaternions
#include <tf/transform_datatypes.h>

// project includes
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <manipulation_planning/common/utils.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <search/action_space/action_space_mgs.hpp>
#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include "mobile_manipulation_action_space.hpp"

namespace ims {

/// @class ManipulationMGSActionSpace
/// @brief A class that implements the ActionSpaceMGS for Moveit.
/// @class ManipulationMGSActionSpace
class ManipulationMGSActionSpace : public ActionSpaceMGS, public ManipulationActionSpace {

public:
    /// @brief Constructor
    /// @param moveitInterface The moveit interface
    /// @param ManipulationType The manipulation type
    ManipulationMGSActionSpace(const MoveitInterface &env,
                               const ManipulationType &actions_ptr,
                               BFSHeuristic *bfs_heuristic = nullptr) :
                               ActionSpaceMGS(), ManipulationActionSpace(env, actions_ptr, bfs_heuristic) {
        action_type_ = std::make_shared<ManipulationType>(actions_ptr);
        moveit_interface_ = std::make_shared<MoveitInterface>(env);
        manipulation_type_ = std::make_shared<ManipulationType>(actions_ptr);

        // Get the joint limits.
        moveit_interface_->getJointLimits(joint_limits_);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    }

    /// @brief Visualize a state via its end effector pose in rviz for debugging
    /// @param pose
    void visualizePose(const geometry_msgs::Pose &pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
        marker.header.stamp = ros::Time();
        marker.ns = "graph";
        marker.id = vis_id_;
        marker.type = visualization_msgs::Marker::ARROW; // Other options are CUBE, SPHERE, CYLINDER, AXIS, etc.
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;

        marker.scale.x = 0.1; marker.scale.y = 0.01; marker.scale.z = 0.01;
        // green
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        marker.color.a = 0.5;

        // Lifetime.
        marker.lifetime = ros::Duration(5.0);

        // visualize
        vis_pub_.publish(marker);
        vis_id_++;
    }


    bool generateRandomState(const StateType& s1,
                             const StateType& s2,
                             double dist,
                             StateType& random_state) override {
        // bernoulli distribution
        std::random_device rd0;
        std::mt19937 gen0(rd0());
        std::bernoulli_distribution dis0(0.5);
        if (dis0(gen0)){
            return ActionSpaceMGS::generateRandomState(s1, s2, dist, random_state);
        }
//        return ActionSpaceMGS::generateRandomState(s1, s2, dist, random_state);
        //// TODO: This is working less well then the action_space_msg implementation
        // sample random x, y in the map (env_->map_size)
        if (s1.size() != moveit_interface_->num_joints_ || s2.size() != moveit_interface_->num_joints_){
            ROS_ERROR_STREAM("Error: state size is not 2!" << std::endl) ;
            return false;
        }
        // maximum attempts to generate a random state
        int max_attempts = 100;
        int attempts = 0;
        while (attempts < max_attempts){
            // get the random state
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            random_state.resize(moveit_interface_->num_joints_);
            // generate random state based on joint_limits
            for (int i = 0; i < moveit_interface_->num_joints_; i++){
                random_state[i] = dis(gen) * (joint_limits_[i].second - joint_limits_[i].first) + joint_limits_[i].first;
            }

            if (isStateValid(random_state)){
                return true;
            } else {
                random_state.clear();
                attempts++;
            }
        }
        return false;
    }


    bool connectStates(int s1, int s2, std::vector<int> &path, std::vector<double> &costs) override {
        auto s1_state = getRobotState(s1);
        auto s2_state = getRobotState(s2);
        PathType path_vals = interpolatePath(s1_state->state, s2_state->state);
        if (!isPathValid(path_vals)) {
            return false;
        } else {
            path.push_back(s2);
            costs.push_back(1000);
            return true;
        }
    }
};

class MobileManipulationMGSActionSpace : public ActionSpaceMGS,
                                         public MobileManipulationActionSpace {

public:

    MobileManipulationMGSActionSpace(const MoveitInterface &env,
                                     const MobileManipulationType &actions_ptr,
                                     BFSHeuristic *bfs_heuristic = nullptr) :
    ActionSpaceMGS(), MobileManipulationActionSpace(env, actions_ptr, bfs_heuristic) {
        action_type_ = mobile_manipulation_type_;
    }

        /// @brief Visualize a state via its end effector pose in rviz for debugging
    /// @param pose
    void visualizePose(const geometry_msgs::Pose &pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
        marker.header.stamp = ros::Time();
        marker.ns = "graph";
        marker.id = vis_id_;
        marker.type = visualization_msgs::Marker::ARROW; // Other options are CUBE, SPHERE, CYLINDER, AXIS, etc.
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;

        marker.scale.x = 0.1; marker.scale.y = 0.01; marker.scale.z = 0.01;
        // green
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        marker.color.a = 0.5;

        // Lifetime.
        marker.lifetime = ros::Duration(5.0);

        // visualize
        vis_pub_.publish(marker);
        vis_id_++;
    }


    bool generateRandomState(const StateType& s1,
                             const StateType& s2,
                             double dist,
                             StateType& random_state) override {
        // bernoulli distribution
        std::random_device rd0;
        std::mt19937 gen0(rd0());
        std::bernoulli_distribution dis0(0.5);
        if (dis0(gen0)){
            return ActionSpaceMGS::generateRandomState(s1, s2, dist, random_state);
        }
//        return ActionSpaceMGS::generateRandomState(s1, s2, dist, random_state);
        //// TODO: This is working less well then the action_space_msg implementation
        // sample random x, y in the map (env_->map_size)
        if (s1.size() != moveit_interface_->num_joints_ || s2.size() != moveit_interface_->num_joints_){
            ROS_ERROR_STREAM("Error: state size is not 2!" << std::endl) ;
            return false;
        }
        // maximum attempts to generate a random state
        int max_attempts = 100;
        int attempts = 0;
        while (attempts < max_attempts){
            // get the random state
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            random_state.resize(moveit_interface_->num_joints_);
            // generate random state based on joint_limits
            for (int i = 0; i < moveit_interface_->num_joints_; i++){
                random_state[i] = dis(gen) * (joint_limits_[i].second - joint_limits_[i].first) + joint_limits_[i].first;
            }

            if (isStateValid(random_state)){
                return true;
            } else {
                random_state.clear();
                attempts++;
            }
        }
        return false;
    }


    bool connectStates(int s1, int s2, std::vector<int> &path, std::vector<double> &costs) override {
        auto s1_state = getRobotState(s1);
        auto s2_state = getRobotState(s2);
        PathType path_vals = interpolatePath(s1_state->state, s2_state->state);
        if (!isPathValid(path_vals)) {
            return false;
        } else {
            path.push_back(s2);
            costs.push_back(1000);
            return true;
        }
    }
};

}  // namespace ims

