/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \date   4/3/23
 */

#ifndef MANIPULATIONACTIONSPACE_HPP
#define MANIPULATIONACTIONSPACE_HPP

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
// #include <search/action_space/action_space.hpp>
#include "action_types.hpp"

#include "manipulation_planning/common/moveit_scene_interface.hpp"
#include "manipulation_planning/common/utils.hpp"
#include "manipulation_planning/heuristics/manip_heuristics.hpp"

namespace ims {

/// @class ManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
class ManipulationActionSpace : virtual public ActionSpace {
protected:
    /// @brief Manipulation type
    std::shared_ptr<ManipulationType> manipulation_type_;
    /// @brief Moveit interface
    std::shared_ptr<MoveitInterface> moveit_interface_;
    /// @brief joint limits
    std::vector<std::pair<double, double>> joint_limits_;
    /// @brief The BFS heuristic
    BFSHeuristic *bfs_heuristic_;

    // TODO: delete: temp
    int vis_id_ = 0;
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;

public:
    /// @brief Constructor
    /// @param env
    /// @param actions_ptr
    /// @param bfs_heuristic
    ManipulationActionSpace(const MoveitInterface &env,
                            const ManipulationType &actions_ptr,
                            BFSHeuristic *bfs_heuristic = nullptr) : ActionSpace(), bfs_heuristic_(bfs_heuristic) {
        moveit_interface_ = std::make_shared<MoveitInterface>(env);
        manipulation_type_ = std::make_shared<ManipulationType>(actions_ptr);
        // get the joint limits
        moveit_interface_->getJointLimits(joint_limits_);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &actions_seq,
                    bool check_validity) override {
        auto curr_state = this->getRobotState(state_id);
        auto curr_state_val = curr_state->state;
        if (bfs_heuristic_ == nullptr) {
            auto actions = manipulation_type_->getPrimActions();
            for (int i{0}; i < actions.size(); i++) {
                auto action = actions[i];
                ActionSequence action_seq{curr_state_val};
                // push back the new state after the action
                StateType next_state_val(curr_state_val.size());
                std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(), std::plus<>());
                action_seq.push_back(next_state_val);
                actions_seq.push_back(action_seq);
            }
        }
        else {
            if (curr_state->state_mapped.empty()) {
                moveit_interface_->calculateFK(curr_state_val, curr_state->state_mapped);
                this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
            }
            auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                                   curr_state->state_mapped.at(1),
                                                                   curr_state->state_mapped.at(2));
            auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                     curr_state->state_mapped.at(1),
                                                                     curr_state->state_mapped.at(2));
            auto actions = manipulation_type_->getAdaptiveActions(start_dist, goal_dist);

            for (int i{0}; i < actions.size(); i++) {
                auto action = actions[i];
                ActionSequence action_seq{curr_state_val};
                // if the action is snap, then the next state is the goal state
                // TODO: Add the option to have a goal state defined in ws even if planning in conf space
                if (action[0] == INF_DOUBLE) {
//                    action_seq.push_back(bfs_heuristic_->goal_);  // TODO: It is wierd that I am using the heuristic here
                    action_seq.push_back(states_.at(1)->state);  // TODO: It is wierd that I am using the heuristic here
                }
                else {
                    // push back the new state after the action
                    StateType next_state_val(curr_state_val.size());
                    std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(), std::plus<>());
                    action_seq.push_back(next_state_val);
                }
                actions_seq.push_back(action_seq);
            }
        }
    }

    /// @brief Set the manipulation space type
    /// @param SpaceType The manipulation type
    void setManipActionType(ManipulationType::SpaceType SpaceType) const {
        manipulation_type_->setSpaceType(SpaceType);
    }

    ManipulationType::SpaceType getManipActionType() const {
        return manipulation_type_->getSpaceType();
    }

    /// @brief Get current joint states
    /// @param joint_states The joint states
    void getCurrJointStates(StateType &joint_states) const {
        auto joints = moveit_interface_->planning_scene_->getCurrentState();
        joints.copyJointGroupPositions(moveit_interface_->group_name_,
                                       joint_states);
    }

    /// @brief Get the workspace state
    /// @param ws_state The workspace state
    void getCurrWorkspaceState(StateType &ws_state) const {
        // get the tip link name
        auto tip_link = moveit_interface_->planning_scene_->getRobotModel()->getJointModelGroup(moveit_interface_->group_name_)->getLinkModelNames().back();
        // get the end-effector pose
        auto ee_pose = moveit_interface_->planning_scene_->getCurrentState().getGlobalLinkTransform(tip_link);
        // get the euler angles
        ws_state.resize(6);
        ws_state[0] = ee_pose.translation().x();
        ws_state[1] = ee_pose.translation().y();
        ws_state[2] = ee_pose.translation().z();
        Eigen::Vector3d euler_angles = ee_pose.rotation().eulerAngles(2, 1, 0);
        ws_state[3] = euler_angles[2];
        ws_state[4] = euler_angles[1];
        ws_state[5] = euler_angles[0];
        normalize_euler_zyx(ws_state[5], ws_state[4], ws_state[3]);
        roundStateToDiscretization(ws_state, manipulation_type_->state_discretization_);
    }

    /// @brief Get the end effector pose in the robot frame.
    /// @param ee_pose The end effector pose
    void calculateFK(const StateType &state, StateType &ee_pose) const {
        moveit_interface_->calculateFK(state, ee_pose);
    }

    bool isStateValid(const StateType &state_val) override {
        // check if the state is valid
        switch (manipulation_type_->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return moveit_interface_->isStateValid(state_val);
            case ManipulationType::SpaceType::WorkSpace:
                // check if state exists with IK solution already
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0];
                pose.position.y = state_val[1];
                pose.position.z = state_val[2];
                // Euler angles to quaternion
                Eigen::Quaterniond q;
                from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                StateType joint_state;
                bool succ = moveit_interface_->calculateIK(pose, joint_state);
                if (!succ) {
                    return false;
                }
                else {
                    return moveit_interface_->isStateValid(joint_state);
                }
        }
        return false;
    }

    /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
    /// @param state_val The state to check
    /// @param joint_state The ik solution
    /// @return True if the state is valid, false otherwise
    bool isStateValid(const StateType &state_val,
                      StateType &joint_state) const {
        switch (manipulation_type_->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return moveit_interface_->isStateValid(state_val);
            case ManipulationType::SpaceType::WorkSpace:
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0];
                pose.position.y = state_val[1];
                pose.position.z = state_val[2];
                // Euler angles to quaternion
                Eigen::Quaterniond q;
                from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                bool succ = moveit_interface_->calculateIK(pose, joint_state);
                if (!succ) {
                    ROS_INFO("IK failed");
                    return false;
                }
                else {
                    return moveit_interface_->isStateValid(joint_state);
                }
        }
        return false;
    }

    bool isStateValid(const StateType &state_val,
                      const StateType &seed,
                      StateType &joint_state) const {
        // check if the state is valid
        switch (manipulation_type_->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return moveit_interface_->isStateValid(state_val);
            case ManipulationType::SpaceType::WorkSpace:
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0];
                pose.position.y = state_val[1];
                pose.position.z = state_val[2];
                // Euler angles to quaternion
                Eigen::Quaterniond q;
                from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                joint_state.resize(moveit_interface_->num_joints_);
                bool succ = moveit_interface_->calculateIK(pose, seed, joint_state);
                normalizeAngles(joint_state);
                if (!succ) {
                    return false;
                }
                else {
                    return moveit_interface_->isStateValid(joint_state);
                }
        }
        return false;
    }

    /// @brief Interpolate path between two states
    /// @param start The start state
    /// @param end The end state
    /// @param resolution The resolution of the path (default: 0.005 rad)
    /// @return The interpolated path
    static PathType interpolatePath(const StateType &start, const StateType &end,
                                    const double resolution = 0.01) {
        // TODO: Currently only works for configuration space
        assert(start.size() == end.size());
        PathType path;
        // get the maximum distance between the two states
        double max_distance{0.0};
        for (int i{0}; i < start.size(); i++) {
            double distance = std::abs(start[i] - end[i]);
            if (distance > max_distance) {
                max_distance = distance;
            }
        }
        // calculate the number of steps
        int steps = std::ceil(max_distance / resolution);
        // interpolate the path
        for (int i{0}; i < steps; i++) {
            StateType state;
            for (int j{0}; j < start.size(); j++) {
                state.push_back(start[j] + (end[j] - start[j]) * i / steps);
            }
            path.push_back(state);
        }
        return path;
    }

    bool isStateToStateValid(const StateType &start, const StateType &end) {
        PathType path = interpolatePath(start, end);
        return isPathValid(path);
    }

    bool isPathValid(const PathType &path) override {
        switch (manipulation_type_->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return moveit_interface_->isPathValid(path);
            case ManipulationType::SpaceType::WorkSpace:
                PathType poses;
                for (auto &state : path) {
                    geometry_msgs::Pose pose;
                    pose.position.x = state[0];
                    pose.position.y = state[1];
                    pose.position.z = state[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state[5], state[4], state[3], q);
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();
                    StateType joint_state;
                    bool succ = moveit_interface_->calculateIK(pose, joint_state);
                    if (!succ) {
                        return false;
                    }
                    else {
                        poses.push_back(joint_state);
                    }
                }
                return moveit_interface_->isPathValid(poses);
        }
        return false;
    }

    virtual bool getSuccessorsWs(int curr_state_ind,
                                 std::vector<int> &successors,
                                 std::vector<double> &costs) {
        // get the current state
        auto curr_state = this->getRobotState(curr_state_ind);
        auto curr_state_val = curr_state->state;
        // get the actions
        auto actions = manipulation_type_->getPrimActions();
        // convert to quaternion
        Eigen::Quaterniond q_curr;
        from_euler_zyx(curr_state_val[5], curr_state_val[4], curr_state_val[3], q_curr);
        // get the successors
        StateType new_state_val;
        for (auto action : actions) {
            new_state_val.clear();
            // create a new state in the length of the current state
            new_state_val.resize(curr_state_val.size());
            // increment the xyz coordinates
            for (int i{0}; i < 3; i++) {
                new_state_val[i] = curr_state_val[i] + action[i];
            }

            Eigen::Quaterniond q_action{action[6], action[3], action[4], action[5]};
            auto q_new = q_curr * q_action;

            // convert the quaternion to euler angles
            get_euler_zyx(q_new, new_state_val[5], new_state_val[4], new_state_val[3]);
            normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
            // discretize
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);

            //            if (isStateToStateValid(curr_state_val, new_state_val)) {
            bool succ;
            StateType mapped_state;
            if (curr_state->state_mapped.empty()) {
                //                    ROS_INFO("No mapped state, using IK without seed");
                succ = isStateValid(new_state_val,
                                    mapped_state);
            }
            else
                succ = isStateValid(new_state_val,
                                    curr_state->state_mapped,
                                    mapped_state);
            if (succ) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);
                auto new_state = this->getRobotState(next_state_ind);
                new_state->state_mapped = mapped_state;
                // add the state to the successors
                successors.push_back(next_state_ind);
                // add the cost
                double cost{0};
                for (int i{0}; i < 3; i++) {
                    cost += action[i] * action[i];
                }
                // add the cost of the rotation which is quaternion
                double r, p, y;
                get_euler_zyx(q_action, y, p, r);
                cost += r * r + p * p + y * y;
                costs.push_back(cost);
            }
        }
        return true;
    }

    virtual bool getSuccessorsCs(int curr_state_ind,
                                 std::vector<int> &successors,
                                 std::vector<double> &costs) {
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        // get the successors
        for (auto &action : actions) {
            // the first state is the current state and the last state is the successor
            const auto& curr_state_val = action.front();
            auto new_state_val = action.back();
            // normalize the angles
            normalizeAngles(new_state_val, joint_limits_);
            // discretize the state
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
            // check if the state went through discontinuity
            bool discontinuity{false};
            // check for maximum absolute action
            for (int i{0}; i < curr_state_val.size(); i++) {
                if (new_state_val[i] < joint_limits_[i].first || new_state_val[i] > joint_limits_[i].second) {
                    discontinuity = true;
                    break;
                }
            }

            if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val)) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);
                // add the state to the successors
                successors.push_back(next_state_ind);
                // add the cost
                costs.push_back(1000);
            }
        }
        return true;
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int> &successors,
                       std::vector<double> &costs) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCs(curr_state_ind, successors, costs);
        }
        else {
            return getSuccessorsWs(curr_state_ind, successors, costs);
        }
    }

    /// @brief Visualize a state point in rviz for debugging
    /// @param state_id The state id
    /// @param type The type of state (greedy, attractor, etc)
    void VisualizePoint(double x, double y, double z) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
        marker.header.stamp = ros::Time();
        marker.ns = "graph";
        marker.id = vis_id_;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        // green
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        // Lifetime.
        marker.lifetime = ros::Duration(5.0);

        // visualize
        vis_pub_.publish(marker);
        vis_id_++;
    }

    /// @brief Get the scene interface.
    /// @return The scene interface
    std::shared_ptr<MoveitInterface> getSceneInterface() {
        return moveit_interface_;
    }
};
}  // namespace ims

#endif  // MANIPULATIONACTIONSPACE_HPP
