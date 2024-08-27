/*
 * Copyright (C) 2024, Hanlan Yang
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
 * \file   manipulation_edge_action_space.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   8/12/24
 */
#pragma once

// include standard libraries
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <utility>
#include <vector>

// include ROS libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
// include tf to convert euler angles to quaternions
#include <tf/transform_datatypes.h>

// project includes
#include <search/action_space/edge_action_space.hpp>

#include "manipulation_planning/action_space/manipulation_action_space.hpp"
#include "manipulation_planning/common/moveit_scene_interface.hpp"
#include "manipulation_planning/common/utils.hpp"
#include "manipulation_planning/heuristics/manip_heuristics.hpp"

namespace ims {

/// @class ManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
class ManipulationEdgeActionSpace : public EdgeActionSpace {
protected:
    /// @brief Manipulation type
    std::shared_ptr<ManipulationType> manipulation_type_;
    /// @brief Moveit interface
    std::shared_ptr<MoveitInterface> moveit_interface_;
    /// @brief joint limits
    std::vector<std::pair<double, double>> joint_limits_;
    /// @brief The BFS heuristic
    std::shared_ptr<BFSHeuristic> bfs_heuristic_;

    // TODO: delete: temp
    int vis_id_ = 0;
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;

    // Debug
    bool vis_ = false;

public:
    /// @brief Constructor
    /// @param moveitInterface The moveit interface
    /// @param ManipulationType The manipulation type
    ManipulationEdgeActionSpace(const MoveitInterface &env,
                                const ManipulationType &actions_ptr,
                                // std::shared_ptr<BFSHeuristic> q_heuristic = nullptr,
                                std::shared_ptr<BFSHeuristic> bfs_heuristic = nullptr) : EdgeActionSpace(bfs_heuristic), bfs_heuristic_(bfs_heuristic) {
        moveit_interface_ = std::make_shared<MoveitInterface>(env);
        manipulation_type_ = std::make_shared<ManipulationType>(actions_ptr);
        // get the joint limits
        moveit_interface_->getJointLimits(joint_limits_);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    }

    void getActionSequences(int state_id,
                            std::vector<ActionSequence> &action_seqs,
                            std::vector<std::vector<double>> &action_transition_costs,
                            bool check_validity) {
        auto curr_state = this->getRobotState(state_id);
        auto curr_state_val = curr_state->state;
        if (bfs_heuristic_ == nullptr) {
            std::vector<ActionSequence> prim_actions_seqs;
            std::vector<std::vector<double>> prim_action_transition_costs;
            manipulation_type_->getPrimActions(prim_actions_seqs, prim_action_transition_costs);
            for (int i{0}; i < prim_actions_seqs.size(); i++) {
                auto prim_action_seq = prim_actions_seqs[i];
                // Create the transformed action sequence. The action costs do not change.
                ActionSequence transformed_action_seq;
                std::vector<double> transformed_action_transition_costs = prim_action_transition_costs[i];
                for (int i = 0; i < prim_action_seq.size(); i++) {
                    const Action &prim_action = prim_action_seq[i];
                    StateType next_state_val(curr_state_val.size());
                    std::transform(curr_state_val.begin(), curr_state_val.end(), prim_action.begin(), next_state_val.begin(),
                                   std::plus<>());
                    transformed_action_seq.push_back(next_state_val);
                }
                action_seqs.push_back(transformed_action_seq);
                action_transition_costs.push_back(transformed_action_transition_costs);
            }
        } else {
            if (curr_state->state_mapped.empty()) {
                moveit_interface_->calculateFK(curr_state_val, curr_state->state_mapped);
            }
            // if (vis_) {
            //     moveit_interface_->calculateFK(curr_state_val, curr_state->state_mapped);
            //     this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
            // }
            auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                                   curr_state->state_mapped.at(1),
                                                                   curr_state->state_mapped.at(2));
            auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                     curr_state->state_mapped.at(1),
                                                                     curr_state->state_mapped.at(2));
            std::vector<ActionSequence> prim_action_seqs;
            std::vector<std::vector<double>> prim_action_transition_costs;
            manipulation_type_->getAdaptivePrimActionSequences(start_dist,
                                                               goal_dist,
                                                               prim_action_seqs,
                                                               prim_action_transition_costs,
                                                               true);

            for (int i{0}; i < prim_action_seqs.size(); i++) {
                auto prim_action_seq = prim_action_seqs[i];
                ActionSequence transformed_action_seq;
                std::vector<double> transformed_action_transition_costs = prim_action_transition_costs[i];
                // if the action is snap, then the next state is the goal state
                // TODO: Add the option to have a goal state defined in ws even if planning in conf space
                if (prim_action_seq.back()[0] == INF_DOUBLE) {
                    transformed_action_seq = {curr_state_val, bfs_heuristic_->goal_};  // TODO: It is wierd that I am using the heuristic here
                    transformed_action_transition_costs = {1.0, 0.0};
                } else {
                    for (int i = 0; i < prim_action_seq.size(); i++) {
                        const Action &prim_action = prim_action_seq[i];
                        StateType next_state_val(curr_state_val.size());
                        std::transform(curr_state_val.begin(), curr_state_val.end(), prim_action.begin(), next_state_val.begin(),
                                       std::plus<>());
                        transformed_action_seq.push_back(next_state_val);
                    }
                }
                action_seqs.push_back(transformed_action_seq);
                action_transition_costs.push_back(transformed_action_transition_costs);
            }
        }
    }
    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        std::vector<std::vector<double>> action_transition_costs;
        getActionSequences(state_id, action_seqs, action_transition_costs, check_validity);
    }

    /// @brief Set the manipulation space type
    /// @param SpaceType The manipulation type
    void setManipActionType(ManipulationType::SpaceType SpaceType) {
        manipulation_type_->setSpaceType(SpaceType);
    }

    ManipulationType::SpaceType getManipActionType() {
        return manipulation_type_->getSpaceType();
    }

    /// @brief Get current joint states
    /// @param joint_states The joint states
    void getCurrJointStates(StateType &joint_states) {
        auto joints = moveit_interface_->planning_scene_->getCurrentState();
        joints.copyJointGroupPositions(moveit_interface_->group_name_,
                                       joint_states);
    }

    /// @brief Get the workspace state
    /// @param ws_state The workspace state
    void getCurrWorkspaceState(StateType &ws_state) {
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
    void calculateFK(const StateType &state, StateType &ee_pose) {
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
                } else {
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
                      StateType &joint_state) {
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
                } else {
                    return moveit_interface_->isStateValid(joint_state);
                }
        }
        return false;
    }

    bool isStateValid(const StateType &state_val,
                      const StateType &seed,
                      StateType &joint_state) {
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
                } else {
                    return moveit_interface_->isStateValid(joint_state);
                }
        }
        return false;
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
                    } else {
                        poses.push_back(joint_state);
                    }
                }
                return moveit_interface_->isPathValid(poses);
        }
        return false;
    }

    virtual bool getSuccessorWs(int curr_state_ind,
                                ActionSequence &prim_action_seq,
                                std::vector<int> &seq_state_ids,
                                std::vector<double> &seq_transition_costs) {
        // Get the current state.
        auto curr_state = this->getRobotState(curr_state_ind);
        StateType curr_state_val = curr_state->state;
        Eigen::Quaterniond q_curr;
        from_euler_zyx(curr_state_val[5], curr_state_val[4], curr_state_val[3], q_curr);

        // The first state is the current state and the last state is the successor.
        // The primitive action sequences are of for xyz-xyzw. Those start from the origin.
        // The first state is assumed to be valid.
        // Go through all the states in the sequence, compute new quaternion, normalize, discretize, and check for validity.
        for (size_t j{1}; j < prim_action_seq.size(); j++) {
            Eigen::Quaterniond q_action{prim_action_seq[j][6], prim_action_seq[j][3], prim_action_seq[j][4], prim_action_seq[j][5]};
            Eigen::Quaterniond q_new = q_curr * q_action;
            // convert the quaternion to euler angles
            double r, p, y;
            get_euler_zyx(q_new, y, p, r);
            // Create a new state. Now it is in xyzrpy.
            StateType new_state_val = {curr_state_val[0] + prim_action_seq[j][0],  // Add the action on x.
                                       curr_state_val[1] + prim_action_seq[j][1],  // Add the action on y.
                                       curr_state_val[2] + prim_action_seq[j][2],  // Add the action on z.
                                       r, p, y};
            // Normalize the angles.
            normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
            // Discretize the state.
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
            // Check if the next state is valid.
            bool succ;
            StateType mapped_state;
            if (curr_state->state_mapped.empty()) {
                succ = isStateValid(new_state_val,
                                    mapped_state);
            } else
                succ = isStateValid(new_state_val,
                                    curr_state->state_mapped,
                                    mapped_state);
            if (succ) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);
                auto new_state = this->getRobotState(next_state_ind);
                new_state->state_mapped = mapped_state;
                // Add the state to the successor sequence.
                seq_state_ids.push_back(next_state_ind);
                // Add the cost. This overrides the value given from the primitive.
                double cost{0};
                for (int dim{0}; dim < 3; dim++) {
                    cost += prim_action_seq[j][dim] * prim_action_seq[j][dim];
                }
                // Add the cost of the rotation which is quaternion
                double r, p, y;
                get_euler_zyx(q_action, y, p, r);
                cost += r * r + p * p + y * y;
                seq_transition_costs[j - 1] = cost;  // TODO(yoraish): probably a cleaner way to do this.
            } else {
                // If the state is not valid, break the loop.
                return false;
            }
        }
        return true;
    }

    virtual bool getSuccessorsWs(int curr_state_ind,
                                 std::vector<std::vector<int>> &seqs_state_ids,
                                 std::vector<std::vector<double>> &seqs_transition_costs) {
        seqs_state_ids.clear();
        seqs_transition_costs.clear();
        // Get the primitive actions. Those will be "added" to the current state.
        std::vector<ActionSequence> prim_action_seqs;
        std::vector<std::vector<double>> prim_action_transition_costs;
        manipulation_type_->getPrimActions(prim_action_seqs, prim_action_transition_costs);

        // Get the successors. Each successor is a sequence of state_ids and a sequence of transition costs.
        for (size_t i{0}; i < prim_action_seqs.size(); i++) {
            ActionSequence &prim_action_seq = prim_action_seqs[i];
            // Objects for the successor resulting from this action.
            std::vector<int> successor_seq_state_ids{curr_state_ind};
            std::vector<double> successor_seq_transition_costs = prim_action_transition_costs[i];

            // Only if the getSuccessorWs return true, add the successor_seq to the list.
            // However, if a paritial action is desired, we need to add the partial successor_seq to the list.
            // Here we are assuming that we only want the full action success cases to be returned.
            if (getSuccessorWs(curr_state_ind, prim_action_seq, successor_seq_state_ids, successor_seq_transition_costs)) {
                seqs_state_ids.push_back(successor_seq_state_ids);
                seqs_transition_costs.push_back(successor_seq_transition_costs);
            }
        }
        if (seqs_state_ids.empty()) {
            return false;
        }
        return true;
    }

    virtual bool getSuccessorCs(int curr_state_ind,
                                ActionSequence &action_seq,
                                std::vector<int> &seq_state_ids,
                                std::vector<double> &seq_transition_costs) {
        // The first state is the current state and the last state is the successor.
        // Go through all the states in the sequence, normalize angles, discretize, and check for validity.
        // Normalize and discretize the first state and then go through all pairs [i, i+1].
        // The first state is assumed to be valid.
        normalizeAngles(action_seq.front(), joint_limits_);
        roundStateToDiscretization(action_seq.front(), manipulation_type_->state_discretization_);
        if (vis_) {
            auto curr_state = this->getRobotState(curr_state_ind);
            auto curr_state_val = curr_state->state;
            moveit_interface_->calculateFK(curr_state_val, curr_state->state_mapped);
            this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
        }
        for (size_t j{0}; j < action_seq.size() - 1; j++) {
            auto curr_state_val = action_seq[j];
            auto new_state_val = action_seq[j + 1];
            // Normalize the angles.
            normalizeAngles(new_state_val, joint_limits_);
            // Discretize the state.
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
            // Check if the state transition went through discontinuity.
            bool discontinuity{false};
            // check for maximum absolute action
            for (int dim{0}; dim < curr_state_val.size(); dim++) {
                if (new_state_val[dim] < joint_limits_[dim].first || new_state_val[dim] > joint_limits_[dim].second) {
                    discontinuity = true;
                    break;
                }
            }

            if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val)) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);
                // add the state to the successors
                seq_state_ids.push_back(next_state_ind);
                // Transition costs were already added before.
            } else {
                return false;
            }
        }
        return true;
    }

    virtual bool getSuccessorsCs(int curr_state_ind,
                                 std::vector<std::vector<int>> &seqs_state_ids,
                                 std::vector<std::vector<double>> &seqs_transition_costs) {
        seqs_state_ids.clear();
        seqs_transition_costs.clear();

        std::vector<ActionSequence> action_seqs;
        std::vector<std::vector<double>> action_transition_costs;
        getActionSequences(curr_state_ind, action_seqs, action_transition_costs, false);
        // Get the successors. Each successor is a sequence of state_ids and a sequence of transition costs.
        for (size_t i{0}; i < action_seqs.size(); i++) {
            ActionSequence &action_seq = action_seqs[i];
            // Objects for the successor resulting from this action.
            std::vector<int> successor_seq_state_ids{curr_state_ind};
            std::vector<double> successor_seq_transition_costs = action_transition_costs[i];
            if (getSuccessorCs(curr_state_ind, action_seq, successor_seq_state_ids, successor_seq_transition_costs)) {
                seqs_state_ids.push_back(successor_seq_state_ids);
                seqs_transition_costs.push_back(successor_seq_transition_costs);

            } else {
                seqs_state_ids.push_back(std::vector<int>{});
                seqs_transition_costs.push_back(std::vector<double>{});
            }
        }
        if (seqs_state_ids.empty()) {
            return false;
        }
        return true;
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<std::vector<int>> &seqs_state_ids,
                       std::vector<std::vector<double>> &seqs_transition_costs) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCs(curr_state_ind, seqs_state_ids, seqs_transition_costs);
        } else if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::WorkSpace) {
            return getSuccessorsWs(curr_state_ind, seqs_state_ids, seqs_transition_costs);
        } else {
            throw std::runtime_error("Space type not supported.");
        }
    }

    bool getSuccessorProxy(int curr_edge_ind, StateType &next_state_val) override {
        // Get the seq transition cost for the specific action of the edge
        auto curr_edge = getRobotEdge(curr_edge_ind);
        int curr_state_ind = getRobotStateId(curr_edge->state);
        ActionSequence action_seq = curr_edge->action;
        std::vector<double> seq_transition_costs = curr_edge->action_cost;

        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            // The first state is the current state and the last state is the successor.
            // Go through all the states in the sequence, normalize angles, discretize, and check for validity.
            // Normalize and discretize the first state and then go through all pairs [i, i+1].
            // The first state is assumed to be valid.
            normalizeAngles(action_seq.front(), joint_limits_);
            roundStateToDiscretization(action_seq.front(), manipulation_type_->state_discretization_);

            auto curr_state_val = action_seq[0];
            auto next_state_val = action_seq.back();
            // Normalize the angles.
            normalizeAngles(next_state_val, joint_limits_);
            // Discretize the state.
            roundStateToDiscretization(next_state_val, manipulation_type_->state_discretization_);
            // Check if the state transition went through discontinuity.
            bool discontinuity{false};
            // check for maximum absolute action
            for (int dim{0}; dim < curr_state_val.size(); dim++) {
                if (next_state_val[dim] < joint_limits_[dim].first || next_state_val[dim] > joint_limits_[dim].second) {
                    discontinuity = true;
                    return false;
                }
            }
            return true;
        } else if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::WorkSpace) {
            throw std::runtime_error("Space type (WorkSpace) not yet supported.");
        } else {
            throw std::runtime_error("Space type not supported.");
        }
    }

    bool getSuccessor(int curr_edge_ind,
                      std::vector<int> &seq_state_ids,
                      std::vector<double> &seq_transition_costs) override {
        // Get the seq transition cost for the specific action of the edge
        auto curr_edge = getRobotEdge(curr_edge_ind);
        int curr_state_ind = getRobotStateId(curr_edge->state);
        ActionSequence action_seq = curr_edge->action;
        seq_transition_costs = curr_edge->action_cost;

        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorCs(curr_state_ind, action_seq, seq_state_ids, seq_transition_costs);
        } else if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::WorkSpace) {
            return getSuccessorWs(curr_state_ind, action_seq, seq_state_ids, seq_transition_costs);
        } else {
            throw std::runtime_error("Space type not supported.");
        }
    }

    void setVisualization(bool vis) {
        vis_ = vis;
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
        ros::Duration(0.01).sleep();
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
