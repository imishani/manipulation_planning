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
#pragma once

// include standard libraries
#include <iostream>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

// include ROS libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
// include tf to convert euler angles to quaternions
#include <tf/transform_datatypes.h>

// project includes
#include <search/action_space/action_space.hpp>

#include "manipulation_planning/common/moveit_scene_interface.hpp"
#include "manipulation_planning/common/utils.hpp"
#include "manipulation_planning/heuristics/manip_heuristics.hpp"

namespace ims {

struct ManipulationType : ActionType {
    /// @brief Constructor
    /// @param[in] bfs_heuristic A pointer to a BFSHeuristic object. Default = nullptr
    explicit ManipulationType() : action_type_(ActionType::MOVE),
                                  space_type_(SpaceType::ConfigurationSpace),
                                  mprim_file_name_("../config/manip_6dof.mprim"),
                                  max_action_(0.0){
                                  };

    /// @brief Constructor with motion primitives file given
    /// @param[in] mprim_file The path to the motion primitives file
    /// @param[in] bfs_heuristic A pointer to a BFSHeuristic object
    explicit ManipulationType(std::string mprim_file) : action_type_(ActionType::MOVE),
                                                        space_type_(SpaceType::ConfigurationSpace),
                                                        mprim_file_name_(std::move(mprim_file)),
                                                        max_action_(0.0){
                                                        };

    /// @brief Constructor with adaptive motion primitives given
    /// @brief Destructor
    ~ManipulationType() override = default;

    /// @brief The type of the action
    enum class ActionType {
        MOVE,
        GRASP,
        RELEASE
    };

    enum class SpaceType {
        ConfigurationSpace,
        WorkSpace
    };
    /// @{ getters and setters
    /// @brief Get the action type
    /// @return The action type
    ActionType getActionType() const {
        return action_type_;
    }

    /// @brief Set the action type
    /// @param ActionType The action type
    void setActionType(ActionType ActionType) {
        action_type_ = ActionType;
    }

    /// @brief Get the space type
    /// @return The space type
    SpaceType getSpaceType() const {
        return space_type_;
    }

    /// @brief Set the space type
    /// @param SpaceType The space type
    void setSpaceType(SpaceType SpaceType) {
        space_type_ = SpaceType;
    }
    /// @}

    void Discretization(StateType &state_des) override {
        state_discretization_ = state_des;
    }

    void loadMprimFamily(const YAML::Node& root_node,
                         const std::string& family_name,
                         std::vector<ActionSequence>& action_seqs,
                         std::vector<std::vector<double>>& action_transition_times) {
    if (root_node[family_name]) {
        for (const auto& action : root_node[family_name]) {
            std::string action_name = action.first.as<std::string>();
            // Check if "delta_degrees" exists in the key list.
            // The structure would be like this:
            //          action_name:
            //            delta_degrees:
            //              - [ 0, 0, 0, 0, 0, 0, 0 ]
            //              - [ 7, 0, 0, 0, 0, 0, 0 ]
            //            delta_time_steps: [ 1, 0 ]
            //            generate_negative: true
            if (!action.second["delta_degrees"]) {
                std::cerr << "Action \"" << action_name << "\" does not have \"delta_degrees\".\n";
                throw std::runtime_error("Action \"" + action_name + "\" does not have \"delta_degrees\".");
            }
            // Check if required to also generate negated actions.
            bool is_generate_negated_action = false;
            if (action.second["generate_negative"]) {
                is_generate_negated_action = action.second["generate_negative"].as<bool>();
            }
            ActionSequence action_seq;
            ActionSequence neg_action_seq;
            std::vector<std::vector<double>> action_transition_cost;
            for (const auto& action_seq_state_it : action.second["delta_degrees"]) {
                StateType state;
                StateType neg_state;
                for (const auto& num : action_seq_state_it) {
                    double num_rads = num.as<double>() / 180.0 * M_PI;
                    // Populate max_action_.
                    if (abs(num_rads) > max_action_) {
                        max_action_ = abs(num_rads);
                    }
                    state.push_back(num_rads);
                    neg_state.push_back(-num_rads);
                }
                action_seq.push_back(state);
                if (is_generate_negated_action) {
                    neg_action_seq.push_back(neg_state);
                }
            }
            // Note that we do not add the action_seq and the neg_action_seq to the action_seqs. We only do it at the end as those may need to be modified.
            // Get the transition times, if those are available.
            // This does two things. First, it stores the transition times for each action sequence:
            if (action.second["delta_time_steps"]) {
                std::vector<double> transition_time; // This is the same for both the normal and negated action.
                for (size_t i{0}; i < action.second["delta_time_steps"].size(); i++) {
                    double num = action.second["delta_time_steps"][i].as<TimeType>();
                    // Store the time interval of the step in the transition time.
                    transition_time.push_back(num);
                }
                assert(transition_time.size() == action_seq.size());

                // Second, it adds a time dimension to the action sequence. Time 0 for the first, 1 for the second, etc.
                // If we have time information, then modify the action sequence to include the time in the last element.
                // Add the time 0 to the first state in the sequence.
                action_seq[0].push_back(0.0);
                if (is_generate_negated_action) {
                    neg_action_seq[0].push_back(0.0);
                }

                for (size_t i{0}; i < action_seq.size() - 1; i++) {
                    action_seq[i + 1].push_back(action_seq[i].back() + transition_time[i]);
                    if (is_generate_negated_action) {
                        neg_action_seq[i + 1].push_back(neg_action_seq[i].back() + transition_time[i]);
                    }
                }

                // Save the transition times for the action sequence.
                action_transition_times.push_back(transition_time);
                if (is_generate_negated_action) {
                    action_transition_times.push_back(transition_time);
                }
            }

            // Add the action sequence to the list of action sequences.
            action_seqs.push_back(action_seq);
            if (is_generate_negated_action) {
                action_seqs.push_back(neg_action_seq);
            }
        }

        /*// PRINT PRINT PRINT.
        std::cout << GREEN << "Created actions from YAML file and family name: " << family_name << RESET << std::endl;
        for (int i = 0; i < action_seqs.size(); i++) {
            std::cout << "Action Sequence " << i << ": " << std::endl;
            for (int j = 0; j < action_seqs[i].size(); j++) {
                std::cout << " * State: " << action_seqs[i][j] << " ";
                std::cout << " * Time to next: " << action_transition_times[i][j] << std::endl;
            }
            std::cout << std::endl;
        }
        // END PRINT PRINT PRINT. */

    } else {
        std::cerr << "Family \"" << family_name << "\" not found in the YAML file.\n";
    }
    // Check that the length of the transition times is the same as the length of the action sequence.
    if (action_seqs.size() != action_transition_times.size()) {
        std::cerr << "The number of action sequences and the number of transition times do not match.\n";
        throw std::runtime_error("The number of action sequences and the number of transition times do not match.");
    }
}

    void readMPfile(){
        // Read the motion primitive YAML file.
        YAML::Node mprim_config = YAML::LoadFile(mprim_file_name_);
        // The motion primitive file defines families of actions. In this action space we care about the
        // 1. short distance motion primitives (short_mprim_) and
        // 2. long distance motion primitives (long_mprim_).
        if (space_type_ == SpaceType::ConfigurationSpace) {
            std::vector <ActionSequence> short_mprim_seqs; // This will become a member variable.
            std::vector <std::vector<double>> short_mprim_transition_costs; // This will become a member variable.
            loadMprimFamily(mprim_config, "short_primitives", short_mprim_seqs, short_mprim_transition_costs);
            std::vector <ActionSequence> long_mprim_seqs; // This will become a member variable.
            std::vector <std::vector<double>> long_mprim_transition_costs; // This will become a member variable.
            loadMprimFamily(mprim_config, "long_primitives", long_mprim_seqs, long_mprim_transition_costs);

            // TEST TEST TEST.
            // Populate short_mprim_ and long_mprim_ from the YAML file.
            for (size_t i{0}; i < short_mprim_seqs.size(); i++) {
                Action action_tip = short_mprim_seqs[i].back();
                short_mprim_.push_back(action_tip);
            }
            for (size_t i{0}; i < long_mprim_seqs.size(); i++) {
                Action action_tip = long_mprim_seqs[i].back();
                long_mprim_.push_back(action_tip);
            }
            // END TEST TEST TEST.
        }
        else if (space_type_ == SpaceType::WorkSpace) {
            std::cerr << "Work space motion primitives are not implemented yet.\n";
            throw std::runtime_error("Work space motion primitives are not implemented yet.");
        }
    }

    std::vector<ActionSequence> getPrimActionEdges() override {
        // TODO.
        return {};
    }

    /// @brief Get the possible actions
    /// @return A vector of all possible actions
    std::vector<Action> getPrimActions() override {
        // Read the motion primitive file if needed.
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        // Populate the actions_ vector if not done so already.
        if (actions_.empty()) {
            switch (action_type_) {
                case ActionType::MOVE:
                    switch (space_type_) {
                        case SpaceType::ConfigurationSpace: {
                            // TODO: Add snap option
                            actions_.insert(actions_.end(), long_mprim_.begin(), long_mprim_.end());
                            actions_.insert(actions_.end(), short_mprim_.begin(), short_mprim_.end());
                        } break;
                        case SpaceType::WorkSpace: {
                            std::vector<std::vector<double>> mprim;
                            mprim.insert(mprim.end(), short_mprim_.begin(), short_mprim_.end());
                            for (auto &action_ : mprim) {
                                throw std::runtime_error ("Work space motion primitives are not implemented yet. Handle reverse actions in the readmpfile function.");
                                // make an inverted action
                                Action inverted_action(action_.size());
                                inverted_action[0] = -action_[0];
                                inverted_action[1] = -action_[1];
                                inverted_action[2] = -action_[2];
                                inverted_action[3] = -action_[3];
                                inverted_action[4] = -action_[4];
                                inverted_action[5] = -action_[5];
                                // convert from euler angles to quaternions
                                tf::Quaternion q;
                                q.setRPY(action_[3] * M_PI / 180.0,
                                         action_[4] * M_PI / 180,
                                         action_[5] * M_PI / 180);
                                // check from antipodal quaternions
                                int sign = 1;
                                if (q.w() < 0) {
                                    sign = -1;
                                }
                                action_.resize(7);
                                action_[3] = sign * q.x();
                                action_[4] = sign * q.y();
                                action_[5] = sign * q.z();
                                action_[6] = sign * q.w();
                                actions_.push_back(action_);
                                // get the opposite action
                                q.setRPY(inverted_action[3] * M_PI / 180,
                                         inverted_action[4] * M_PI / 180,
                                         inverted_action[5] * M_PI / 180);
                                // check from antipodal quaternions
                                sign = 1;
                                if (q.w() < 0) {
                                    sign = -1;
                                }
                                inverted_action.resize(7);
                                inverted_action[3] = sign * q.x();
                                inverted_action[4] = sign * q.y();
                                inverted_action[5] = sign * q.z();
                                inverted_action[6] = sign * q.w();
                                actions_.push_back(inverted_action);
                            }
                        } break;
                    }
                    break;
                case ActionType::GRASP:
                    break;
                case ActionType::RELEASE:
                    break;
            }
            return actions_;
        }
        else {
            return actions_;
        }
    }

    /// @brief Get adaptive motion primitives
    /// @param start_dist The distance from the start
    /// @param goal_dist The distance from the goal
    /// @return A vector of actions
    std::vector<Action> getAdaptiveActions(double &start_dist, double &goal_dist) {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        actions_.clear();
        // Decide whether we want to use short or long primitives.
        bool use_short = false;
        if (mprim_active_type_.short_dist.first && (goal_dist < mprim_active_type_.short_dist.second)) {
            use_short = true;
        }
        // Add the short or long primitives.
        auto &action_prim = use_short ? short_mprim_ : long_mprim_;
        // Add the actions to the list of actions.
        actions_.insert(actions_.end(), action_prim.begin(), action_prim.end());

        // If allowed, and the cartesian goal distance is less than a threshold, insert snap primitive.
        // Snap I: only xyz.
        if (mprim_active_type_.snap_xyz.first && goal_dist < mprim_active_type_.snap_xyz.second) {
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});
        }
        // Snap II: only rpy.
        if (mprim_active_type_.snap_rpy.first && goal_dist < mprim_active_type_.snap_rpy.second) {
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});
        }
        // Snap III: xyz and rpy. (Most common).
        if (mprim_active_type_.snap_xyzrpy.first && goal_dist < mprim_active_type_.snap_xyzrpy.second) {
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});
            ROS_DEBUG_NAMED("adaptive_mprim", "snap xyzrpy");
            ROS_DEBUG_STREAM("goal_dist: " << goal_dist);
        }
        return actions_;
    }

    /// @brief Set values in the motion primitive active type.
    /// @param short_dist The short distance threshold
    /// @param long_dist The long distance threshold
    /// @param snap_xyz The snap xyz threshold
    /// @param snap_rpy The snap rpy threshold
    /// @param snap_xyzrpy The snap xyzrpy threshold
    void setMprimActiveType(std::pair<bool, double> short_dist,
                            std::pair<bool, double> long_dist,
                            std::pair<bool, double> snap_xyz,
                            std::pair<bool, double> snap_rpy,
                            std::pair<bool, double> snap_xyzrpy) {
        mprim_active_type_.short_dist = short_dist;
        mprim_active_type_.long_dist = long_dist;
        mprim_active_type_.snap_xyz = snap_xyz;
        mprim_active_type_.snap_rpy = snap_rpy;
        mprim_active_type_.snap_xyzrpy = snap_xyzrpy;
    }

    /// @brief Motion primitive active type: Used for adaptive motion primitives, given a few motion primitives,
    /// which one is active at a given time and it's threshold
    struct MotionPrimitiveActiveType {
        std::pair<bool, double> short_dist = std::make_pair(true, 0.1);
        std::pair<bool, double> long_dist = std::make_pair(true, 0.4);
        std::pair<bool, double> snap_xyz = std::make_pair(false, 0.2);
        std::pair<bool, double> snap_rpy = std::make_pair(false, 0.2);
        std::pair<bool, double> snap_xyzrpy = std::make_pair(true, 0.04);
    };

    ActionType action_type_;
    SpaceType space_type_;
    std::string mprim_file_name_;
    MotionPrimitiveActiveType mprim_active_type_;

    /// @brief The motion primitives. In radians.
    std::vector<Action> actions_; // This could be bug prone if getPrimActions is called after getAdaptiveActions since it will append to the actions_ vector.
    /// @brief The short distance motion primitives. In degrees.
    std::vector<Action> short_mprim_;
    /// @brief The long distance motion primitives. In degrees.
    std::vector<Action> long_mprim_;

    std::vector<bool> mprim_enabled_;
    std::vector<double> mprim_thresh_;
    double max_action_;
};

/// @class ManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
class ManipulationActionSpace : public ActionSpace {
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
    /// @param moveitInterface The moveit interface
    /// @param ManipulationType The manipulation type
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
                    action_seq.push_back(bfs_heuristic_->goal_);  // TODO: It is wierd that I am using the heuristic here
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
                }
                else {
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
                                    const double resolution = 0.05) {
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

    virtual bool getSuccessorEdgesWs(int curr_state_ind,
                                    std::vector<std::vector<int>>& edges_state_ids,
                                    std::vector<std::vector<double>> & edges_transition_costs) {
        edges_state_ids.clear();
        edges_transition_costs.clear();
        // REMOVE.
        std::vector<int> successors;
        std::vector<double> costs;
        // END REMOVE.

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

        // REMOVE.
        for (int i{0}; i < successors.size(); i++) {
            edges_state_ids.push_back({curr_state_ind, successors[i]});
            edges_transition_costs.push_back({costs[i], 0});
        }
        // END REMOVE.

        return true;
    }

    virtual bool getSuccessorEdgesCs(int curr_state_ind,
                                   std::vector<std::vector<int>>& edges_state_ids,
                                   std::vector<std::vector<double>> & edges_transition_costs) {
        edges_state_ids.clear();
        edges_transition_costs.clear();
        // REMOVE.
        std::vector<int> successors;
        std::vector<double> costs;
        // END REMOVE.

        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        // get the successors
        for (auto &action : actions) {
            // the first state is the current state and the last state is the successor
            auto curr_state_val = action.front();
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

        // REMOVE.
        for (int i{0}; i < successors.size(); i++) {
            edges_state_ids.push_back({curr_state_ind, successors[i]});
            edges_transition_costs.push_back({costs[i], 0});
        }
        // END REMOVE.

        return true;
    }

    bool getSuccessorEdges(int curr_state_ind,
                           std::vector<std::vector<int>>& edges_state_ids,
                           std::vector<std::vector<double>> & edges_transition_costs) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorEdgesCs(curr_state_ind, edges_state_ids, edges_transition_costs);
        }
        else {
            return getSuccessorEdgesWs(curr_state_ind, edges_state_ids, edges_transition_costs);
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

