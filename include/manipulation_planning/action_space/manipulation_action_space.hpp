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
                                  max_action_(0.0) {
                                  };

    /// @brief Constructor with motion primitives file given
    /// @param[in] mprim_file The path to the motion primitives file
    /// @param[in] bfs_heuristic A pointer to a BFSHeuristic object
    explicit ManipulationType(std::string mprim_file) : action_type_(ActionType::MOVE),
                                                        space_type_(SpaceType::ConfigurationSpace),
                                                        mprim_file_name_(std::move(mprim_file)),
                                                        max_action_(0.0) {
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

    void loadMprimFamily(const YAML::Node &root_node,
                         const std::string &family_name,
                         std::vector<ActionSequence> &action_seqs,
                         std::vector<std::vector<double>> &action_transition_times) {
        if (root_node[family_name]) {
            for (const auto &action : root_node[family_name]) {
                std::string action_name = action.first.as<std::string>();
                // Check if "mprim_sequence" exists in the key list.
                // This will commonly be in degrees for configuration space primitives, and in [m, m, m, deg, deg, deg] for workspace.
                // The structure would be like this:
                //          action_name:
                //            mprim_sequence:
                //              - [ 0, 0, 0, 0, 0, 0, 0 ]
                //              - [ 7, 0, 0, 0, 0, 0, 0 ]
                //            delta_time_steps: [ 1, 0 ]
                //            generate_negative: true
                if (!action.second["mprim_sequence"]) {
                    std::cerr << "Action \"" << action_name << "\" does not have \"mprim_sequence\".\n";
                    throw std::runtime_error("Action \"" + action_name + "\" does not have \"mprim_sequence\".");
                }
                // Check if required to also generate negated actions.
                bool is_generate_negated_action = false;
                if (action.second["generate_negative"]) {
                    is_generate_negated_action = action.second["generate_negative"].as<bool>();
                }
                ActionSequence action_seq;
                ActionSequence neg_action_seq;
                std::vector<std::vector<double>> action_transition_cost;
                for (const auto &action_seq_state_it : action.second["mprim_sequence"]) {
                    StateType state;
                    StateType neg_state;
                    for (int i{0}; i < action_seq_state_it.size(); i++) {
                        double num = action_seq_state_it[i].as<double>();
                        // Converte to radians if the planning space is configuration, or if planning in workspace and i in {3, 4 5}.
                        if (space_type_ == SpaceType::ConfigurationSpace || (space_type_ == SpaceType::WorkSpace && i > 2)) {
                            num = num / 180.0 * M_PI;
                        }
                        // Populate max_action_.
                        if (abs(num) > max_action_) {
                            max_action_ = abs(num);
                        }
                        state.push_back(num);
                        neg_state.push_back(-num);
                    }
                    action_seq.push_back(state);
                    if (is_generate_negated_action) {
                        neg_action_seq.push_back(neg_state);
                    }
                }
                // Note that we do not add the action_seq and the neg_action_seq to the action_seqs. We only do it at the end as those may need to be modified.
                // Get the transition times, if those are available.
                // This does two things. First, it stores the transition times for each action sequence:
                if (action.second["mprim_sequence_transition_costs"]) {
                    std::vector<double> transition_time;  // This is the same for both the normal and negated action.
                    for (size_t i{0}; i < action.second["mprim_sequence_transition_costs"].size(); i++) {
                        double num = action.second["mprim_sequence_transition_costs"][i].as<TimeType>();
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
        } else {
            std::cerr << "Family \"" << family_name << "\" not found in the YAML file.\n";
        }
        // Check that the length of the transition times is the same as the length of the action sequence.
        if (action_seqs.size() != action_transition_times.size()) {
            std::cerr << "The number of action sequences and the number of transition times do not match.\n";
            throw std::runtime_error("The number of action sequences and the number of transition times do not match.");
        }
    }

    void readMPfile() {
        // Read the motion primitive YAML file.
        YAML::Node mprim_config = YAML::LoadFile(mprim_file_name_);
        // The motion primitive file defines families of actions. In this action space we care about the
        // 1. short distance motion primitives (short_mprim_) and
        // 2. long distance motion primitives (long_mprim_).
        if (space_type_ == SpaceType::ConfigurationSpace) {
            loadMprimFamily(mprim_config, "short_primitives", short_mprim_seqs_, short_mprim_transition_times_);
            loadMprimFamily(mprim_config, "long_primitives", long_mprim_seqs_, long_mprim_transition_times_);
        } else if (space_type_ == SpaceType::WorkSpace) {
            // TODO(yoraish, imishani): keeping previous convention: using short_mprim_ for workspace motion primitives.
            loadMprimFamily(mprim_config, "short_primitives", short_mprim_seqs_, short_mprim_transition_times_);
        }
    }

    // Returns the actions as they are in the motion primitive file. Those are not rooted in any state, but in the origin.
    void getPrimActions(std::vector<ActionSequence> &action_seqs,
                        std::vector<std::vector<double>> &action_transition_costs) override {
        // Read the motion primitive file if needed.
        if (short_mprim_seqs_.empty() && long_mprim_seqs_.empty()) {
            readMPfile();
        }
        // Populate the actions_ vector if not done so already.
        if (action_seqs_.empty() || action_transition_times_.empty()) {
            switch (action_type_) {
                case ActionType::MOVE:
                    switch (space_type_) {
                        case SpaceType::ConfigurationSpace: {
                            action_seqs_.insert(action_seqs_.end(), long_mprim_seqs_.begin(), long_mprim_seqs_.end());
                            action_seqs_.insert(action_seqs_.end(), short_mprim_seqs_.begin(), short_mprim_seqs_.end());
                            action_transition_times_.insert(action_transition_times_.end(),
                                                            long_mprim_transition_times_.begin(),
                                                            long_mprim_transition_times_.end());
                            action_transition_times_.insert(action_transition_times_.end(),
                                                            short_mprim_transition_times_.begin(),
                                                            short_mprim_transition_times_.end());

                        } break;
                        case SpaceType::WorkSpace: {
                            // Populate the action_seqs_ vector with the short_mprim_ and the long_mprim_.
                            // Convert the euler angles (rad) to quaternions.
                            for (int i_short_action_seq = 0; i_short_action_seq < short_mprim_seqs_.size(); i_short_action_seq++) {
                                ActionSequence &short_action_seq = short_mprim_seqs_[i_short_action_seq];
                                ActionSequence action_seq_converted;
                                for (Action &short_action : short_action_seq) {
                                    // Convert from euler angles (rad) to quaternions.
                                    tf::Quaternion q;
                                    q.setRPY(short_action[3],
                                             short_action[4],
                                             short_action[5]);
                                    // Check from antipodal quaternions.
                                    int sign = 1;
                                    if (q.w() < 0) {
                                        sign = -1;
                                    }
                                    short_action.resize(7);
                                    short_action[3] = sign * q.x();
                                    short_action[4] = sign * q.y();
                                    short_action[5] = sign * q.z();
                                    short_action[6] = sign * q.w();
                                    action_seq_converted.push_back(short_action);
                                }
                                action_transition_times_.push_back(short_mprim_transition_times_[i_short_action_seq]);
                                action_seqs_.push_back(action_seq_converted);
                            }
                        } break;
                    }
                    break;
                case ActionType::GRASP:
                    break;
                case ActionType::RELEASE:
                    break;
            }
        }
        action_seqs = action_seqs_;
        action_transition_costs = action_transition_times_;
    }

    /// @brief Get the possible actions
    /// @return A vector of all possible actions
    [[deprecated("Use getPrimActions() instead.")]]
    std::vector<Action> getPrimActions() override {
        // Call the getPrimActions function.
        std::vector<ActionSequence> action_seqs;
        std::vector<std::vector<double>> action_transition_times;
        getPrimActions(action_seqs, action_transition_times);

        // Backwards compatability hack. This will return the actions_ vector where actions are only single-step actions.
        std::vector<Action> actions;
        for (auto action_seq : action_seqs) {
            actions.push_back({action_seq.back()});
            // Only allow this if the action is a single step action (aka with exactly two elements in the sequence).
            if (action_seq.size() != 2) {
                throw std::runtime_error(
                    "The action sequence is not a single step action, GetPrimActions will lose information. Aborting.");
            }
        }
        return actions;
    }

    /// @brief Get the possible action sequences in an adaptive manner.
    /// \param start_dist
    /// \param goal_dist
    /// \param action_seqs
    /// \param action_transition_costs
    /// \param is_add_long_with_short If true, then the long primitives are added to the action set with the short primitives.
    void getAdaptivePrimActionSequences(double &start_dist,
                                        double &goal_dist,
                                        std::vector<ActionSequence> &action_seqs,
                                        std::vector<std::vector<double>> &action_transition_costs,
                                        bool is_add_long_with_short = false) {
        if (short_mprim_seqs_.empty() && long_mprim_seqs_.empty()) {
            readMPfile();
        }
        action_seqs.clear();
        action_transition_costs.clear();
        // Decide whether we are in the short- or long-primitive region.
        bool is_short = mprim_active_type_.short_dist.first &&
                        (start_dist < mprim_active_type_.short_dist.second || goal_dist < mprim_active_type_.short_dist.second);

        // Add long primitives under two conditions.
        // 1. If need to add short and adding the long primitives to the set of primitives is required, do that first.
        // 2. If the goal distance is larger than the long distance threshold, add the long primitives.
        if ((is_short && is_add_long_with_short) ||
            (!is_short &&
             mprim_active_type_.long_dist.first)) {
            action_seqs.insert(action_seqs.end(), long_mprim_seqs_.begin(), long_mprim_seqs_.end());
            action_transition_costs.insert(action_transition_costs.end(), long_mprim_transition_times_.begin(),
                                           long_mprim_transition_times_.end());
        }
        // Add the short motion primitives as well if required.
        if (is_short) {
            action_seqs.insert(action_seqs.end(), short_mprim_seqs_.begin(), short_mprim_seqs_.end());
            action_transition_costs.insert(action_transition_costs.end(), short_mprim_transition_times_.begin(),
                                           short_mprim_transition_times_.end());
        }

        // If allowed, and the cartesian goal distance is less than a threshold, insert snap primitive. The time for this is 1.
        ActionSequence snap_action_seq = {{0, 0, 0, 0, 0, 0}, {INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE}};
        std::vector<double> snap_transition_time = {230, 0.0};
        // TODO(yoraish): the xyz and rpy snaps are not handled correctly yet.
        // Snap I: only xyz.
        if (mprim_active_type_.snap_xyz.first && goal_dist < mprim_active_type_.snap_xyz.second) {
            action_seqs.push_back(snap_action_seq);
            action_transition_costs.push_back(snap_transition_time);
        }
        // Snap II: only rpy.
        if (mprim_active_type_.snap_rpy.first && goal_dist < mprim_active_type_.snap_rpy.second) {
            action_seqs.push_back(snap_action_seq);
            action_transition_costs.push_back(snap_transition_time);
        }
        // Snap III: xyz and rpy. (Most common).
        if (mprim_active_type_.snap_xyzrpy.first && goal_dist < mprim_active_type_.snap_xyzrpy.second) {
            action_seqs.push_back(snap_action_seq);
            action_transition_costs.push_back(snap_transition_time);
            ROS_DEBUG_NAMED("adaptive_mprim", "snap xyzrpy");
            ROS_DEBUG_STREAM("goal_dist: " << goal_dist);
        }
    }

    /// @brief Get adaptive motion primitives
    /// @param start_dist The distance from the start
    /// @param goal_dist The distance from the goal
    /// @return A vector of actions
    /// @note This should be named getAdaptivePrimActions since it returns non-transformed actions (delta from origin).
    [[deprecated("Use getAdaptivePrimActionSequences() instead.")]]
    std::vector<Action> getAdaptiveActions(double &start_dist,
                                           double &goal_dist,
                                           bool is_add_long_with_short = false) {
        // Call the getAdaptivePrimActionSequences function.
        std::vector<ActionSequence> action_seqs;
        std::vector<std::vector<double>> action_transition_times;
        getAdaptivePrimActionSequences(start_dist, goal_dist, action_seqs, action_transition_times,
                                       is_add_long_with_short);
        std::vector<Action> actions;
        // Backwards compatability hack. This will return the only single-step actions and abort if the action sequences
        // are not of a single step (two states).
        for (auto action_seq : action_seqs) {
            actions.push_back({action_seq.back()});
            if (action_seq.size() != 2) {
                throw std::runtime_error(
                    "The action sequence is not a single step action, GetAdaptiveActions will lose information. Aborting.");
            }
        }
        return actions;
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
        // std::pair<bool, double> snap_xyzrpy = std::make_pair(true, 0.04);
        // std::pair<bool, double> snap_xyzrpy = std::make_pair(true, 0.05);
        std::pair<bool, double> snap_xyzrpy = std::make_pair(true, 0.4);
    };

    ActionType action_type_;
    SpaceType space_type_;
    std::string mprim_file_name_;
    MotionPrimitiveActiveType mprim_active_type_;

    /// @brief The short distance motion primitives. In radians.
    /// @details The sequence is a list of states (potentially with a time dimension in the back). This list usually starts from the zero state and ends at the final state.
    ///          Each entry is not a delta from the previous, but a delta from the zero state.
    std::vector<ActionSequence> short_mprim_seqs_;
    /// @brief The transitions costs at i are the time it takes to move from state i to state i+1 in the action sequence.
    std::vector<std::vector<double>> short_mprim_transition_times_;
    /// @brief The long distance motion primitives. In radians. Similar explanations as for short_mprim_seqs_ and short_mprim_transition_times_.
    std::vector<ActionSequence> long_mprim_seqs_;
    std::vector<std::vector<double>> long_mprim_transition_times_;
    /// @brief The motion primitives. In radians.
    std::vector<ActionSequence> action_seqs_;  // This could be bug prone if getPrimActions is called after getAdaptiveActions since it will append to the actions_ vector.
    std::vector<std::vector<double>> action_transition_times_;

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
    bool vis_ = false;

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
                    std::transform(curr_state_val.begin(), curr_state_val.end(), prim_action.begin(),
                                   next_state_val.begin(),
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
            if (vis_) {
                moveit_interface_->calculateFK(curr_state_val, curr_state->state_mapped);
                this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1),
                                     curr_state->state_mapped.at(2));
            }
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
                    transformed_action_seq = {curr_state_val,
                                              bfs_heuristic_->goal_};  // TODO: It is wierd that I am using the heuristic here
                    transformed_action_transition_costs = {1.0, 0.0};
                } else {
                    for (int i = 0; i < prim_action_seq.size(); i++) {
                        const Action &prim_action = prim_action_seq[i];
                        StateType next_state_val(curr_state_val.size());
                        std::transform(curr_state_val.begin(), curr_state_val.end(), prim_action.begin(),
                                       next_state_val.begin(),
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
        auto tip_link = moveit_interface_->planning_scene_->getRobotModel()->getJointModelGroup(
                                                                               moveit_interface_->group_name_)
                            ->getLinkModelNames()
                            .back();
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
