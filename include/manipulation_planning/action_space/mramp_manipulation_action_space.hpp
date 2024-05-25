/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   mramp_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2023-07-26
 */

#pragma once

// include standard libraries
#include <iostream>
#include <utility>
#include <vector>

// include ROS libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
// Include tf to convert euler angles to quaternions.
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose.hpp>

// project includes
#include <manipulation_planning/action_space/manipulation_experience_accelerated_constrained_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <manipulation_planning/common/utils.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <search/action_space/action_space.hpp>
#include <search/action_space/roadmap_action_space.hpp>
#include <search/planners/multi_agent/cbs.hpp>
#include <search/common/conflicts.hpp>
#include <search/action_space/mixin/action_space_experience_acceleratable_mixin.hpp>

namespace ims {

struct MrampManipulationActionType : ManipulationType {
    /// @brief Constructor with a given manipulation motion primitives filename.
    /// @param mprim_filename path to the file name.
    MrampManipulationActionType(const std::string &mprim_filename) : ManipulationType(mprim_filename) {
        std::cout << "MrampManipulationActionType constructor" << std::endl;

        // Set values for the motion primitives.
        setMprimActiveType(std::make_pair(true, 0.2),   // short_dist
                           std::make_pair(true, 0.4),   // long_dist
                           std::make_pair(false, 0.2),  // snap_xyz
                           std::make_pair(false, 0.2),  // snap_rpy
                           std::make_pair(true, 0.08)); // snap_xyzrpy
    }

    /// @brief Default destructor.
    ~MrampManipulationActionType() = default;

    /// @brief Get adaptive motion primitives
    /// @param start_dist The distance from the start
    /// @param goal_dist The distance from the goal
    /// @return A vector of actions
    inline std::vector<Action> getAdaptiveActions(double &start_dist, double &goal_dist) {
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
        for (auto &action_prim : use_short ? short_mprim_ : long_mprim_) {
            std::vector<double> action, action_rev;

            // Convert to radians and add a positive and negative version.
            action = action_prim;
            deg2rad(action, state_angles_valid_mask_);
            action_rev = action;
            negateState(action_rev, state_angles_valid_mask_);

            // Add the forward action to the list of actions.
            actions_.push_back(action);

            // If the reverse action is different from the forward action, add it to the list of actions.
            if (action != action_rev) {
                actions_.push_back(action_rev);
            }
        }

        // If the goal is close enough, add a snap action.
        if (mprim_active_type_.snap_xyz.first && goal_dist < mprim_active_type_.snap_xyz.second)
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed

        if (mprim_active_type_.snap_rpy.first && goal_dist < mprim_active_type_.snap_rpy.second)
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed

        if (mprim_active_type_.snap_xyzrpy.first && goal_dist < mprim_active_type_.snap_xyzrpy.second) {
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
        }

        return actions_;
    }

    inline std::vector<bool> GetStateAnglesValidMask() const {
        return state_angles_valid_mask_;
    }

    // Parameters.
    std::vector<bool> state_angles_valid_mask_ = {true, true, true, true, true, true, true, false};
};

/// @class MrampManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
// Inherits the public and protected members of ManipulationSubcostExperienceAcceleratedConstrainedActionSpace
class MrampManipulationActionSpace : public ManipulationSubcostExperienceAcceleratedConstrainedActionSpace, public RoadmapActionSpace {
private:
    /// @brief A map from a state to a set of collisions. If the state is valid, the number of collisions within the collective is zero.
    std::map<StateType, CollisionsCollective, StateTypeHash> state_to_collisions_map_;

    /// @brief The number of controllable degrees of freedom.
    int num_dof_;

protected:
    /// @brief The manipulation type
    std::shared_ptr<MrampManipulationActionType> manipulation_type_;

    /// @param the values in the robot state that are angles.
    std::vector<bool> state_angles_valid_mask_;
    
    /// @brief ROS node and marker publisher.
    rclcpp::Node::SharedPtr node_;

public:
    /// @brief Strip time off of state vectors. These may include time, if so it is removed, otherwise the state is returned as is.
    /// @param state The state to strip time from.
    /// @return The state without time.
    inline StateType getUntimedConfiguration(const StateType &state) {
        return {state.begin(), state.begin() + num_dof_};
    }

    /// @brief Constructor
    /// @param moveitInterface The moveit interface
    /// @param MrampManipulationActionType The manipulation type
    MrampManipulationActionSpace(MoveitInterface &env,
                                 const MrampManipulationActionType &actions_ptr,
                                 BFSHeuristic *bfs_heuristic = nullptr) : ManipulationSubcostExperienceAcceleratedConstrainedActionSpace(env, actions_ptr, bfs_heuristic),
                                 RoadmapActionSpace() {
        std::cout << "MrampManipulationActionSpace constructor" << std::endl;
        manipulation_type_ = std::make_shared<MrampManipulationActionType>(actions_ptr);
        state_angles_valid_mask_ = manipulation_type_->GetStateAnglesValidMask();
        num_dof_ = std::count(state_angles_valid_mask_.begin(), state_angles_valid_mask_.end(), true);

        // Set the ros node for this class.
        static int node_id = -1;
        node_id ++;
        std::string node_name = "mramp_manipulation_action_space" + std::to_string(node_id);
        node_ = rclcpp::Node::make_shared(node_name);

        // Spin the node.
        rclcpp::spin_some(node_);
    }

    /// @brief Default destructor
    ~MrampManipulationActionSpace() = default;

    /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
    /// @param state_val The state to check.
    bool isStateValid(const StateType &state_val) override;
    bool isStateValid(const StateType &state,
                    const std::vector<std::string> &other_move_group_names,
                    const std::vector<StateType> &other_move_group_states,
                    CollisionsCollective& collisions_collective);

    inline bool getSuccessors(int curr_state_ind,
                              std::vector<int> &successors,
                              std::vector<double> &costs) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCs(curr_state_ind, successors, costs);
        }
        else {
            return getSuccessorsWs(curr_state_ind, successors, costs);
        }
    }

    /// @brief Get the successors of a state.
    /// @param curr_state_ind The index of the current state.
    /// @param successors The vector of successors -- to be updated with successor state indices.
    /// @param costs The vector of costs -- to be updated with the cost of each successor.
    /// @return True if the successors were found, false otherwise.
    inline bool getSuccessorsCs(int curr_state_ind,
                                std::vector<int> &successors,
                                std::vector<double> &costs) override {
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        // get the successors
        for (auto &action : actions) {
            // the first state is the current state and the last state is the successor
            auto curr_state_val = action.front();
            auto new_state_val = action.back();
            // normalize the angles
            normalizeAngles(new_state_val, joint_limits_, manipulation_type_->state_angles_valid_mask_);
            // discretize the state
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
            // Check if the state went through a discontinuity.
            bool discontinuity{false};

            // Check for maximum absolute action TODO(yoraish): check this in time.
            for (int i{0}; i < curr_state_val.size(); i++) {
                if (manipulation_type_->state_angles_valid_mask_[i]) {
                    if (new_state_val[i] < joint_limits_[i].first || new_state_val[i] > joint_limits_[i].second) {
                        discontinuity = true;
                        break;
                    }
                }
            }

            if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val) && isSatisfyingAllConstraints(curr_state_val, new_state_val)) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);

                // Add the state to the successors.
                successors.push_back(next_state_ind);                
                // Add the cost.
                costs.push_back(1.0); // 0.087 is 5 degrees in radians.

            }
        }
        return true;
    }

    /// @brief Get the successors of a state.
    /// @param curr_state_ind The index of the current state.
    /// @param successors The vector of successors -- to be updated with successor state indices.
    /// @param costs The vector of costs -- to be updated with the cost of each successor.
    /// @return True if the successors were found, false otherwise.
    inline bool getSuccessorsCs(int curr_state_ind,
                                std::vector<int> &successors,
                                std::vector<double>& costs,
                                std::vector<double>& subcosts) override {
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        // get the successors
        for (auto &action : actions) {
            // the first state is the current state and the last state is the successor
            auto curr_state_val = action.front();
            auto new_state_val = action.back();
            // normalize the angles
            normalizeAngles(new_state_val, joint_limits_, manipulation_type_->state_angles_valid_mask_);
            // discretize the state
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
            // Check if the state went through a discontinuity.
            bool discontinuity{false};

            // Check for maximum absolute action TODO(yoraish): check this in time.
            for (int i{0}; i < curr_state_val.size(); i++) {
                if (manipulation_type_->state_angles_valid_mask_[i]) {
                    if (new_state_val[i] < joint_limits_[i].first || new_state_val[i] > joint_limits_[i].second) {
                        discontinuity = true;
                        break;
                    }
                }
            }

            if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val) && isSatisfyingAllConstraints(curr_state_val, new_state_val)) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);

                // Add the state to the successors.
                successors.push_back(next_state_ind);                
                // Add the cost.
                costs.push_back(1.0); // 0.087 is 5 degrees in radians.
            
                // Compute the subcosts. Do this by checking if the next state is in collision with the current-paths of the other agents at the next time step.
                double transition_conflict_cost = 0;
                computeTransitionConflictsCost(curr_state_val, new_state_val, transition_conflict_cost);

                // Set the subcost.
                subcosts.push_back(transition_conflict_cost);
            
            }

        }
        return true;
    }

    void computeTransitionConflictsCost(const StateType& state_val, const StateType& next_state_val, double & conflicts_cost){

        // Aggregate the states of the other agents and their names from the constraints context.
        // std::vector<StateType> other_agent_states_from;
        std::vector<StateType> other_agent_states_to_wo_time;
        std::vector<std::string> other_move_group_names;

        if (constraints_collective_ptr_->getConstraintsContext()->agent_paths.empty()) {
            // There are no other agents.
            conflicts_cost = 0.0;
            return;
        }
        for (auto other_agent_id_and_path : constraints_collective_ptr_->getConstraintsContext()->agent_paths) {
            int other_agent_id = other_agent_id_and_path.first;
            const PathType & other_agent_path  = other_agent_id_and_path.second;

            // Get the state of the other agent at the current time step. Get this from the constraints context.
            // The check here is for vertex conflicts.
            if (other_agent_path.empty()) {
                continue;
            }
            TimeType other_agent_last_time = other_agent_path.back().back();
            TimeType agent_time = next_state_val.back();
            TimeType other_agent_time = std::min(other_agent_last_time, agent_time);
            StateType other_agent_state = other_agent_path.at(other_agent_time);
            StateType other_agent_state_wo_time = getUntimedConfiguration(other_agent_state);

            // Check if the state is valid w.r.t the constraint.
            std::string other_agent_move_group_name = constraints_collective_ptr_->getConstraintsContext()->agent_names.at(other_agent_id);

            std::vector<std::string> other_move_group_names_local = constraints_collective_ptr_->getConstraintsContext()->agent_names;

            // Add the state of the other agent to the vector of states.
            other_agent_states_to_wo_time.push_back(other_agent_state_wo_time);
            other_move_group_names.push_back(other_agent_move_group_name);
        }

        CollisionsCollective collisions;
        StateType next_state_wo_time = getUntimedConfiguration(next_state_val);
        bool is_valid = isStateValid(next_state_wo_time, other_move_group_names, other_agent_states_to_wo_time, collisions);
        int num_conflicts = collisions.getCollisions().size();

        // Set the cost.
        conflicts_cost = (double)num_conflicts;
    }

    inline bool getSuccessors(int curr_state_ind,
                              std::vector<int> &successors,
                              std::vector<double>& costs,
                              std::vector<double>& subcosts) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCs(curr_state_ind, successors, costs, subcosts);
        }
        else {
            return getSuccessorsWs(curr_state_ind, successors, costs, subcosts);
        }
    }


    inline bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                              std::vector<int> &successors,
                              std::vector<double>& costs,
                              std::vector<double>& subcosts) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCsExperienceAccelerated(curr_state_ind, successors, costs, subcosts);
        }
        else {
            return getSuccessorsWs(curr_state_ind, successors, costs, subcosts);
        }
    }

    /// @brief Get the successors of a state.
    /// @param curr_state_ind The index of the current state.
    /// @param successors The vector of successors -- to be updated with successor state indices.
    /// @param costs The vector of costs -- to be updated with the cost of each successor.
    /// @return True if the successors were found, false otherwise.
    inline bool getSuccessorsCsExperienceAccelerated(int curr_state_ind,
                                std::vector<int> &successors,
                                std::vector<double>& costs,
                                std::vector<double>& subcosts) {
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        // get the successors
        for (auto &action : actions) {
            // the first state is the current state and the last state is the successor
            auto curr_state_val = action.front();
            auto new_state_val = action.back();
            // normalize the angles
            normalizeAngles(new_state_val, joint_limits_, manipulation_type_->state_angles_valid_mask_);
            // discretize the state
            roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
            // Check if the state went through a discontinuity.
            bool discontinuity{false};

            // Check for maximum absolute action TODO(yoraish): check this in time.
            for (int i{0}; i < curr_state_val.size(); i++) {
                if (manipulation_type_->state_angles_valid_mask_[i]) {
                    if (new_state_val[i] < joint_limits_[i].first || new_state_val[i] > joint_limits_[i].second) {
                        discontinuity = true;
                        break;
                    }
                }
            }

            // Check if the state transition is already recorded as valid or invalid in the experience collective.
            StateType curr_state_wo_time = getUntimedConfiguration(curr_state_val);
            StateType new_state_wo_time = getUntimedConfiguration(new_state_val);
            std::pair <StateType, StateType> state_transition = std::make_pair(curr_state_wo_time, new_state_wo_time);
            bool transition_valid_wrt_scene;

            if(experiences_collective_ptr_->isTransitionInExperiences(state_transition)){
                // The transition is already recorded as valid or invalid in the experience collective.
                // Get the transition validity from the experience collective.
                bool transition_valid = experiences_collective_ptr_->isTransitionValid(state_transition);
                transition_valid_wrt_scene = transition_valid && !discontinuity;
            }
            else{
                // The transition is not recorded in the experience collective.
                // Check if the transition is valid w.r.t the scene.
                transition_valid_wrt_scene = !discontinuity && isStateToStateValid(curr_state_val, new_state_val);

                // Add the transition to the experience collective.
                std::shared_ptr<TransitionExperience> transition_experience_ptr = std::make_shared<TransitionExperience>(state_transition, transition_valid_wrt_scene);
                experiences_collective_ptr_->addTransitionExperience(transition_experience_ptr);
            }

            // Check constraint satisfaction.
            bool is_satisfying_constraints = isSatisfyingAllConstraints(curr_state_val, new_state_val);

            // if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val) && isSatisfyingAllConstraints(curr_state_val, new_state_val)) {
            if (transition_valid_wrt_scene && is_satisfying_constraints) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);

                // Add the state to the successors.
                successors.push_back(next_state_ind);                
                // Add the cost.
                costs.push_back(1.0); // 0.087 is 5 degrees in radians.
            
                // Compute the subcosts. Do this by checking if the next state is in collision with the current-paths of the other agents at the next time step.
                double transition_conflict_cost = 0;
                computeTransitionConflictsCost(curr_state_val, new_state_val, transition_conflict_cost);

                // Set the subcost.
                subcosts.push_back(transition_conflict_cost);
            }
        }
        return true;
    }



    inline bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                              std::vector<int> &successors,
                              std::vector<double>& costs) override {
        if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCsExperienceAccelerated(curr_state_ind, successors, costs);
        }
        else {
            return getSuccessorsWs(curr_state_ind, successors, costs);
        }
    }

    /// @brief Get the successors of a state.
    /// @param curr_state_ind The index of the current state.
    /// @param successors The vector of successors -- to be updated with successor state indices.
    /// @param costs The vector of costs -- to be updated with the cost of each successor.
    /// @return True if the successors were found, false otherwise.
    inline bool getSuccessorsCsExperienceAccelerated(int curr_state_ind,
                                std::vector<int> &successors,
                                std::vector<double>& costs) {
        std::vector<double>subcosts;
        return getSuccessorsCsExperienceAccelerated(curr_state_ind, successors, costs, subcosts);
    }

    /// @brief Is the path between two robot states valid.
    inline bool isStateToStateValid(const StateType &start, const StateType &end) {
        PathType path = interpolatePath(start, end);
        return isPathValid(path);
    }

    /// @brief Is the path between two robot states valid. This method allows to check against transitions of other agents as well, checking for validity of a robot transition with respect to the other agents as well as the world.
    bool isStateToStateValid(const StateType &state_from,
                             const StateType &state_to,
                             const std::vector<std::string> &other_move_group_names,
                             const std::vector<StateType> &other_move_group_states_from,
                             const std::vector<StateType> &other_move_group_states_to,
                             CollisionsCollective& collisions_collective);

    /// @brief Whether a path is valid.
    bool isPathValid(const PathType &path) override;

    /// @brief Whether the state is satisfying a specific constraint.
    /// @param state_val The state value to check. Timed.
    /// @param next_state_val The next state value to check. Timed.
    /// @param constraint_ptr The constraint to check.
    /// @return True if the constraint is satisfied, false otherwise.
    bool isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<Constraint> &constraint_ptr);

    /// @brief Get the safe intervals for a given state configuration. Untimed.
    /// @param state_val 
    /// @param next_state_val 
    /// @param safe_intervals 
//     void getSafeIntervals(int state_id, std::vector<SafeIntervalType>& safe_intervals) override;

    /// @brief Get the conflicts from a set of paths.
    /// @param paths The paths to check for conflicts.
    /// @param conflicts_ptrs The vector of conflicts -- to be updated with the conflicts.
    /// @param conflict_types The types of conflicts that are requested.
    /// @param max_conflicts The maximum number of conflicts to return.
    /// @param agent_names The names of the agents to check for conflicts. These are the moveit move group names.
    /// @param time_start The start time from which to check for conflicts. Inclusive. -1 defaults to zero.
    /// @param time_end The end time until which to check for conflicts. Inclusive. -1 defaults to the end.
    void getPathsConflicts(std::shared_ptr<MultiAgentPaths> paths, std::vector<std::shared_ptr<Conflict>> &conflicts_ptrs, const std::vector<ConflictType> & conflict_types, int max_conflicts, const std::vector<std::string> &agent_names, TimeType time_start = 0, TimeType time_end = -1) override;


    /// @brief Get the actions from a state.
    /// @param state_id The state id.
    /// @param actions_seq The vector of actions populated -- to be updated with the actions.
    /// @param check_validity Whether to check the validity of the actions.
    void getActions(int state_id,
                    std::vector<ActionSequence> &actions_seq,
                    bool check_validity) override;

    /// @brief Sample a random state.
    void sampleState(StateType &state_val) override;

    /// @brief Get the limits of the states and its dimension.
    void getStateLimits(std::vector<std::pair<double, double>> & state_limits) const override;
    int getStateDimensionality() const override;

    /// @brief Get a successor to a state id in the direction of a given state vector.
    void getSuccessorInDirection(int state_id, int& state_id_succ, StateType state_target) override;

    /// @brief Whether a multi agent state transition is valid.

    bool multiAgentStateToStateConnector(const MultiAgentStateType& state_from, const MultiAgentStateType& state_to, MultiAgentPaths & paths, std::vector<std::string> agent_names) override;

};

bool MrampManipulationActionSpace::isStateValid(const StateType &state_val) {
    // This method checks for (a) collisions with world objects and (b) satisfaction of constraints. Collisions with other robots is taken care of by overloads of this isStateValid method.
    switch (manipulation_type_->getSpaceType()) {
        case ManipulationType::SpaceType::ConfigurationSpace: {
            // Check for collisions with world objects.
            // Remove time.
            StateType state_val_wo_time = getUntimedConfiguration(state_val);

            // Create a collisions object to store any collisions found.
            CollisionsCollective collisions;

            // Check for collisions. This also checks for joint limits.
            bool collision_free = moveit_interface_->isStateValid(state_val_wo_time, collisions);

            // Look through the collisions and check if any of them are with world objects.
            if (!collision_free) {
                for (auto &collision : collisions.getCollisions()) {
                    if (collision.body_type_0 == BodyType::WORLD_OBJECT || collision.body_type_1 == BodyType::WORLD_OBJECT) {
                        return false;
                    }

                    // TODO(yoraish): need to convert the 'base' link of the URDF to a collision world object. For now, check explicitly.
                    if (collision.body_name_0 == "base" || collision.body_name_1 == "base") {
                        return false;
                    }
                }
            }

            // Check for constraint satisfaction.
            // TODO(yoraish).
            return true;
        }
        case ManipulationType::SpaceType::WorkSpace: {
            // Workspace planning is not supported.
            throw std::runtime_error("Workspace planning is not supported in isStateValid.");
            return false;
        }
    }

    return false;
}

bool MrampManipulationActionSpace::isStateValid(const StateType &state,
                    const std::vector<std::string> &other_move_group_names,
                    const std::vector<StateType> &other_move_group_states,
                    CollisionsCollective& collisions_collective){
    return moveit_interface_->isStateValid(state, other_move_group_names, other_move_group_states, collisions_collective);
}

void MrampManipulationActionSpace::getActions(int state_id,
                                              std::vector<ActionSequence> &actions_seq,
                                              bool check_validity) {
    // This state may or may not have a time component to it.
    auto curr_state = this->getRobotState(state_id);
    auto curr_state_val = curr_state->state;
    // If it does not, add a time-zero component.
    bool is_state_timed = curr_state_val.size() == num_dof_ + 1;
    if (!is_state_timed) {
        curr_state_val.push_back(0);
    }
    if (bfs_heuristic_ == nullptr) {
        auto actions = manipulation_type_->getPrimActions();
        for (int i{0}; i < actions.size(); i++) {
            auto action = actions[i];
            ActionSequence action_seq{curr_state_val};
            // push back the new state after the action
            StateType next_state_val(curr_state_val.size());
            std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(), std::plus<>());
            // Do not add the action if the state is not timed and the action is a wait.
            bool is_action_wait = std::all_of(action.begin(), action.end() - 1, [](double val) { return val == 0; });
            if (!is_state_timed && is_action_wait) {
                continue;
            }
            actions_seq.push_back(action_seq);
        }
    }
    else {
        // Remove the time component from the state.
        StateType curr_state_val_wo_time = getUntimedConfiguration(curr_state_val);

        if (curr_state->state_mapped.empty()) {
            moveit_interface_->calculateFK(curr_state_val_wo_time, curr_state->state_mapped);
        }

        // Visualize the current state.
        geometry_msgs::msg::Pose pose;
        pose.position.x = curr_state->state_mapped.at(0);
        pose.position.y = curr_state->state_mapped.at(1);
        pose.position.z = curr_state->state_mapped.at(2);
        // Use tf2::convert to convert Roll-Pitch-Yaw angles to Quaternion
        tf2::Quaternion quaternion;
        // TODO(yorai): I think the conversion below is wrong.
        quaternion.setRPY(curr_state->state_mapped.at(3),
                        curr_state->state_mapped.at(4),
                        curr_state->state_mapped.at(5));

        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        this->visualizePose(pose);
        // this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
        
        auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                               curr_state->state_mapped.at(1),
                                                               curr_state->state_mapped.at(2));  // TODO(yoraish): can return negative value?
        auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                 curr_state->state_mapped.at(1),
                                                                 curr_state->state_mapped.at(2));
        auto actions = manipulation_type_->getAdaptiveActions(start_dist, goal_dist);
 
        for (int i{0}; i < actions.size(); i++) {
            auto action = actions[i];
            bool is_action_wait = std::all_of(action.begin(), action.end() - 1, [](double val) { return val == 0; });
            if (!is_state_timed && is_action_wait) {
                continue;
            }
            ActionSequence action_seq{curr_state_val};
            // if the action is snap, then the next state is the goal state
            // TODO: Add the option to have a goal state defined in ws even if planning in conf space
            if (action[0] == INF_DOUBLE) {

                // Two snap options.
                // The first, is to snap to the goal state as specified in the configuration space directly.
                // The second, is to snap to the neares IK solution to the previous state that also satisfies the goal state EE pose.
                int snap_type = 0;
                
                if (snap_type == 0) {
                    // SNAP to goal.
                    StateType goal_state_val = bfs_heuristic_->goal_;
                    goal_state_val.push_back(curr_state_val.back() + manipulation_type_->state_discretization_.back());
                    action_seq.push_back(goal_state_val);
                }

                else if (snap_type == 1) {
                    // SNAP to IK EE pose.
                    // Get the EE goal pose.
                    Eigen::Isometry3d ee_goal_pose = bfs_heuristic_->getGoalPoseEE();

                    // Determine the goal state value (configuration space) to be the closes IK solution satisfying the EE goal pose to the current configuration space state.
                    StateType goal_state_val;
                    if (moveit_interface_->calculateIK(ee_goal_pose, curr_state_val_wo_time, goal_state_val, 0.1)){

                        // The goal does not have time, so add the appropriate time.
                        goal_state_val.push_back(curr_state_val.back() + manipulation_type_->state_discretization_.back());
                        action_seq.push_back(goal_state_val); 
                    }
                    else{
                        // If there is no IK solution, then the action is not valid.
                        continue;
                    }
                }
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

    // If the state is not timed, strip time from all the states in the action sequences.
    if (!is_state_timed) {
        for (auto &action_seq : actions_seq) {
            for (auto &state : action_seq) {
                state.pop_back();
            }
        }
    }
}

bool MrampManipulationActionSpace::isPathValid(const PathType &path) {
    switch (manipulation_type_->getSpaceType()) {
        case ManipulationType::SpaceType::ConfigurationSpace:
            return std::all_of(path.begin(), path.end(), [this](const StateType &state_val) { return isStateValid(state_val); });

        case ManipulationType::SpaceType::WorkSpace:
            throw std::runtime_error("WorkSpace is not supported yet for isPathValid");
    }
    return false;
}

bool MrampManipulationActionSpace::isStateToStateValid(const StateType &state_from, const StateType &state_to, const std::vector<std::string> &other_move_group_names, const std::vector<StateType> &other_agent_states_from, const std::vector<StateType> &other_agent_states_to, CollisionsCollective& collisions_collective){

    // The agent states without time.
    StateType state_from_wo_time = getUntimedConfiguration(state_from);
    StateType state_to_wo_time = getUntimedConfiguration(state_to);

    // The other agent states without time.
    std::vector<StateType> other_states_from_wo_time;
    std::vector<StateType> other_states_to_wo_time;
    for (size_t i{0}; i < other_agent_states_from.size(); i++) {
        other_states_from_wo_time.push_back(getUntimedConfiguration(other_agent_states_from[i]));
        other_states_to_wo_time.push_back(getUntimedConfiguration(other_agent_states_to[i]));
    }
    
    // Interpolate between the states at time t_from and time t_to. We use the timesteps [dt, 2dt, ... 1] to include the states at time t_to and not at time t_from in the check. This operates under the assumption that the initial state was already tested for validity.
    double resolution = 0.1; // The steps between [0,1].
    std::vector<double> weights_to;
    for (double weight_to = resolution; weight_to < 1; weight_to += resolution) {
        weights_to.push_back(weight_to);
    }
    weights_to.push_back(1);

    // For each dt, create the weighted average of the states at t_from and t_to with state = state_from * (1 - weight_to) + state_tot * weight_to.
    for (double weight_to : weights_to){
        // Create a state for the agent at time t (interpolated).
        StateType state_wo_time_at_t;
        for (size_t dim{0}; dim < state_from_wo_time.size(); dim++) {
            state_wo_time_at_t.push_back(state_from_wo_time[dim] * (1 - weight_to) + state_to_wo_time[dim] * weight_to);
        }

        // Create a state for each other agent at time t (interpolated).
        std::vector<StateType> other_states_wo_time_at_t;
        for (size_t i{0}; i < other_states_from_wo_time.size(); i++) {
            // For each agent create a new interpolated state.
            StateType state_wo_time_at_t;
            for (size_t j{0}; j < other_states_from_wo_time[i].size(); j++) {
                state_wo_time_at_t.push_back(other_states_from_wo_time[i][j] * (1 - weight_to) + other_states_to_wo_time[i][j] * weight_to);
            }
            other_states_wo_time_at_t.push_back(state_wo_time_at_t);
        }

        // Check for validity.
        bool is_valid = isStateValid(state_wo_time_at_t, other_move_group_names, other_states_wo_time_at_t, collisions_collective);

        // If there is no collision, continue.
        if (!is_valid) {
            return false;
        }
    }

    // If there were no collisions, then return true.
    return true;
}

// TODO(yoraish): make into class method. Also find a better way to do this, it is a termporary hack.
int bodyNameToAgentId(std::string body_name, std::vector<std::string> agent_names) {
    // If the substring before the first underscore is similar in both the body name and the agent name, then return the agent id.
    for (int i{0}; i < agent_names.size(); i++) {
        auto agent_name = agent_names[i];
        auto body_name_substr = body_name.substr(0, body_name.find("_"));
        auto agent_name_substr = agent_name.substr(0, agent_name.find("_"));
        if (body_name_substr == agent_name_substr) {
            return i;
        }
    }

    // If the body name is not a substring of the agent name, then return -1.
    return -1;
}


bool MrampManipulationActionSpace::isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<Constraint> &constraint_ptr){
    // Check if the constraint is a vertex constraint or an edge constraint.
    switch (constraint_ptr->type) {

        /////////////////////////
        // Vertex constraints. //
        /////////////////////////
        case ims::ConstraintType::VERTEX: {
            // Convert to a vertex constraint pointer to get access to its members.
            auto* vertex_constraint_ptr = dynamic_cast<ims::VertexConstraint*>(constraint_ptr.get());
            if (vertex_constraint_ptr != nullptr) {
                // If the constraint is a vertex constraint, check if the state is valid w.r.t the constraint.
                bool is_valid_wrt_constraint = false;
                // TODO(yoraish): create a check state equality helper.
                for (int i{0}; i < vertex_constraint_ptr->state.size(); i++) {
                    if (vertex_constraint_ptr->state[i] != next_state_val[i]) {
                        is_valid_wrt_constraint = true;
                        break;
                    }
                }
                if (!is_valid_wrt_constraint){
                    return false;
                }
            }
            else {
                throw std::runtime_error("Could not cast constraint to vertex constraint");
            }
            break;
        }

        /////////////////////////
        // Edge constraints.   //
        /////////////////////////
        case ims::ConstraintType::EDGE: {
            // Convert to an edge constraint pointer to get access to its members.
            auto* edge_constraint_ptr = dynamic_cast<ims::EdgeConstraint*>(constraint_ptr.get());
            if (edge_constraint_ptr != nullptr) {
                // If the constraint is an edge constraint, check if the state is valid w.r.t the constraint.
                if (edge_constraint_ptr->state_from == state_val && edge_constraint_ptr->state_to == next_state_val) {
                    return false;
                }
            }
            else {
                throw std::runtime_error("Could not cast constraint to edge constraint");
            }
            break;
        }

        /////////////////////////
        // Sphere3D constraints./
        /////////////////////////
        // TODO(yoraish): this needs to change into an interpolation. The sphere constraints are marked with a time interval,
        // and all interpolated configurations that fall within this interval should be checked for validity.
        case ims::ConstraintType::SPHERE3D: {
            // Convert to a sphere 3d constraint pointer to get access to its members.
            auto* sphere3d_constraint_ptr = dynamic_cast<ims::Sphere3dConstraint*>(constraint_ptr.get());
            if (sphere3d_constraint_ptr != nullptr) {
                // Check for equality of time.
                if (sphere3d_constraint_ptr->time != next_state_val.back()) {
                    // continue;
                    return true;
                }

                // Remove time from the state.
                StateType next_state_val_wo_time = getUntimedConfiguration(next_state_val);

                // Get the point and the radius of the sphere.
                Eigen::Vector3d sphere_center = sphere3d_constraint_ptr->center;
                double sphere_radius = sphere3d_constraint_ptr->radius;
                
                // Method I: check distance from robot to point.
                if (false){
                    // Visualize the sphere.
                    this->visualizeSphere(sphere_center.x(), sphere_center.y(), sphere_center.z(), sphere_radius);

                    // Check if the point distance to the robot is smaller than the radius.
                    double distance_to_robot = -1;
                    moveit_interface_->getDistanceToRobot(next_state_val_wo_time, sphere_center, sphere_radius, distance_to_robot);

                    // If the distance is smaller than the radius, then the state is not valid.
                    if (distance_to_robot < sphere_radius) {
                        return false;
                    }
                }

                // Method II: check collision between robot and sphere.
                else{
                    // Visualize the sphere.
                    this->visualizeSphere(sphere_center.x(), sphere_center.y(), sphere_center.z(), sphere_radius);

                    bool is_colliding = moveit_interface_->isRobotCollidingWithSphere(next_state_val_wo_time, sphere_center, sphere_radius);
                    if (is_colliding) {
                        return false;
                    }
                }
            }
            else {
                throw std::runtime_error("Could not cast constraint to sphere3d constraint");
            }
            break;
        }

        /////////////////////////
        // Sphere3DLARGE constraints./
        /////////////////////////
        case ims::ConstraintType::SPHERE3DLARGE: {
            // Convert to a sphere 3d constraint pointer to get access to its members.
            auto* sphere3d_constraint_ptr = dynamic_cast<ims::Sphere3dLargeConstraint*>(constraint_ptr.get());
            if (sphere3d_constraint_ptr != nullptr) {
                // Check for equality of time.
                if (sphere3d_constraint_ptr->time != next_state_val.back()) {
                    // continue;
                    return true;
                }

                // Remove time from the state.
                StateType next_state_val_wo_time = getUntimedConfiguration(next_state_val);

                // Get the point and the radius of the sphere.
                Eigen::Vector3d sphere_center = sphere3d_constraint_ptr->center;
                double sphere_radius = sphere3d_constraint_ptr->radius;
                
                // Method I: check distance from robot to point.
                if (false){
                    // Visualize the sphere.
                    this->visualizeSphere(sphere_center.x(), sphere_center.y(), sphere_center.z(), sphere_radius);

                    // Check if the point distance to the robot is smaller than the radius.
                    double distance_to_robot = -1;
                    moveit_interface_->getDistanceToRobot(next_state_val_wo_time, sphere_center, sphere_radius, distance_to_robot);

                    // If the distance is smaller than the radius, then the state is not valid.
                    if (distance_to_robot < sphere_radius) {
                        return false;
                    }
                }

                // Method II: check collision between robot and sphere.
                else{
                    // Visualize the sphere.
                    this->visualizeSphere(sphere_center.x(), sphere_center.y(), sphere_center.z(), sphere_radius);

                    bool is_colliding = moveit_interface_->isRobotCollidingWithSphere(next_state_val_wo_time, sphere_center, sphere_radius);
                    if (is_colliding) {
                        return false;
                    }
                }
            }
            else {
                throw std::runtime_error("Could not cast constraint to sphere3d constraint");
            }
            break;
        }


        /////////////////////////
        // Sphere3XDLARGE constraints./
        /////////////////////////
        case ims::ConstraintType::SPHERE3DXLARGE: {
            // Convert to a sphere 3d constraint pointer to get access to its members.
            auto* sphere3d_constraint_ptr = dynamic_cast<ims::Sphere3dXLargeConstraint*>(constraint_ptr.get());
            if (sphere3d_constraint_ptr != nullptr) {
                // Check for equality of time.
                if (sphere3d_constraint_ptr->time != next_state_val.back()) {
                    // continue;
                    return true;
                }

                // Remove time from the state.
                StateType next_state_val_wo_time = getUntimedConfiguration(next_state_val);

                // Get the point and the radius of the sphere.
                Eigen::Vector3d sphere_center = sphere3d_constraint_ptr->center;
                double sphere_radius = sphere3d_constraint_ptr->radius;

                // Visualize the sphere.
                this->visualizeSphere(sphere_center.x(), sphere_center.y(), sphere_center.z(), sphere_radius);

                bool is_colliding = moveit_interface_->isRobotCollidingWithSphere(next_state_val_wo_time, sphere_center, sphere_radius);
                if (is_colliding) {
                    return false;
                }

            }
            else {
                throw std::runtime_error("Could not cast constraint to sphere3d constraint");
            }
            break;
        }


        /////////////////////////
        // Vertex PRIORITY.   //
        /////////////////////////
        case ims::ConstraintType::VERTEX_PRIORITY: {
            // Convert to a vertex avoidance constraint pointer to get access to its members.
            auto* vertex_priority_constraint_ptr = dynamic_cast<ims::VertexPriorityConstraint*>(constraint_ptr.get());
            if (vertex_priority_constraint_ptr != nullptr) {

                // Check for equality of time.
                TimeType next_state_time = next_state_val.back();
                if (vertex_priority_constraint_ptr->time != next_state_time && vertex_priority_constraint_ptr->time != -1) {
                    // continue;
                    return true;
                }

                // Check if the agent state is valid when the states of all other higher-priority agents are set in the scene.
                // Make sure that we have agent names here. This is needed for move-group setting.
                if (vertex_priority_constraint_ptr->agent_names_to_avoid.size() != vertex_priority_constraint_ptr->agent_ids_to_avoid.size()) {
                    throw std::runtime_error("Agent names and agent ids to avoid are not the same in vertex avoidance constraint.");
                }

                // Get the agent names and the agent ids.
                std::vector<std::string> agent_names_to_avoid = vertex_priority_constraint_ptr->agent_names_to_avoid;
                std::vector<int> agent_ids_to_avoid = vertex_priority_constraint_ptr->agent_ids_to_avoid;

                // Get the agent states at this time from the context.
                std::vector<StateType> agent_states_to_avoid;
                for (auto agent_id : agent_ids_to_avoid) {
                    // Get the agent state. This is either the state of the other agent in its context-path at the timestep of the next state or at the goal state of the other agent, if it already reached its goal by this time.
                    TimeType agent_to_avoid_time = std::min(next_state_time, (TimeType) constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).back().back());

                    StateType agent_state_to_avoid = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).at(agent_to_avoid_time);
                    StateType agent_state_to_avoid_wo_time = getUntimedConfiguration(agent_state_to_avoid);
                    
                    // Add the agent state to the vector.
                    agent_states_to_avoid.push_back(agent_state_to_avoid_wo_time);
                }

                // Check if the state is valid when the agent states are set in the scene.
                CollisionsCollective collisions;
                StateType next_state_val_wo_time = getUntimedConfiguration(next_state_val);
                bool is_valid = moveit_interface_->isStateValid(next_state_val_wo_time, agent_names_to_avoid, agent_states_to_avoid, collisions);

                // If there is a collision, then we are no good. Otherwise keep going through constraints.
                if (!is_valid) {
                    return false;
                }
            }
            else {
                throw std::runtime_error("Could not cast constraint to vertex avoidance constraint");
            }
            break;
        }

        /////////////////////////
        // Edge PRIORITY.     //
        /////////////////////////
        case ims::ConstraintType::EDGE_PRIORITY: {
            // Convert to an edge avoidance constraint pointer to get access to its members.
            auto *edge_priority_constraint_ptr = dynamic_cast<ims::EdgePriorityConstraint *>(constraint_ptr.get());
            // dynamic_cast<ims::EdgePriorityConstraint *>(constraint_ptr.get());

            if (edge_priority_constraint_ptr == nullptr) {
                throw std::runtime_error("Could not cast constraint to edge avoidance constraint");
            }

            // Check for equality of time.
            TimeType state_t_to = next_state_val.back();
            TimeType state_t_from = state_val.back();
            TimeType constrain_t_from = edge_priority_constraint_ptr->t_from;
            TimeType constrain_t_to = edge_priority_constraint_ptr->t_to;

            if (constrain_t_from != -1 && constrain_t_to != -1) {
                // Require that the state is in the time interval of the constraint.
                if (state_t_from != constrain_t_from || state_t_to != constrain_t_to) {
                    // continue;
                    return true;
                }
            }

            // Check if the agent state is valid when the states of all other higher-priority agents are set in the scene.
            // Make sure that we have agent names here. This is needed for move-group setting.
            std::vector<std::string> agent_names_to_avoid = edge_priority_constraint_ptr->agent_names_to_avoid;
            if (agent_names_to_avoid.size() != edge_priority_constraint_ptr->agent_ids_to_avoid.size()) {
                throw std::runtime_error("Agent names and agent ids to avoid are not the same in edge avoidance constraint.");
            }

            // Get the agent ids.
            std::vector<int> other_agent_ids = edge_priority_constraint_ptr->agent_ids_to_avoid;

            // Get the agent states at this time from the context.
            std::vector<StateType> other_agent_states_from;
            std::vector<StateType> other_agent_states_to;

            for (auto agent_id : other_agent_ids) {
                // Get the other agent states.
                TimeType other_agent_time_from = std::min(state_t_from, (TimeType) constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).back().back());
                StateType other_agent_state_from = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).at(other_agent_time_from);
                
                TimeType other_agent_time_to = std::min(state_t_to, (TimeType) constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).back().back());
                StateType other_agent_state_to = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).at(other_agent_time_to);

                // Add the agent state to the vector.
                other_agent_states_from.push_back(other_agent_state_from);
                other_agent_states_to.push_back(other_agent_state_to);
            }

            // Check if the state is valid when the agent states are set in the scene.
            CollisionsCollective collisions;
            bool is_transition_valid = isStateToStateValid(state_val, next_state_val, agent_names_to_avoid, other_agent_states_from, other_agent_states_to, collisions);

            // If there is a collision, then we are no good. Otherwise keep going through constraints.
            if (!is_transition_valid) {
                return false;
            }
            break;
        }


        /////////////////////////
        // Path PRIORITY.     //
        /////////////////////////
        case ims::ConstraintType::PATH_PRIORITY: {
            // Convert to an edge avoidance constraint pointer to get access to its members.
            auto *path_priority_constraint_ptr = dynamic_cast<ims::PathPriorityConstraint *>(constraint_ptr.get());

            if (path_priority_constraint_ptr == nullptr) {
                throw std::runtime_error("Could not cast constraint to edge avoidance constraint");
            }

            // Check for equality of time.
            TimeType state_t_to = next_state_val.back();
            TimeType state_t_from = state_val.back();

            // Check if the agent state is valid when the states of all other higher-priority agents are set in the scene.
            // Make sure that we have agent names here. This is needed for move-group setting.
            std::vector<std::string> agent_names_to_avoid = path_priority_constraint_ptr->agent_names_to_avoid;
            if (agent_names_to_avoid.size() != path_priority_constraint_ptr->agent_ids_to_avoid.size()) {
                throw std::runtime_error("Agent names and agent ids to avoid are not the same in edge avoidance constraint.");
            }

            // Get the agent ids.
            std::vector<int> other_agent_ids = path_priority_constraint_ptr->agent_ids_to_avoid;

            // Get the agent states at this time from the context.
            std::vector<StateType> other_agent_states_from;
            std::vector<StateType> other_agent_states_to;

            for (auto agent_id : other_agent_ids) {
                // Get the other agent states.
                TimeType other_agent_time_from = std::min(state_t_from, (TimeType) constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).back().back());
                StateType other_agent_state_from = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).at(other_agent_time_from);

                TimeType other_agent_time_to = std::min(state_t_to, (TimeType) constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).back().back());
                StateType other_agent_state_to = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(agent_id).at(other_agent_time_to);

                // Add the agent state to the vector.
                other_agent_states_from.push_back(other_agent_state_from);
                other_agent_states_to.push_back(other_agent_state_to);
            }

            // Check if the state is valid when the agent states are set in the scene.
            CollisionsCollective collisions;
            bool is_transition_valid = isStateToStateValid(state_val, next_state_val, agent_names_to_avoid, other_agent_states_from, other_agent_states_to, collisions);

            // If there is a collision, then we are no good. Otherwise keep going through constraints.
            if (!is_transition_valid) {
                return false;
            }
            break;
        }

        ///////////////////////////////
        // Vertex State Avoidance.   //
        ///////////////////////////////
        case ims::ConstraintType::VERTEX_STATE_AVOIDANCE: {
            // Convert to a vertex avoidance constraint pointer to get access to its members.
            auto* vertex_state_avoidance_constraint_ptr = dynamic_cast<ims::VertexStateAvoidanceConstraint*>(constraint_ptr.get());
            if (vertex_state_avoidance_constraint_ptr != nullptr) {

                // Check for equality of time.
                TimeType next_state_time = next_state_val.back();
                if (vertex_state_avoidance_constraint_ptr->getTimeInterval().second != next_state_time) {
                    // continue;
                    return true;
                }

                // Check if the agent state is valid when the states of all other higher-priority agents are set in the scene.
                // Make sure that we have agent names here: we get them from the constraints context. This is needed for move-group setting.
                std::vector<StateType> agent_states_to_avoid = vertex_state_avoidance_constraint_ptr->agent_states_to_avoid;
                std::vector<int> agent_ids_to_avoid = vertex_state_avoidance_constraint_ptr->agent_ids_to_avoid;
                std::vector<std::string> agent_names = constraints_collective_ptr_->getConstraintsContext()->agent_names;
                std::vector<std::string> agent_names_to_avoid;
                for (int agent_id_to_avoid : agent_ids_to_avoid) {
                    agent_names_to_avoid.push_back(agent_names.at(agent_id_to_avoid));
                }

                // Remove time from the states to avoid.
                std::vector<StateType> agent_states_to_avoid_wo_time;
                for (auto agent_state_to_avoid : agent_states_to_avoid) {
                    StateType agent_state_to_avoid_wo_time = getUntimedConfiguration(agent_state_to_avoid);
                    agent_states_to_avoid_wo_time.push_back(agent_state_to_avoid_wo_time);
                }

                // Check if the state is valid when the agent states are set in the scene.
                CollisionsCollective collisions;
                StateType next_state_val_wo_time = getUntimedConfiguration(next_state_val);
                bool is_valid = moveit_interface_->isStateValid(next_state_val_wo_time, agent_names_to_avoid, agent_states_to_avoid_wo_time, collisions);

                // If there is a collision, then we are no good. Otherwise keep going through constraints.
                if (!is_valid) {
                    return false;
                }
            }
            else {
                throw std::runtime_error("Could not cast constraint to vertex state avoidance constraint");
            }
            break;
        }


        /////////////////////////////
        // Edge State Avoidance.   //
        /////////////////////////////
        case ims::ConstraintType::EDGE_STATE_AVOIDANCE: {
            // Convert to an edge avoidance constraint pointer to get access to its members.
            auto *edge_state_avoidance_constraint_ptr = dynamic_cast<ims::EdgeStateAvoidanceConstraint *>(constraint_ptr.get());

            if (edge_state_avoidance_constraint_ptr == nullptr) {
                throw std::runtime_error("Could not cast constraint to edge state avoidance constraint");
            }

            // Check for equality of time.
            TimeType state_t_to = next_state_val.back();
            TimeType state_t_from = state_val.back();
            TimeType constrain_t_from = edge_state_avoidance_constraint_ptr->getTimeInterval().first;
            TimeType constrain_t_to = edge_state_avoidance_constraint_ptr->getTimeInterval().second;

            if (state_t_from != constrain_t_from || state_t_to != constrain_t_to) {
                // continue;
                return true;
            }

            // Check if the agent state is valid when the states of all other higher-priority agents are set in the scene.
            // Make sure that we have agent names here, which we get from the context. This is needed for move-group setting.
            std::vector<std::string> agent_names = constraints_collective_ptr_->getConstraintsContext()->agent_names;
            std::vector<int> other_agent_ids = edge_state_avoidance_constraint_ptr->agent_ids_to_avoid;
            std::vector<std::string> agent_names_to_avoid;
            for (int agent_id_to_avoid : edge_state_avoidance_constraint_ptr->agent_ids_to_avoid) {
                agent_names_to_avoid.push_back(agent_names.at(agent_id_to_avoid));
            }

            // Get the agent states at this time from the context.
            std::vector<StateType> other_agent_states_from = edge_state_avoidance_constraint_ptr->agent_states_from;
            std::vector<StateType> other_agent_states_to = edge_state_avoidance_constraint_ptr->agent_states_to;

            // Check if the state is valid when the agent states are set in the scene.
            CollisionsCollective collisions;
            bool is_transition_valid = isStateToStateValid(state_val, next_state_val, agent_names_to_avoid, other_agent_states_from, other_agent_states_to, collisions);

            // If there is a collision, then we are no good. Otherwise keep going through constraints.
            if (!is_transition_valid) {
                return false;
            }
            break;
        }
    } // End of switch.
    return true;
}


void MrampManipulationActionSpace::getPathsConflicts(std::shared_ptr<MultiAgentPaths> paths, std::vector<std::shared_ptr<Conflict>> &conflicts_ptrs, const std::vector<ConflictType> & conflict_types, int max_conflicts, const std::vector<std::string> &agent_names, TimeType time_start, TimeType time_end)  {

    // Loop through the paths and check for conflicts.
    // If requested, get all the conflicts.
    if (max_conflicts == -1) {
        max_conflicts = INF_INT;
    }

    // Length of the longest path.
    int max_path_length = 0;
    for (auto &path : *paths) {
        if (path.second.size() > max_path_length) {
            max_path_length = path.second.size();
        }
    }

    // If time_start and time_end are not specified, then set them to the beginning and end of the path. Time start is taken care of by the default value.
    if (time_end == -1) {
        time_end = max_path_length - 1;
    }

    // Loop through the paths and check for conflicts.
    for (TimeType t_to{time_start + 1}; t_to <= time_end; t_to++) {
        // Get the states of all robots at this time and the next time step.
        // First check time_to for collisions. If it is invalid, then stop and convert it to a vertex constraint (if applicable). Else, interpolate the states towards time_to and check for collisions along the way. If collisions are found then create an edge conflict (if applicable). 
        TimeType t_from = t_to - 1;

        std::vector<StateType> states_wo_time_at_t_from;
        std::vector<StateType> states_wo_time_at_t_to;
        for (size_t i{0}; i < paths->size(); i++) {
            // Get the position of the robot at time t. If the robot is at its goal (aka its path is shorter than t), then use its last position.
            TimeType t_from_i = std::min(t_from, (int)paths->at(i).size() - 1);
            TimeType t_to_i = std::min(t_to, (int)paths->at(i).size() - 1);
            StateType state_wo_time_at_t_from(paths->at(i)[t_from_i].begin(), paths->at(i)[t_from_i].end() - 1);
            StateType state_wo_time_at_t_to(paths->at(i)[t_to_i].begin(), paths->at(i)[t_to_i].end() - 1);
            states_wo_time_at_t_from.push_back(state_wo_time_at_t_from);
            states_wo_time_at_t_to.push_back(state_wo_time_at_t_to);
        }



        // Interpolate between the states at time t_from and time t_to. We use the timesteps [dt, 2dt, ... 1] to include the states at time t_to and not at time t_from in the check. This operates under the assumption that the initial state was already tested for validity.
        double resolution = 0.3; // The steps between [0,1].
        std::vector<double> weights_to;
        for (double weight_to = resolution; weight_to < 1; weight_to += resolution) {
            weights_to.push_back(weight_to);
        }
        weights_to.push_back(1);

        // For each dt, create the weighted average of the states at t_from and t_to with state = state_from * (1 - weight_to) + state_tot * weight_to.
        for (double weight_to : weights_to){
            // The interpolated time.
            double t_interp = t_from * (1 - weight_to) + t_to * weight_to;

            // Create the states at time t.
            std::vector<StateType> states_wo_time_at_t;
            // Iterate through agents.
            for (size_t i{0}; i < paths->size(); i++) {
                // For each agent create a new interpolated state.
                StateType state_wo_time_at_t;
                for (size_t j{0}; j < states_wo_time_at_t_from[i].size(); j++) {
                    state_wo_time_at_t.push_back(states_wo_time_at_t_from[i][j] * (1 - weight_to) + states_wo_time_at_t_to[i][j] * weight_to);
                }
                states_wo_time_at_t.push_back(state_wo_time_at_t);
            }

            // Check if the robots are colliding.
            CollisionsCollective collisions;
            std::vector<std::string> move_group_names = agent_names;
            bool is_collision = moveit_interface_->checkCollision(move_group_names, states_wo_time_at_t, collisions);

            // If there is no collision, continue.
            if (!is_collision) {
                continue;
            }

            // If there is a collision, then convert a subset of (oftentimes one) collisions to conflicts. A collision is converted to a Point3dConflict (vertex or edge), which stores the state of each of the two agents in the conflict and the point of collision. Optionally down the line, the Point3D conflict could be converted to a set of constraints. Examples would be (a) vertex constraints on each of the agents, (b) Point3D constraints on each of the agents (do not collide with point), (c) priority constraints (one agent has priority over the other), (d) PositiveVertexConstraint on one and AvoidanceConstraint on the other, etc.
            else {
                // Get the first collision from the available collisions.
                Collision collision = collisions.getCollisions()[0];

                // If either of the bodies involved is not a robot, then the proposed path intersects with the obstacle and is invalid so we raise an exception.
                if (collision.body_type_0 == BodyType::WORLD_OBJECT || collision.body_type_1 == BodyType::WORLD_OBJECT) {
                    // throw std::runtime_error("Proposed path intersects with obstacle. Collision found an obstacle.");
                    RCLCPP_INFO_STREAM(node_->get_logger(), RED << "Proposed path intersects with obstacle. Collision found an obstacle." RESET);
                    continue;
                }

                // Visualize the collision.
                this->VisualizeCube(collision.contacts.at(0).point.x(), collision.contacts.at(0).point.y(), collision.contacts.at(0).point.z(), 1.0, 0, 0, 0.5);
                
                // Get the ids of the agents in the collision.
                std::string body_name_0 = collision.body_name_0;
                std::string body_name_1 = collision.body_name_1;
                int id_agent_0 = bodyNameToAgentId(body_name_0, agent_names);
                int id_agent_1 = bodyNameToAgentId(body_name_1, agent_names);
                std::vector<int> agent_ids = {id_agent_0, id_agent_1};

                // If either of the agents is not in the list of agents, then raise an exception.
                if (id_agent_0 == -1 || id_agent_1 == -1) {
                    throw std::runtime_error("Collision between agent and unknown obstacle.");
                }

                // Get the states of the agents in the collision.
                // If the collision happened at a time step that is larger than the path length, then use the last state of the agent.
                StateType state_wo_time_agent_0 = states_wo_time_at_t[id_agent_0];
                StateType state_wo_time_agent_1 = states_wo_time_at_t[id_agent_1];

                // Add the time before collision, time of collision, and time after collision. These objects will be used in conflict creation, optionally.
                StateType from_state_agent_0 = states_wo_time_at_t_from[id_agent_0];
                from_state_agent_0.push_back(t_from);
                StateType from_state_agent_1 = states_wo_time_at_t_from[id_agent_1];
                from_state_agent_1.push_back(t_from);
                StateType to_state_agent_0 = states_wo_time_at_t_to[id_agent_0];
                to_state_agent_0.push_back(t_to);
                StateType to_state_agent_1 = states_wo_time_at_t_to[id_agent_1];
                to_state_agent_1.push_back(t_to);
                StateType interp_state_agent_0 = states_wo_time_at_t[id_agent_0];
                interp_state_agent_0.push_back(t_interp);
                StateType interp_state_agent_1 = states_wo_time_at_t[id_agent_1];
                interp_state_agent_1.push_back(t_interp);

                // Put in containers.
                std::vector<StateType> from_states = {from_state_agent_0, from_state_agent_1};
                std::vector<StateType> to_states = {to_state_agent_0, to_state_agent_1};
                std::vector<StateType> interp_states = {interp_state_agent_0, interp_state_agent_1};


                // Create the requested conflicts.
                // NOTE(yoraish): all specified conflict types are generated. If there are more conflict types than required conflicts, then there will be more conflicts returned than requested.
                for (ConflictType conflict_type : conflict_types){
                    switch (conflict_type){
                        case ConflictType::VERTEX: {
                            // Create a conflict only if the collision happened at the state_to.
                            if (weight_to != 1) {
                                break;
                            }

                            VertexConflict conflict(to_states, agent_ids);

                            // Add the conflict to the vector of conflicts.
                            std::shared_ptr<Conflict> conflict_ptr = std::make_shared<VertexConflict>(conflict);
                            conflicts_ptrs.push_back(conflict_ptr);

                            break;
                        }

                        case ConflictType::EDGE: {
                            // If the time of the collision is not integral, aka it was found when interpolating between states, then create a private grids edge conflicts for each of the affected agents between their previous and next states.
                            if (weight_to != 1) {
                                EdgeConflict conflict(from_states, to_states, agent_ids);

                                // Add the conflict to the vector of conflicts.
                                std::shared_ptr<Conflict> conflict_ptr = std::make_shared<EdgeConflict>(conflict);
                                conflicts_ptrs.push_back(conflict_ptr);
                            }
                            break;
                        }

                        case ConflictType::POINT3D_VERTEX: {
                            // Create a conflict. This conflict will be a Point3dVertexConflict or a Point3dEdgeConflict, depending on the time of the collision.
                            // If the time of the collision is not integral, aka it was found when interpolating between states, then create a point3d conflict for each of the affected agents between their previous and next states.
                            if (weight_to != 1){
                                break;
                            }

                            else{
                                Point3dVertexConflict point3d_conflict(to_states, agent_ids, collision.contacts.at(0).point);

                                // Add the conflict to the vector of conflicts.
                                std::shared_ptr<Conflict> point3d_conflict_ptr = std::make_shared<Point3dVertexConflict>(point3d_conflict);
                                conflicts_ptrs.push_back(point3d_conflict_ptr);
                            }
                            break;
                        }

                        case ConflictType::POINT3D_EDGE: {
                            // Create a conflict. This conflict will be a Point3dVertexConflict or a Point3dEdgeConflict, depending on the time of the collision.
                            // If the time of the collision is not integral, aka it was found when interpolating between states, then create a point3d conflict for each of the affected agents between their previous and next states.
                            if (weight_to != 1){
                                Point3dEdgeConflict point3d_conflict(from_states, to_states, agent_ids, collision.contacts.at(0).point);

                                // Add the conflict to the vector of conflicts.
                                std::shared_ptr<Conflict> point3d_conflict_ptr = std::make_shared<Point3dEdgeConflict>(point3d_conflict);
                                conflicts_ptrs.push_back(point3d_conflict_ptr);
                            }
                            break;
                        }

                        default: {
                            throw std::runtime_error("Conflict type " + std::to_string((int)conflict_type) + " is not supported.");
                        }
                    }
                }

                // If we have reached the maximum number of conflicts, then break.
                if (conflicts_ptrs.size() >= max_conflicts) {
                    return;
                }
            }
        }
    }
}

void MrampManipulationActionSpace::sampleState(StateType &state_val) {
    // Sample a state from the state space.
    switch (manipulation_type_->getSpaceType()) {
        case ManipulationType::SpaceType::ConfigurationSpace: {
            // Sample a state from the configuration space.
            int num_joints = joint_limits_.size();

            // Seed.
            srand((unsigned) time(NULL));
            while (true){

                // Sample a state.
                StateType sampled_state(num_joints);
                for (int i{0}; i < num_joints; i++) {
                    sampled_state[i] = ((double) rand() / (RAND_MAX)) * (joint_limits_[i].second - joint_limits_[i].first) + joint_limits_[i].first; 
                }

                // Check if the state is valid w.r.t. scene objects.
                bool is_sampled_state_valid = moveit_interface_->isStateValid(sampled_state);
                if (is_sampled_state_valid) {
                    state_val = sampled_state;
                    return;
                }
            }
        }

        case ManipulationType::SpaceType::WorkSpace: {
            throw std::runtime_error("WorkSpace is not supported yet for sampleState");
            return;
        }
    }
}

void MrampManipulationActionSpace::getStateLimits(std::vector<std::pair<double, double>> & state_limits) const {
    // Get the dimensionality of the state space.
    state_limits = joint_limits_;
}

int MrampManipulationActionSpace::getStateDimensionality() const{
    return joint_limits_.size();
}


void MrampManipulationActionSpace::getSuccessorInDirection(int state_id, int& state_id_succ, StateType state_target) {
    throw std::runtime_error("getSuccessorInDirection is not implemented for ManipulationActionSpace");
}

bool MrampManipulationActionSpace::multiAgentStateToStateConnector(const MultiAgentStateType& state_from, const MultiAgentStateType& state_to, MultiAgentPaths& paths, std::vector<std::string> agent_names)  {
    throw std::runtime_error("multiAgentStateToStateConnector is not implemented for ManipulationActionSpace");
}

//  void ims::MrampManipulationActionSpace::getSafeIntervals(int state_id, std::vector<SafeIntervalType>& safe_intervals){
//      throw std::runtime_error("getSafeIntervals is not implemented for ManipulationActionSpace");
//  }


}  // namespace ims

#pragma once
