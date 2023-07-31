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
 * \file   panda_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2023-07-26
 */

#ifndef MANIPULATION_PLANNING_PANDAACTIONSPACE_HPP
#define MANIPULATION_PLANNING_PANDAACTIONSPACE_HPP

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
#include <manipulation_planning/common/moveit_interface.hpp>
#include <manipulation_planning/common/utils.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <manipulation_planning/manipulation_action_space.hpp>
#include <search/common/action_space.hpp>

namespace ims {

struct PandaManipulationType : ManipulationType {
    /// @brief Constructor with a given manipulation motion primitives filename.
    /// @param mprim_filename path to the file name.
    PandaManipulationType(const std::string &mprim_filename) : ManipulationType(mprim_filename) {}

    /// @brief Default destructor.
    ~PandaManipulationType() = default;

    /// @brief Get adaptive motion primitives
    /// @param start_dist The distance from the start
    /// @param goal_dist The distance from the goal
    /// @return A vector of actions
    std::vector<Action> getAdaptiveActions(double &start_dist, double &goal_dist) {
        actions_.clear();
        if (mprim_active_type_.long_dist.first)  // insert long distance primitive and convert to radians
            for (auto &action_ : long_mprim_) {
                std::vector<double> action, action_rev;
                for (auto &num : action_) {
                    action.push_back(num * M_PI / 180.0);
                    action_rev.push_back(-num * M_PI / 180.0);
                }
                actions_.push_back(action);
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.short_dist.first && (start_dist < mprim_active_type_.short_dist.second || goal_dist < mprim_active_type_.short_dist.second))
            for (auto &action_ : short_mprim_) {
                std::vector<double> action, action_rev;
                for (auto &num : action_) {
                    action.push_back(num * M_PI / 180.0);
                    action_rev.push_back(-num * M_PI / 180.0);
                }
                actions_.push_back(action);
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.snap_xyz.first && goal_dist < mprim_active_type_.snap_xyz.second)
            actions_.push_back({0, 0, 0, 0, 0, 0, 0});  // TODO: Fix this to make it better designed
        if (mprim_active_type_.snap_rpy.first && goal_dist < mprim_active_type_.snap_rpy.second)
            actions_.push_back({0, 0, 0, 0, 0, 0, 0});  // TODO: Fix this to make it better designed
        if (mprim_active_type_.snap_xyzrpy.first && goal_dist < mprim_active_type_.snap_xyzrpy.second) {
            actions_.push_back({0, 0, 0, 0, 0, 0, 0});  // TODO: Fix this to make it better designed
            ROS_INFO_NAMED("adaptive_mprim", "snap xyzrpy");
            ROS_INFO_STREAM("goal_dist: " << goal_dist);
        }
        return actions_;
    }
};

/// @class PandaManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
// Inherits the public and protected members of ManipulationActionSpace
class PandaManipulationActionSpace : public ManipulationActionSpace {
protected:


public:
    /// @brief Constructor
    /// @param moveitInterface The moveit interface
    /// @param ManipulationType The manipulation type
    PandaManipulationActionSpace(const MoveitInterface &env,
                                 const ManipulationType &actions_ptr,
                                 BFSHeuristic * bfs_heuristic = nullptr) : ManipulationActionSpace(env, actions_ptr, bfs_heuristic) {
    }

    bool isStateValid(const StateType &state_val) override {
        std::cout << "Validity check state in Panda ";
        // check if the state is valid
        switch (mManipulationType->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                std::cout << "Validity check state ";
                for (int i = 0; i < 8; i++) {
                    std::cout << state_val[i] << std::endl;
                }
                return mMoveitInterface->isStateValid(state_val);
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
                bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                if (!succ) {
                    return false;
                }
                else {
                    return mMoveitInterface->isStateValid(joint_state);
                }
        }
        return false;
    }

/*
    /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
    /// @param state_val The state to check
    /// @param joint_state The ik solution
    /// @return True if the state is valid, false otherwise
    bool isStateValid(const StateType &state_val,
                      StateType &joint_state) {
        std::cout << "maybe this one\n";
        switch (mManipulationType->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return mMoveitInterface->isStateValid(state_val);
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
                bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                if (!succ) {
                    ROS_INFO("IK failed");
                    return false;
                }
                else {
                    return mMoveitInterface->isStateValid(joint_state);
                }
        }
        return false;
    }

    bool isStateValid(const StateType &state_val,
                      const StateType &seed,
                      StateType &joint_state) {
        // check if the state is valid
        switch (mManipulationType->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return mMoveitInterface->isStateValid(state_val);
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
                joint_state.resize(mMoveitInterface->num_joints);
                bool succ = mMoveitInterface->calculateIK(pose, seed, joint_state);
                normalizeAngles(joint_state);
                if (!succ) {
                    return false;
                }
                else {
                    return mMoveitInterface->isStateValid(joint_state);
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
                                    const double resolution = 0.2) {
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
        switch (mManipulationType->getSpaceType()) {
            case ManipulationType::SpaceType::ConfigurationSpace:
                return mMoveitInterface->isPathValid(path);
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
                    bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                    if (!succ) {
                        return false;
                    }
                    else {
                        poses.push_back(joint_state);
                    }
                }
                return mMoveitInterface->isPathValid(poses);
        }
        return false;
    }
*/
    virtual bool getSuccessorsCs(int curr_state_ind,
                                 std::vector<int> &successors,
                                 std::vector<double> &costs) {
        // get the current state
        auto curr_state = this->getRobotState(curr_state_ind);
        auto curr_state_val = curr_state->state;
        auto curr_state_val_wo_time = StateType(curr_state_val.begin(), curr_state_val.end() - 1);

        std::vector<Action> actions;
        if (bfs_heuristic_ == nullptr) {
            actions = mManipulationType->getActions();
        }
        // get the actions
        else {
            if (curr_state->state_mapped.empty()) {
                mMoveitInterface->calculateFK(curr_state_val_wo_time, curr_state->state_mapped);
                this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
            }
            auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                                   curr_state->state_mapped.at(1),
                                                                   curr_state->state_mapped.at(2));
            auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                     curr_state->state_mapped.at(1),
                                                                     curr_state->state_mapped.at(2));
            actions = mManipulationType->getAdaptiveActions(start_dist, goal_dist);
        }
        // get the successors
        for (auto &action : actions) {
            // create a new state in the length of the current state
            StateType new_state_val{};
            new_state_val.resize(curr_state_val.size());
            std::fill(new_state_val.begin(), new_state_val.end(), 0.0);
            bool goal_state_set{false};
            // check if actions are all zero, if so, replace them with the goal state
            if ((bfs_heuristic_ != nullptr) && std::all_of(action.begin(), action.end(), [](double i) { return i == 0; })) {
                new_state_val = bfs_heuristic_->mGoal;
                goal_state_set = true;
            }
            else {
                for (int i{0}; i < curr_state_val.size(); i++) {
                    new_state_val[i] = curr_state_val[i] + action[i];
                }
            }
            // normalize the angles
            normalizeAngles(new_state_val, mJointLimits);
            // discretize the state
            roundStateToDiscretization(new_state_val, mManipulationType->state_discretization_);
            // check if the state went through discontinuity
            bool discontinuity{false};
            // check for maximum absolute action
            for (int i{0}; i < curr_state_val.size(); i++) {
                if (fabs(new_state_val[i] - curr_state_val[i]) > 20.0 * mManipulationType->max_action_) {
                    discontinuity = true;
                    if (goal_state_set)
                        std::cout << "Goal state set FAILED due to Discontinuity" << std::endl;
                    break;
                }
            }

            // if (isStateToStateValid(curr_state_val, new_state_val)) {
            if (isStateToStateValid(curr_state_val, new_state_val) && !discontinuity) {
                // create a new state
                int next_state_ind = getOrCreateRobotState(new_state_val);
                // add the state to the successors
                successors.push_back(next_state_ind);
                // add the cost
                // TODO: change this to the real cost
                //                    double norm = 0;
                //                    for (double i : action)
                //                    {
                //                        norm += i * i;
                //                    }
                //                    costs.push_back(sqrt(norm));
                costs.push_back(1000);
            }
            else {
                if (goal_state_set)
                    std::cout << "Goal state set FAILED" << std::endl;
            }
        }
        return true;
    }
/*
    bool getSuccessors(int curr_state_ind,
                       std::vector<int> &successors,
                       std::vector<double> &costs) override {
        if (mManipulationType->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
            return getSuccessorsCs(curr_state_ind, successors, costs);
        }
        else {
            return getSuccessorsWs(curr_state_ind, successors, costs);
        }
    }
*/
};
}  // namespace ims

#endif  // MANIPULATION_PLANNING_PANDAACTIONSPACE_HPP
