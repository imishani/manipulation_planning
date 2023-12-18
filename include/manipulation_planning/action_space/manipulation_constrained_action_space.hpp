/*
 * Copyright (C) 2023, Itamar Mishani, Yorai Shaoul
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
 * \file   manipulation_constrained_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Aug 6 2023
 */

#ifndef MANIPULATION_PLANNING_MANIPULATIONCONSTRAINEDACTIONSPACE_HPP
#define MANIPULATION_PLANNING_MANIPULATIONCONSTRAINEDACTIONSPACE_HPP

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
#include <search/action_space/constrained_action_space.hpp>
#include <manipulation_planning/action_space/manipulation_action_space.hpp>

namespace ims {

/// @class ManipulationConstrainedActionSpace
/// @brief A class that implements the ConstrainedActionSpace for Moveit. This class borrows many function implementations from the ManipulationConstrainedActionSpace class.
/// @class ManipulationConstrainedActionSpace
/// @brief A class that implements the ActionSpace for Moveit
class ManipulationConstrainedActionSpace : public ConstrainedActionSpace {
protected:
    /// @brief Manipulation type
    std::shared_ptr<ManipulationType> manipulation_type_;
    /// @brief Moveit interface
    std::shared_ptr<MoveitInterface> moveit_interface_;
    /// @brief joint limits
    std::vector<std::pair<double, double>> joint_limits_;
    /// @brief Joint states seed
    //    std::vector<double> mJointStatesSeed {0, 0, 0, 0, 0, 0};
    /// @brief The BFS heuristic
    BFSHeuristic *bfs_heuristic_;

    int vis_id_ = 0;
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;

    // Instance of the ManipulationActionSpace class to use some of its methods.
    // Since the ManipulationActionSpace class does not have a default constructor, the line above does not work. So instead we set it to nullptr and instantiate it in the constructor.
    std::shared_ptr<ManipulationActionSpace> manip_action_space_ = nullptr;

public:
    /// @brief Constructor
    /// @param moveitInterface The moveit interface
    /// @param ManipulationType The manipulation type
    ManipulationConstrainedActionSpace(const MoveitInterface &env,
                            const ManipulationType &actions_ptr,
                            BFSHeuristic *bfs_heuristic = nullptr) : bfs_heuristic_(bfs_heuristic), ConstrainedActionSpace() {
        moveit_interface_ = std::make_shared<MoveitInterface>(env);
        manipulation_type_ = std::make_shared<ManipulationType>(actions_ptr);
        
        // Get the joint limits.
        moveit_interface_->getJointLimits(joint_limits_);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        // Instantiate the ManipulationActionSpace class, for use of some of its methods.
        manip_action_space_ = std::make_shared<ManipulationActionSpace>(env, actions_ptr, bfs_heuristic);
    }

    virtual void getActions(int state_id,
                            std::vector<ActionSequence> &actions_seq,
                            bool check_validity) override {
        manip_action_space_->getActions(state_id, actions_seq, check_validity);
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
        joints.copyJointGroupPositions(moveit_interface_->group_name_, joint_states);
    }

    /// @brief Get the workspace state
    /// @param ws_state The workspace state
    void getCurrWorkspaceState(StateType &ws_state) {
        manip_action_space_->getCurrWorkspaceState(ws_state);
    }

    bool isStateValid(const StateType &state_val) override {
        return manip_action_space_->isStateValid(state_val);
    }

    /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
    /// @param state_val The state to check
    /// @param joint_state The ik solution
    /// @return True if the state is valid, false otherwise
    bool isStateValid(const StateType &state_val,
                      StateType &joint_state) {
        return manip_action_space_->isStateValid(state_val, joint_state);
    }

    bool isStateValid(const StateType &state_val,
                      const StateType &seed,
                      StateType &joint_state) {
        return manip_action_space_->isStateValid(state_val, seed, joint_state);
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
        return manip_action_space_->isPathValid(path);
    }

    virtual bool getSuccessorsWs(int curr_state_ind,
                                 std::vector<int> &successors,
                                 std::vector<double> &costs) {
        return manip_action_space_->getSuccessors(curr_state_ind, successors, costs);
    }

    virtual bool getSuccessorsCs(int curr_state_ind,
                                 std::vector<int> &successors,
                                 std::vector<double> &costs) {
        return manip_action_space_->getSuccessors(curr_state_ind, successors, costs);
    }

    virtual bool getSuccessors(int curr_state_ind,
                       std::vector<int> &successors,
                       std::vector<double> &costs) override {
        return manip_action_space_->getSuccessors(curr_state_ind, successors, costs);
    }

    /// @brief Visualize a state point in rviz for debugging
    /// @param state_id The state id
    /// @param type The type of state (greedy, attractor, etc)
    void VisualizePoint(double x, double y, double z) {
        // Use the implementation from manipulation action space.
        manip_action_space_->VisualizePoint(x, y, z);
    }

    /// @brief Visualize a state point in rviz for debugging, with color.
    void VisualizeCube(double x, double y, double z, double r, double g, double b, double a) {
        // Use the implementation from manipulation action space.
        visualization_msgs::Marker marker;
        marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
        marker.header.stamp = ros::Time();
        marker.ns = "graph";
        marker.id = vis_id_;
        marker.type = visualization_msgs::Marker::CUBE;
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
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        
        // Lifetime.
        marker.lifetime = ros::Duration(5.0);
        // visualize
        vis_pub_.publish(marker);
        vis_id_++;
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

    /// @brief Get the end effector pose in the robot frame.
    /// @param ee_pose The end effector pose
    void calculateFK(const StateType &state, StateType &ee_pose)
    {
        moveit_interface_->calculateFK(state, ee_pose);
    }

    /// @brief Ask for conflicts between paths.
    /// @param paths The paths to be looked at.
    /// @param conflicts_ptrs The conflicts that were found. This is populated by the function.
    /// @param conflict_types The types of conflicts that are requested.
    /// @param max_conflicts The maximum number of conflicts to return.
    /// @param names The names of the agents.
    /// @param time_start The start time from which to check for conflicts. Inclusive. -1 defaults to zero.
    /// @param time_end The end time until which to check for conflicts. Inclusive. -1 defaults to the end.
    void getPathsConflicts(std::shared_ptr<MultiAgentPaths> paths, 
                           std::vector<std::shared_ptr<Conflict>> &conflicts_ptrs, 
                           const std::vector<ConflictType> &conflict_types,
                           int max_conflicts, 
                           const std::vector<std::string> &names,
                           TimeType time_start = 0, TimeType time_end = -1) override {
        throw std::runtime_error("Not implemented");
    }
};
}  // namespace ims

#endif  // MANIPULATION_PLANNING_MANIPULATIONCONSTRAINEDACTIONSPACE_HPP
