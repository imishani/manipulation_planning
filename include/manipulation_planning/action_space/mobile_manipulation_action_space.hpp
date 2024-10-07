//
// Created by itamar on 10/4/24.
//

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
// #include <search/action_space/action_space.hpp>
#include "action_types.hpp"

#include "manipulation_planning/common/moveit_scene_interface.hpp"
#include "manipulation_planning/common/utils.hpp"
#include "manipulation_planning/heuristics/manip_heuristics.hpp"


namespace ims {

class MobileManipulationActionSpace : virtual public ActionSpace {

public:
    /// @brief Manipulation type
    std::shared_ptr<MobileManipulationType> mobile_manipulation_type_;
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

    std::vector<std::pair<std::string, std::vector<int>>> sub_groups_;
    std::vector<bool> mask_;

// public:
    /// @brief Constructor
    /// @param env The moveit interface
    /// @param actions_ptr The manipulation type
    /// @param bfs_heuristic The BFS heuristic
    MobileManipulationActionSpace(const MoveitInterface &env,
                                  const MobileManipulationType &actions_ptr,
                                  BFSHeuristic *bfs_heuristic = nullptr) :
    ActionSpace(),
    bfs_heuristic_(bfs_heuristic)
    {
        moveit_interface_ = std::make_shared<MoveitInterface>(env);
        mobile_manipulation_type_ = std::make_shared<MobileManipulationType>(actions_ptr);

        std::vector<std::pair<std::string, std::vector<int>>> sub_groups;
        moveit_interface_->getSubGroups(sub_groups);
        std::vector<std::pair<std::string, MobileManipulationType::group_type>> sub_groups_types;
        for (auto &sub_group : sub_groups) {
            auto group_name = sub_group.first;
            if (sub_group.second.size() == 6 ) {
                sub_groups_types.emplace_back(group_name, MobileManipulationType::group_type::ARM6DOF);
            }
            else if (sub_group.second.size() == 7) {
                sub_groups_types.emplace_back(group_name, MobileManipulationType::group_type::ARM7DOF);
            }
            else if (sub_group.second.size() == 3) {
                sub_groups_types.emplace_back(group_name, MobileManipulationType::group_type::OMNI_BASE);
            }
            else if (sub_group.second.size() == 1) {
                sub_groups_types.emplace_back(group_name, MobileManipulationType::group_type::TORSO);
            }
            else {
                ROS_ERROR("Unknown group type");
            }
        }
        mobile_manipulation_type_->init(sub_groups_types);
        // get the joint limits
        moveit_interface_->getJointLimits(joint_limits_);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        mask_.resize(joint_limits_.size());
        std::fill(mask_.begin(), mask_.end(), true);
        mask_[0] = false; mask_[1] = false; mask_[2] = false; mask_[3] = false;
    }


    void getActions(int state_id,
                    std::vector<ActionSequence> &actions_seq,
                    bool check_validity) override {
        auto curr_state = this->getRobotState(state_id);
        auto curr_state_val = curr_state->state;
        if (bfs_heuristic_ == nullptr) {
            auto actions = mobile_manipulation_type_->getPrimActions();
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
            }
            this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
            auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                                   curr_state->state_mapped.at(1),
                                                                   curr_state->state_mapped.at(2));
            auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                     curr_state->state_mapped.at(1),
                                                                     curr_state->state_mapped.at(2));
            auto actions = mobile_manipulation_type_->getAdaptivePrimActions(start_dist, goal_dist);

            for (int i{0}; i < actions.size(); i++) {
                auto action = actions[i];
                ActionSequence action_seq{curr_state_val};
                // if the action is snap, then the next state is the goal state
                // TODO: Add the option to have a goal state defined in ws even if planning in conf space
                // if any of the elements in the action is INF_DOUBLE, then the action is snap
                if (std::any_of(action.begin(), action.end(), [](double d) { return d == INF_DOUBLE; })) {
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
        roundStateToDiscretization(ws_state, mobile_manipulation_type_->state_discretization_);
    }

    /// @brief Get the end effector pose in the robot frame.
    /// @param ee_pose The end effector pose
    void calculateFK(const StateType &state, StateType &ee_pose) const {
        moveit_interface_->calculateFK(state, ee_pose);
    }

    bool isStateValid(const StateType &state_val) override {
        // check if the state is valid
        return moveit_interface_->isStateValid(state_val);
    }

    // /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
    // /// @param state_val The state to check
    // /// @param joint_state The ik solution
    // /// @return True if the state is valid, false otherwise
    // bool isStateValid(const StateType &state_val,
    //                   StateType &joint_state) const {
    //     switch (mobile_manipulation_type_->getSpaceType()) {
    //         case ManipulationType::SpaceType::ConfigurationSpace:
    //             return moveit_interface_->isStateValid(state_val);
    //         case ManipulationType::SpaceType::WorkSpace:
    //             geometry_msgs::Pose pose;
    //             pose.position.x = state_val[0];
    //             pose.position.y = state_val[1];
    //             pose.position.z = state_val[2];
    //             // Euler angles to quaternion
    //             Eigen::Quaterniond q;
    //             from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
    //             pose.orientation.x = q.x();
    //             pose.orientation.y = q.y();
    //             pose.orientation.z = q.z();
    //             pose.orientation.w = q.w();
    //             bool succ = moveit_interface_->calculateIK(pose, joint_state);
    //             if (!succ) {
    //                 ROS_INFO("IK failed");
    //                 return false;
    //             }
    //             else {
    //                 return moveit_interface_->isStateValid(joint_state);
    //             }
    //     }
    //     return false;
    // }

    // bool isStateValid(const StateType &state_val,
    //                   const StateType &seed,
    //                   StateType &joint_state) const {
    //     // check if the state is valid
    //     switch (mobile_manipulation_type_->getSpaceType()) {
    //         case ManipulationType::SpaceType::ConfigurationSpace:
    //             return moveit_interface_->isStateValid(state_val);
    //         case ManipulationType::SpaceType::WorkSpace:
    //             geometry_msgs::Pose pose;
    //             pose.position.x = state_val[0];
    //             pose.position.y = state_val[1];
    //             pose.position.z = state_val[2];
    //             // Euler angles to quaternion
    //             Eigen::Quaterniond q;
    //             from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
    //             pose.orientation.x = q.x();
    //             pose.orientation.y = q.y();
    //             pose.orientation.z = q.z();
    //             pose.orientation.w = q.w();
    //             joint_state.resize(moveit_interface_->num_joints_);
    //             bool succ = moveit_interface_->calculateIK(pose, seed, joint_state);
    //             normalizeAngles(joint_state);
    //             if (!succ) {
    //                 return false;
    //             }
    //             else {
    //                 return moveit_interface_->isStateValid(joint_state);
    //             }
    //     }
    //     return false;
    // }

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

        return moveit_interface_->isPathValid(path);
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int> &successors,
                       std::vector<double> &costs) override {
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        // get the successors
        for (auto &action : actions) {
            // the first state is the current state and the last state is the successor
            const auto& curr_state_val = action.front();
            auto new_state_val = action.back();
            // normalize the angles
            normalizeAngles(new_state_val, joint_limits_, mask_);
            // discretize the state
            roundStateToDiscretization(new_state_val, mobile_manipulation_type_->state_discretization_);
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

}

// #endif //MOBILE_MANIPULATION_ACTION_SPACE_HPP
