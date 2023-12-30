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
 * \file   xecbs_test.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   December 27 2023
*/

// C++ includes
#include <memory>
#include <iostream>
#include <boost/filesystem.hpp>

// project includes
#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include <manipulation_planning/action_space/manipulation_constrained_action_space.hpp>
#include <manipulation_planning/action_space/mramp_manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/multi_agent/eaecbs.hpp>
#include <search/planners/wastar.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <manipulation_planning/common/utils.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

std::string getMPrimFilePathFromMoveGroupName(rclcpp::Node::SharedPtr node, std::string move_group_name) {
    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(node, move_group_name);

    // Get the number of joints.
    int num_joints = (int)move_group.getVariableCount();

    std::string path_mprim = full_path.string() + "/config/manip_timed_" + std::to_string(num_joints) + "dof.mprim";
    return path_mprim;
}

int main(int argc, char** argv) {

    ///////////////////////////////////
    // ROS setup.
    ///////////////////////////////////
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("xecbs_test_node", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("xecbs_test_node");

    ///////////////////////////////////
    // Scene Interfaces.
    ///////////////////////////////////
    StateType discretization{1, 1, 1, 1, 1, 1, 1, 1};  // In Degrees, with the last element is time in time-units.
    std::vector<bool> state_valid_mask{true, true, true, true, true, true, true, false};
    ims::deg2rad(discretization, state_valid_mask); // In Radians, the last element is time in time-units.
    int num_joints = 7;

    // The relevant moveit groups.
    std::vector<std::string> move_group_names = {"panda0_arm", "panda1_arm", "panda2_arm", "panda3_arm"};
    std::vector<std::string> agent_names = {"panda0", "panda1", "panda2", "panda3"};
    std::vector<std::shared_ptr<ims::SubcostConstrainedActionSpace>> action_spaces;

    // Get the motion primitives file paths.
    std::vector<std::string> mprim_file_paths;
    for (std::string& move_group_name : move_group_names) {
        mprim_file_paths.push_back(getMPrimFilePathFromMoveGroupName(node, move_group_name));
    }

    // Get the distance field of the scene for the BFS heuristic (used for snap).
    auto df = ims::getDistanceFieldMoveIt();

    // Keep the copies of the scene interfaces.
    std::vector<ims::MoveitInterface> scene_interfaces;
    for (std::string& move_group_name : move_group_names) {
        scene_interfaces.push_back(ims::MoveitInterface(move_group_name));
    }

    // Create a move group interface for the multi-agent.
    //  find the move group prefix, add a "_multi_arm" suffix and create a move group.
    std::string move_group_multi_name = "panda_multi_arm";
    moveit::planning_interface::MoveGroupInterface move_group_multi(node, move_group_multi_name);


    ///////////////////////////////////
    // Start and goal states. Both in radians.
    ///////////////////////////////////
    std::vector<StateType> start_states;
    std::vector<StateType> goal_states;

    // Get the current state of each agent to set as start.
    std::vector<moveit::core::RobotStatePtr> current_states;
    for (std::string& move_group_name : move_group_names) {
        moveit::planning_interface::MoveGroupInterface move_group(node, move_group_name);
        current_states.push_back(move_group.getCurrentState());
    }

    // Set the start and goal states.
    for (int i = 0; i < move_group_names.size(); i++) {
        StateType start_state = std::vector<double>(num_joints, 0.0);
        scene_interfaces[i].getCurrentState(start_state);
        // Round to discretization.
        ims::roundStateToDiscretization(start_state, discretization);
        // Add time.
        start_state.push_back(0);
        start_states.push_back(start_state);
    }


    // Create a common goal for the sake of the example.
    StateType goal_state{0, -29, 0, -85, 0, 57, 0, -1};
    // Convert to radians.
    ims::deg2rad(goal_state, state_valid_mask);
    ims::roundStateToDiscretization(goal_state, discretization);
    for (int i = 0; i < start_states.size(); i++){
        goal_states.push_back(goal_state);
    }

    // Show the bounding box of the distance field.
    /*
    std::string move_group_name0 = move_group_names[0];
    moveit::planning_interface::MoveGroupInterface move_group0(node, move_group_name0);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bb_pub =
        node->create_publisher<visualization_msgs::msg::Marker>("bounding_box", 1);
    ims::visualizeBoundingBox(df, bb_pub, move_group0.getPlanningFrame());
    */

    ///////////////////////////////////
    // Action Spaces.
    ///////////////////////////////////

    // Set up the action types.
    std::vector<ims::MrampManipulationActionType> action_types;
    for (std::string& mprim_file_path : mprim_file_paths) {
        auto action_type = ims::MrampManipulationActionType(mprim_file_path);
        action_type.Discretization(discretization);
        action_types.push_back(action_type);
    }

    // Set up the action spaces.
    for (int i = 0; i < move_group_names.size(); i++) {
        std::string move_group_name = move_group_names[i];
        auto* bfs_heuristic = new ims::BFSRemoveTimeHeuristic(df, move_group_name);
        bfs_heuristic->setGoal(goal_states[i]);

        action_spaces.push_back(std::make_shared<ims::MrampManipulationActionSpace>(
                scene_interfaces[i], 
                action_types[i], 
                bfs_heuristic));
    }

    ///////////////////////////////////
    // Planner.
    ///////////////////////////////////
    double weight_low_level_heuristic = 55.0;
    double low_level_focal_suboptimality =  1.3;
    double high_level_focal_suboptimality = 1.3;
    ims::EAECBSParams params_eaecbs;
    params_eaecbs.weight_low_level_heuristic = weight_low_level_heuristic;
    params_eaecbs.low_level_focal_suboptimality = low_level_focal_suboptimality;
    params_eaecbs.high_level_focal_suboptimality = high_level_focal_suboptimality;

    params_eaecbs.low_level_heuristic_ptrs;
    for (size_t i = 0; i < move_group_names.size(); i++) {
        params_eaecbs.low_level_heuristic_ptrs.push_back(new ims::EuclideanRemoveTimeHeuristic());
    }
    // Initialize the planner.
    ims::MultiAgentPaths paths;
    PlannerStats stats;
    ims::EAECBS planner(params_eaecbs);
    planner.initializePlanner(action_spaces, move_group_names, start_states, goal_states);
    if (!planner.plan(paths)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan.");
        return 0;
    }

    // Print the stats.
    stats = planner.reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
    std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;

    ///////////////////////////////////
    // Move the robots.
    ///////////////////////////////////
    // Create a composite-state trajectory.
    std::vector<StateType> path_composite;
    MultiAgentPaths ids_and_paths;
    for (int i{0}; i < move_group_names.size(); ++i) {
        std::pair<int, PathType> id_and_path;
        id_and_path.first = i;
        id_and_path.second = paths[i];
        ids_and_paths.insert(id_and_path);
    }

    // Pad the paths to the same length.
    ims::padPathsToMaxLength(ids_and_paths);

    // Create a composite path.
    for (int i{0}; i < paths[0].size(); ++i) {
        StateType composite_state;
        for (int agent_id{0}; agent_id < move_group_names.size(); ++agent_id) {
            PathType agent_path = ids_and_paths[agent_id];

            composite_state.insert(composite_state.end(), agent_path[i].begin(), agent_path[i].end() -1);
        }
        path_composite.push_back(composite_state);
    }

    // Execute the path.
    // A composite start state.
    StateType start_state_composite;
    for (int i{0}; i < move_group_names.size(); ++i) {
        start_state_composite.insert(start_state_composite.end(), start_states[i].begin(), start_states[i].end() -1);
    }

    // A composite goal state.
    StateType goal_state_composite;
    for (int i{0}; i < move_group_names.size(); ++i) {
        goal_state_composite.insert(goal_state_composite.end(), goal_states[i].begin(), goal_states[i].end() - 1);
    }

    // Profile and execute the path.
    moveit_msgs::msg::RobotTrajectory trajectory;
    ims::profileTrajectory(start_state_composite,
                      goal_state_composite,
                      path_composite,
                      move_group_multi,
                      trajectory);

    RCLCPP_INFO(node->get_logger(), "Executing trajectory");
    move_group_multi.execute(trajectory);

    RCLCPP_INFO_STREAM(node->get_logger(), GREEN << "DONE SCRIPT" << RESET);/*

    // /*



    // profile and execute the path
    // @{
    std::vector<StateType> traj;
    for (auto& state : path_) {
        traj.push_back(state);
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    ims::profileTrajectory(start_state,
                      goal_state,
                      traj,
                      move_group,
                      trajectory);

    RCLCPP_INFO(node->get_logger(), "Executing trajectory");
    move_group.execute(trajectory);
    // @}
    */
    return 0;
}
