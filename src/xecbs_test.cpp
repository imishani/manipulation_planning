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
    // Search Action Spaces.
    ///////////////////////////////////

    // The relevant moveit groups.
    std::vector<std::string> move_group_names = {"panda0_arm", "panda1_arm", "panda2_arm", "panda3_arm"};
    std::vector<std::string> agent_names = {"panda0", "panda1", "panda2", "panda3"};
    std::vector<std::shared_ptr<ims::ManipulationConstrainedActionSpace>> action_spaces;

    // Get the motion primitives file paths.
    std::vector<std::string> mprim_file_paths;
    for (std::string& move_group_name : move_group_names) {
        mprim_file_paths.push_back(getMPrimFilePathFromMoveGroupName(node, move_group_name));
    }

    // Get the distance field of the scene for the BFS heuristic (used for snap).
    auto df = ims::getDistanceFieldMoveIt();

    // Keep the copies of the scene interfaces.
    std::vector<std::shared_ptr<ims::MoveitInterface>> scene_interfaces;
    for (std::string& move_group_name : move_group_names) {
        scene_interfaces.push_back(std::make_shared<ims::MoveitInterface>(move_group_name));
    }

    // Show the bounding box of the distance field.
    /*
    std::string move_group_name0 = move_group_names[0];
    moveit::planning_interface::MoveGroupInterface move_group0(node, move_group_name0);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bb_pub =
        node->create_publisher<visualization_msgs::msg::Marker>("bounding_box", 1);
    ims::visualizeBoundingBox(df, bb_pub, move_group0.getPlanningFrame());
    */

   // Set up the BFS heuristics.
    std::vector<std::shared_ptr<ims::BFSRemoveTimeHeuristic>> bfs_heuristics;
    for (std::string& move_group_name : move_group_names) {
        bfs_heuristics.push_back(std::make_shared<ims::BFSRemoveTimeHeuristic>(df, move_group_name));
    }

    // Set up the action types.
    StateType discretization{1, 1, 1, 1, 1, 1, 1, 1};  // In Degrees, with the last element is time in time-units.
    ims::deg2rad(discretization, std::vector<bool>{true, true, true, true, true, true, true, false});
    std::vector<ims::ManipulationType> action_types;
    for (std::string& mprim_file_path : mprim_file_paths) {
        auto action_type = ims::ManipulationType(mprim_file_path);
        action_type.Discretization(discretization);
        action_types.push_back(ims::ManipulationType(mprim_file_path));
    }

    // Set up the action spaces.
    for (int i = 0; i < move_group_names.size(); i++) {
        action_spaces.push_back(std::make_shared<ims::ManipulationConstrainedActionSpace>(
            scene_interfaces[i], action_types[i], bfs_heuristics[i]));
    }





    RCLCPP_INFO_STREAM(node->get_logger(), GREEN << "DONE SCRIPT" << RESET);/*

    // /*

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    auto* heuristic = new ims::BFSHeuristic(df, group_name);
//    auto* heuristic = new ims::JointAnglesHeuristic;
    double weight = 40.0;
    ims::wAStarParams params(heuristic, weight);


    ims::ManipulationType action_type (path_mprim);
    StateType discretization(num_joints, discret);
    ims::deg2rad(discretization);
    action_type.Discretization(discretization);

//    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);
    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type, heuristic);

    StateType start_state  = std::vector<double>(num_joints, 0.0);
    const std::vector<std::string>& joint_names = move_group.getActiveJoints();
    for (int i = 0; i < num_joints; i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
        RCLCPP_INFO_STREAM(node->get_logger(), "Joint " << joint_names[i] << " is " << start_state[i]);
    }
    // make a goal_state a copy of start_state
    ims::rad2deg(start_state);
    StateType goal_state = start_state;

    // change the goal state
    goal_state[0] = 70;
    goal_state[5] = 166;


    ims::deg2rad(start_state); ims::deg2rad(goal_state);
    // normalize the start and goal states
    // get the joint limits
    std::vector<std::pair<double, double>> joint_limits;
    scene_interface.getJointLimits(joint_limits);
    ims::normalizeAngles(start_state, joint_limits);
    ims::normalizeAngles(goal_state, joint_limits);
    ims::roundStateToDiscretization(start_state, action_type.state_discretization_);
    ims::roundStateToDiscretization(goal_state, action_type.state_discretization_);
    
    ims::wAStar planner(params);
    try {
        planner.initializePlanner(action_space, start_state, goal_state);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    std::vector<StateType> path_;
    if (!planner.plan(path_)) {
        RCLCPP_ERROR(node->get_logger(), "No path found");
        return 0;
    }
    else {
        RCLCPP_INFO_STREAM(node->get_logger(), GREEN << "Path found" << RESET);
    }

    // Print the path.
    int counter = 0;
    for (auto& state : path_) {
        std::cout << "State: " << counter++ << ": ";
        for (auto& val : state) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }

    // report stats
    PlannerStats stats = planner.reportStats();
    RCLCPP_INFO_STREAM(node->get_logger(), "\n" << GREEN << "\t Planning time: " << stats.time << " sec" << std::endl
                    << "\t cost: " << stats.cost << std::endl
                    << "\t Path length: " << path_.size() << std::endl
                    << "\t Number of nodes expanded: " << stats.num_expanded << std::endl
                    << "\t Suboptimality: " << stats.suboptimality << RESET);

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
