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
 * \file   mramp_test.cpp
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
#include <search/planners/multi_agent/cbs_sphere3d.hpp>
#include <search/planners/wastar.hpp>
#include <search/planners/multi_agent/cbs_sipp.hpp>
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


class MrampTestNode : public rclcpp::Node {
public:
    MrampTestNode() : Node("mramp_test_node") {

        // Accept parameter planner_name.
        // rclcpp::NodeOptions node_options;
        // node_options.automatically_declare_parameters_from_overrides(true);
        this->declare_parameter("planner_name", "planner_name_default");
        planner_name_ = this->get_parameter("planner_name").as_string();
        RCLCPP_INFO_STREAM(this->get_logger(), "planner_name: " << planner_name_);
    }

    ~MrampTestNode() {
        rclcpp::shutdown();
    }

    std::string getPlannerName() {
        return planner_name_;
    }

private:
    std::string planner_name_;
};

int main(int argc, char** argv) {

    ///////////////////////////////////
    // ROS setup.
    ///////////////////////////////////
    rclcpp::init(argc, argv);
    // We create a node to get parameters.
    auto node = std::make_shared<MrampTestNode>();
    std::string planner_name_test;
    // rclcpp::spin(node);

    // Spin.
    rclcpp::executors::SingleThreadedExecutor executor_test;
    executor_test.add_node(node);
    std::thread([&executor_test]() { executor_test.spin(); }).detach();

    // Run the test.
    // Create a ROS logger
    auto const logger = node->get_logger();

    ///////////////////////////////////
    // Scene Interfaces.
    ///////////////////////////////////
    StateType discretization{1, 1, 1, 1, 1, 1, 1, 1};  // In Degrees, with the last element is time in time-units.
    std::vector<bool> state_valid_mask{true, true, true, true, true, true, true, false};
    ims::deg2rad(discretization, state_valid_mask); // In Radians, the last element is time in time-units.
    int num_joints = 7;

    // The relevant moveit groups.
    std::vector<std::string> move_group_names = {"panda0_arm", "panda1_arm", "panda2_arm", "panda3_arm"};
    std::vector<std::string> move_group_names_ee = {"panda0_hand", "panda1_hand", "panda2_hand", "panda3_hand"};
    std::vector<std::string> robot_names = {"panda0", "panda1", "panda2", "panda3"};
    std::vector<std::shared_ptr<ims::SubcostConstrainedActionSpace>> action_spaces;
    std::vector<std::shared_ptr<ims::MrampManipulationActionSpace>> action_spaces_mramp;

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

    // Set the end effector names for each scene interface.
    for (int i = 0; i < move_group_names.size(); i++) {
        scene_interfaces[i].setEndEffectorMoveGroupName(move_group_names_ee[i]);
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
    std::vector<moveit::core::RobotStatePtr> current_moveit_robot_states;
    for (int i =0; i < move_group_names.size(); i++){
        moveit::core::RobotStatePtr current_state;
        scene_interfaces.at(i).getCurrentRobotStateMoveIt(current_state);

        current_moveit_robot_states.push_back(current_state);
    }

    // Get the current state of each agent end-effector.

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


    // Create a goal state each robot.
    // StateType goal_state0{0, -41, 0, -109, 0, 159, 0, -1};
    StateType goal_state0{0, -5, 0, -156, 0, 152, 0, -1};
    StateType goal_state1{9, 7, -25, -91, 77, 101, -13, -1};
    // StateType goal_state2{0, -41, 0, -109, 0, 159, 0, -1};
    StateType goal_state2{0, -41, 0, -109, 0, 159, 0, -1};
    StateType goal_state3{4, 0, -18, -93, 65, 111, -10, -1};
    goal_states = {goal_state0, goal_state1, goal_state2, goal_state3};
    // Convert to radians.
    for (StateType& goal_state : goal_states) {
        ims::deg2rad(goal_state, state_valid_mask);
        ims::roundStateToDiscretization(goal_state, discretization);
    }


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

        action_spaces_mramp.push_back(std::make_shared<ims::MrampManipulationActionSpace>(
                scene_interfaces[i], 
                action_types[i], 
                bfs_heuristic));
    }

    // Keep copies of action spaces casted to ConstrainedActionSpace (for CBS).
    std::vector<std::shared_ptr<ims::ConstrainedActionSpace>> action_spaces_constrained;
    for (auto& action_space : action_spaces) {
        action_spaces_constrained.push_back(std::dynamic_pointer_cast<ims::ConstrainedActionSpace>(action_space));
    }

    ///////////////////////////////////
    // Planner.
    ///////////////////////////////////
    std::string planner_name; // = "CBS_Sphere3d";
    // std::string planner_name = "CBS";

    // Get the name of the planner from a ros param.
    node->get_parameter("planner_name", planner_name);
    
    ims::MultiAgentPaths paths;
    PlannerStats stats;

    if (planner_name == "xECBS") {
            
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
        ims::EAECBS planner(params_eaecbs);
        planner.initializePlanner(action_spaces, move_group_names, start_states, goal_states);
        if (!planner.plan(paths)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan.");
            return 0;
        }
        stats = planner.reportStats();
    }

    else if (planner_name == "CBS"){
        // Set the parameters.
        ims::CBSParams params_cbs;
        for (size_t i = 0; i < move_group_names.size(); i++) {
            params_cbs.low_level_heuristic_ptrs.push_back(new ims::EuclideanRemoveTimeHeuristic());
        }
        params_cbs.weight_low_level_heuristic = 55.0;
        params_cbs.time_limit_ = 1000.0;

        // Initialize the planner.
        ims::CBS planner(params_cbs);
        planner.initializePlanner(action_spaces_constrained, move_group_names, start_states, goal_states);
        if (!planner.plan(paths)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan.");
            return 0;
        }
        stats = planner.reportStats();
    }


    else if (planner_name == "CBS_Sphere3d"){
        // Set the parameters.
        ims::CBSSphere3dParams params_cbs;
        for (size_t i = 0; i < move_group_names.size(); i++) {
            params_cbs.low_level_heuristic_ptrs.push_back(new ims::EuclideanRemoveTimeHeuristic());
        }
        params_cbs.weight_low_level_heuristic = 55.0;
        params_cbs.sphere3d_constraint_radius = 0.1;

        // Initialize the planner.
        ims::CBSSphere3d planner(params_cbs);
        planner.initializePlanner(action_spaces_constrained, move_group_names, start_states, goal_states);
        if (!planner.plan(paths)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan.");
            return 0;
        }
        stats = planner.reportStats();
    }

    else if (planner_name == "CBS_SIPP"){
        // Set the parameters.
        ims::CBSSIPPParams params_cbs;
        for (size_t i = 0; i < move_group_names.size(); i++) {
            params_cbs.low_level_heuristic_ptrs.push_back(new ims::EuclideanRemoveTimeHeuristic());
        }
        params_cbs.weight_low_level_heuristic = 55.0;
        params_cbs.time_limit_ = 10.0;
        params_cbs.verbose = false;

        // Initialize the planner.
        ims::CBSSIPP planner(params_cbs);
        planner.initializePlanner(action_spaces_constrained, move_group_names, start_states, goal_states);
        if (!planner.plan(paths)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan.");
            return 0;
        }
        stats = planner.reportStats();
    }

    // Print the stats.
    std::cout << GREEN << "Planner: " << planner_name << RESET << "\n";
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
    std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;

    ///////////////////////////////////
    // Move the robots.
    ///////////////////////////////////

    RCLCPP_INFO_STREAM(node->get_logger(), "Shortcutting paths.");
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    MultiAgentPaths smoothed_paths;
    std::unordered_map<int, std::string> robot_names_map;
    for (size_t i = 0; i < robot_names.size(); i++) {
        robot_names_map[i] = robot_names[i];
    }
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(move_group_multi.getRobotModel()));
    ims::shortcutMultiAgentPathsIterative(paths, move_group_multi, planning_scene, robot_names_map, smoothed_paths, 1.0);
    paths = smoothed_paths;

    // Create a composite-state trajectory.
    moveit_msgs::msg::RobotTrajectory trajectory;
    int num_agents = paths.size();
    std::vector<StateType> path_composite;
    MultiAgentPaths ids_and_paths;
    for (int i{0}; i < num_agents; ++i) {
        std::pair<int, PathType> id_and_path;
        id_and_path.first = i;
        id_and_path.second = paths.at(i);
        ids_and_paths.insert(id_and_path);
    }

    // Pad the paths to the same length.
    ims::padPathsToMaxLength(ids_and_paths);
    int T = ids_and_paths[0].size();

    // Create a composite path.
    for (int i{0}; i < T; ++i) {
        StateType composite_state;
        for (int agent_id{0}; agent_id < num_agents; ++agent_id) {
            PathType agent_path = ids_and_paths[agent_id];

            composite_state.insert(composite_state.end(), agent_path[i].begin(), agent_path[i].end() -1);
        }
        path_composite.push_back(composite_state);
    }

    // Execute the path.
    // A composite start state.
    StateType start_state_composite;
    for (int i{0}; i < num_agents; ++i) {
        start_state_composite.insert(start_state_composite.end(), start_states[i].begin(), start_states[i].end() -1);
    }

    // A composite goal state.
    StateType goal_state_composite;
    for (int i{0}; i < num_agents; ++i) {
        goal_state_composite.insert(goal_state_composite.end(), goal_states[i].begin(), goal_states[i].end() - 1);
    }

    ims::profileTrajectory(start_state_composite,
                    goal_state_composite,
                    path_composite,
                    move_group_multi,
                    trajectory);

    RCLCPP_INFO(node->get_logger(), "Executing trajectory");
    // Allow for large steps.
    move_group_multi.execute(trajectory);

    RCLCPP_INFO_STREAM(node->get_logger(), GREEN << "DONE SCRIPT" << RESET);
    return 0;
}
