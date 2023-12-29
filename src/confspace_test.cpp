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
 * \file   confspace_test.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/18/23
*/

// C++ includes
#include <memory>
#include <iostream>
#include <boost/filesystem.hpp>

// project includes
#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include <manipulation_planning/action_space/manipulation_constrained_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/wastar.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <manipulation_planning/common/utils.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

int main(int argc, char** argv) {

    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("confspace_test_node", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();


    // Create a ROS logger
    auto const logger = rclcpp::get_logger("confspace_test_node");

    // Get the ros param for group_name.
    std::string group_name;
    node->get_parameter_or("group_name", group_name, std::string("panda0_arm"));

    double discret = 5;

    // if (argc == 0) {
    //     RCLCPP_INFO_STREAM(node->get_logger(), BOLDMAGENTA << "No arguments given: using default values");
    //     RCLCPP_INFO_STREAM(node->get_logger(), "<group_name(string)> <discretization(int)> <save_experience(bool int)>");
    //     RCLCPP_INFO_STREAM(node->get_logger(), "Using default values: manipulator_1 1 0" << RESET);
    // } else if (argc == 2) {
    //     group_name = argv[1];
    // } else if (argc == 3) {
    //     group_name = argv[1];
    //     discret = std::stod(argv[2]);
    // } else if (argc == 4) {
    //     group_name = argv[1];
    //     discret = std::stod(argv[2]);
    //     save_experience = std::stoi(argv[3]);
    // } else {
    //     RCLCPP_INFO_STREAM(node->get_logger(), BOLDMAGENTA << "No arguments given: using default values");
    //     RCLCPP_INFO_STREAM(node->get_logger(), "<group_name(string)> <discretization(int)> <save_experience(bool int)>" );
    //     RCLCPP_INFO_STREAM(node->get_logger(), "Using default values: manipulator_1 1 0" << RESET);
    // }

    // Print the arguments.
    RCLCPP_INFO_STREAM(node->get_logger(), "Using group: " << group_name);
    RCLCPP_INFO_STREAM(node->get_logger(), "Using discretization: " << discret);


    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);

    // get the number of joints
    int num_joints = (int)move_group.getVariableCount();

    std::string path_mprim = full_path.string() + "/config/manip_" + std::to_string(num_joints) + "dof.mprim";

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // check for collision
    planning_scene::PlanningScenePtr planning_scene;
    planning_scene.reset(new planning_scene::PlanningScene(move_group.getRobotModel()));
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene->checkCollision(collision_request, collision_result, *current_state);

    auto df = ims::getDistanceFieldMoveIt();
    // show the bounding box of the distance field
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bb_pub =
        node->create_publisher<visualization_msgs::msg::Marker>("bounding_box", 1);

    ims::MoveitInterface scene_interface(group_name);

    // get the planning frame
    // ims::visualizeBoundingBox(df, bb_pub, move_group.getPlanningFrame());
    auto* heuristic = new ims::BFSHeuristic(df, group_name);
//    auto* heuristic = new ims::JointAnglesHeuristic;
    double weight = 100.0;

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
    goal_state[0] = 90;

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
    return 0;
}
