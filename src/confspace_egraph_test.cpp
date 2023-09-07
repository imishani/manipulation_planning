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
 * \file   confspace_egraph_test.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/12/23
*/

// C++ includes
#include <memory>

// project includes
#include <manipulation_planning/action_space/egraph_manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/egraph_wastar.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <manipulation_planning/common/utils.hpp>

// ROS includes
#include <ros/ros.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "configuration_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    std::string group_name = "manipulator_1";
    double discret = 1;

    if (argc == 0) {
        ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
        ROS_INFO_STREAM("<group_name(string)> <discretization(int)> ");
        ROS_INFO_STREAM("Using default values: manipulator_1 1" << RESET);
    } else if (argc == 2) {
        group_name = argv[1];
    } else if (argc == 3) {
        group_name = argv[1];
        discret = std::stod(argv[2]);
    } else {
        ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
        ROS_INFO_STREAM("<group_name(string)> <discretization(int)> ");
        ROS_INFO_STREAM("Using default values: manipulator_1 1" << RESET);
    }

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
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
    ros::Publisher bb_pub = nh.advertise<visualization_msgs::Marker>("bb_marker", 10);
    // get the planning frame
    ims::visualizeBoundingBox(df, bb_pub, move_group.getPlanningFrame());

    auto* heuristic = new ims::BFSHeuristicEgraph;
    double weight = 100.0;

    ims::ExperienceWAStarParams params(heuristic, weight, 100.0,
                                       full_path.string() + "/data/experiences/" + group_name);

    ims::MoveitInterface scene_interface(group_name);

    ims::ManipulationType action_type (path_mprim);
    StateType discretization(num_joints, discret);
    ims::deg2rad(discretization);
    action_type.Discretization(discretization);

//    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);
    std::shared_ptr<ims::EgraphManipulationActionSpace> action_space =
        std::make_shared<ims::EgraphManipulationActionSpace>(scene_interface, action_type,heuristic);

    heuristic->init(action_space, df, group_name);

    StateType start_state {0, 0, 0, 0, 0, 0};
    const std::vector<std::string>& joint_names = move_group.getVariableNames();
    int num_joints_ = (int)move_group.getVariableCount();
    for (int i = 0; i < num_joints; i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
        ROS_INFO_STREAM("Joint " << joint_names[i] << " is " << start_state[i]);
    }
    // make a goal_state a copy of start_state
    ims::rad2deg(start_state);
    StateType goal_state = start_state;

    // change the goal state
    goal_state[0] = 5;// 78; //0;
    goal_state[1] = 30; //25; //30;
    goal_state[2] = -39; //-18; //-30;
    goal_state[3] = 0; //-147; //0;
    goal_state[4] = 0; //73; //0;
    goal_state[5] = 0;//-66; //0;


    ims::deg2rad(start_state); ims::deg2rad(goal_state);
    // normalize the start and goal states
    // get the joint limits
    std::vector<std::pair<double, double>> joint_limits;
    scene_interface.getJointLimits(joint_limits);
    ims::normalizeAngles(start_state, joint_limits);
    ims::normalizeAngles(goal_state, joint_limits);
    ims::roundStateToDiscretization(start_state, action_type.state_discretization_);
    ims::roundStateToDiscretization(goal_state, action_type.state_discretization_);

    ims::ExperienceWAstar planner(params);
    try {
        planner.initializePlanner(action_space, start_state, goal_state);
    }
    catch (std::exception& e) {
        ROS_INFO_STREAM(e.what() << std::endl);
    }

    std::vector<StateType> path_;
    if (!planner.plan(path_)) {
        ROS_INFO_STREAM(RED << "No path found" << RESET);
        return 0;
    }
    else
        ROS_INFO_STREAM(GREEN << "Path found" << RESET);

    // Print nicely the path
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
    ROS_INFO_STREAM("\n" << GREEN << "\t Planning time: " << stats.time << " sec" << std::endl <<
                    "\t cost: " << stats.cost << std::endl <<
                    "\t Path length: " << path_.size() << std::endl <<
                    "\t Number of nodes expanded: " << stats.num_expanded << std::endl <<
                    "\t Suboptimality: " << stats.suboptimality << RESET);

    // profile and execute the path
    // @{
    std::vector<StateType> traj;
    for (auto& state : path_) {
        traj.push_back(state);
    }
    moveit_msgs::RobotTrajectory trajectory;
    ims::profileTrajectory(start_state,
                           goal_state,
                           traj,
                           move_group,
                           trajectory);

    ROS_INFO("Executing trajectory");
    move_group.execute(trajectory);
    // @}
    return 0;
}
