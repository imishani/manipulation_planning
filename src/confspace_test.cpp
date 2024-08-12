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
#include <chrono>

// project includes
#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/wastar.hpp>
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
    bool save_experience = false;

    if (argc == 0) {
        ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
        ROS_INFO_STREAM("<group_name(string)> <discretization(int)> <save_experience(bool int)>");
        ROS_INFO_STREAM("Using default values: manipulator_1 1 0" << RESET);
    } else if (argc == 2) {
        group_name = argv[1];
    } else if (argc == 3) {
        group_name = argv[1];
        discret = std::stod(argv[2]);
    } else if (argc == 4) {
        group_name = argv[1];
        discret = std::stod(argv[2]);
        save_experience = std::stoi(argv[3]);
    } else {
        ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
        ROS_INFO_STREAM("<group_name(string)> <discretization(int)> <save_experience(bool int)>" );
        ROS_INFO_STREAM("Using default values: manipulator_1 1 0" << RESET);
    }

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    // get the number of joints
    int num_joints = (int)move_group.getVariableCount();

    std::string path_mprim = full_path.string() + "/config/manip_" + std::to_string(num_joints) + "dof_mprim.yaml";

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
    auto* heuristic = new ims::BFSHeuristic(df, group_name);
//    auto* heuristic = new ims::JointAnglesHeuristic;
    double weight = 100.0;

    ims::wAStarParams params(heuristic, weight);

    ims::MoveitInterface scene_interface(group_name);

    ims::ManipulationType action_type (path_mprim);
    StateType discretization(num_joints, discret);
    ims::deg2rad(discretization);
    action_type.Discretization(discretization);

//    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);
    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type,
                                                                                                                heuristic);

    const std::vector<std::string>& joint_names = move_group.getVariableNames();
    StateType start_state(num_joints, 0);

    for (int i = 0; i < joint_names.size(); i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
        ROS_INFO_STREAM("Joint " << joint_names[i] << " is " << start_state[i]);
    }
    // make a goal_state a copy of start_state
    ims::rad2deg(start_state);
    StateType goal_state = start_state;

    // change the goal state
    goal_state[0] = 1.9708*180/M_PI;// 78; //0;
//    goal_state[1] = 0.0698132*180/M_PI; //25; //30;
//    goal_state[2] = -0.9948*180/M_PI; //-18; //-30;
    goal_state[3] -= 40; // = -1.5708*180/M_PI; //-147; //0;
//    goal_state[4] = 0; //73; //0;
    goal_state[5] += 10;//-66; //0;


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
        ROS_INFO_STREAM(e.what() << std::endl);
    }

    std::vector<StateType> path;
    if (!planner.plan(path)) {
        ROS_INFO_STREAM(RED << "No path found" << RESET);
        return 0;
    }
    else {
        ROS_INFO_STREAM(GREEN << "Path found" << RESET);
        if (save_experience) {
            ROS_INFO("Saving path as experience");
            // check if the directory exists
            boost::filesystem::path dir(full_path.string() + "/data/experiences/" + group_name);
            if (boost::filesystem::is_directory(dir)) {
                ROS_INFO_STREAM("Directory " << dir << " exists");
            } else {
                ROS_INFO_STREAM("Directory " << dir << " does not exist");
                ROS_INFO_STREAM("Creating directory " << dir);
                boost::filesystem::create_directory(dir);
            }
            // check how many experiences in the directory
            int num_experiences = 0;
            for (auto& p : boost::filesystem::directory_iterator(dir)) {
                num_experiences++;
            }
            // Save the path to a file as csv
            std::string path_file = dir.string() + "/path_" +
                std::to_string(num_experiences + 1) + ".csv";

            std::ofstream file(path_file);
            // header line
            file << "Experience," << path.size() << "," << num_joints << std::endl;
            // write the path
            for (int j {0}; j < path.size(); j++){
                for (int i {0}; i < num_joints; i++) {
                    file << path[j][i] << ",";
                }
                file << std::endl;
            }
            file.close();
        }
    }

    // Shortcut the path.
    // Timing.
    auto shortcutting_startt = std::chrono::high_resolution_clock::now();
    PathType smoothed_path;
    ims::shortcutPath(path, move_group, scene_interface.getPlanningSceneMoveit(), smoothed_path);
    path = smoothed_path;
    auto shortcutting_endt = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> shortcutting_time = shortcutting_endt - shortcutting_startt;
    ROS_INFO_STREAM("Shortcutting time: " << shortcutting_time.count() << " sec");

    // Print nicely the path
    int counter = 0;
    for (auto& state : path) {
        std::cout << "State: " << counter++ << ": ";
        for (auto& val : state) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }

    // report stats
    PlannerStats stats = planner.reportStats();
    ROS_INFO_STREAM("\n" << GREEN << "\t Planning time: " << stats.time << " sec" << std::endl
                    << "\t cost: " << stats.cost << std::endl
                    << "\t Path length: " << path.size() << std::endl
                    << "\t Number of nodes expanded: " << stats.num_expanded << std::endl
                    << "\t Suboptimality: " << stats.suboptimality << RESET);

    // profile and execute the path
    // @{
    std::vector<StateType> traj;
    for (auto& state : path) {
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
