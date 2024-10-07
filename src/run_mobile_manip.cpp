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
 * \file   run_mobile_manip.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   10/6/24
*/

// C++ includes
#include <memory>

// project includes
#include <manipulation_planning/action_space/mobile_manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/wastar.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <manipulation_planning/common/utils.hpp>

// ROS includes
#include <ros/ros.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>



int main(int argc, char** argv) {

    ros::init(argc, argv, "mobile_manip_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    std::string group_name = "base_l_arm";
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
        ROS_INFO_STREAM("Using default values: base_l_arm 1 0" << RESET);
    }

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();

    // Define Robot interface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    // get the number of joints
    int num_joints = static_cast<int>(move_group.getVariableCount());

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // check for collision
    planning_scene::PlanningScenePtr planning_scene;
    planning_scene = std::make_shared<planning_scene::PlanningScene>(move_group.getRobotModel());
    std::vector<std::string> nmame;
    planning_scene->getCollisionDetectorNames(nmame);
    // planning_scene->getCollisionDetectorNames()
    // auto cc = collision_detection::CollisionDetectorAllocatorBullet::create();
    // planning_scene->addCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    // planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                     // true /* exclusive */);
    // planning_scene->setActiveCollisionDetector(cc, false);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene->checkCollision(collision_request, collision_result, *current_state);

    auto df = ims::getDistanceFieldMoveIt(
        10, 10, 3, 0.02, -5.0,-5.0, 0.0, 10);
    // show the bounding box of the distance field
    ros::Publisher bb_pub = nh.advertise<visualization_msgs::Marker>("bb_marker", 10);
    // get the planning frame
    ims::visualizeBoundingBox(df, bb_pub, move_group.getPlanningFrame());
    auto* heuristic = new ims::BFSHeuristic(df, group_name);
    ims::visualizeOccupancy(df, bb_pub, move_group.getPlanningFrame());
//    auto* heuristic = new ims::JointAnglesHeuristic;
    double weight = 100.0;

    ims::wAStarParams params(heuristic, weight);

    // ims::MoveitInterface scene_interface(group_name, planning_scene);
    ims::MoveitInterface scene_interface(group_name);

    ims::MobileManipulationType action_type;

//    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);
    std::shared_ptr<ims::MobileManipulationActionSpace> action_space =
        std::make_shared<ims::MobileManipulationActionSpace>(scene_interface, action_type, heuristic);
    StateType start_state = StateType(num_joints, 0);
    const std::vector<std::string>& joint_names = move_group.getVariableNames();
    for (int i = 0; i < num_joints; i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
        ROS_INFO_STREAM("Joint " << joint_names[i] << " is " << start_state[i]);
    }
    // make a goal_state a copy of start_state
    StateType goal_state = start_state;

    // change the goal state
    goal_state[0] = 1.675;
    goal_state[1] = 0.126;
    goal_state[2] = 60.0 * M_PI / 180.0;
    goal_state[3] = 0.3;
    goal_state[4] = 1.5;

    // normalize the start and goal states
    // get the joint limits
    std::vector<std::pair<double, double>> joint_limits;
    scene_interface.getJointLimits(joint_limits);
    std::vector<bool> mask(num_joints, true);
    mask[0] = false; mask[1] = false; mask[2] = false; mask[3] = false;
    ims::normalizeAngles(start_state, joint_limits, mask);
    ims::normalizeAngles(goal_state, joint_limits, mask);
    // create a deep copy of the start and goal states
    StateType start_state_copy = start_state;
    StateType goal_state_copy = goal_state;
    ims::roundStateToDiscretization(start_state, action_space->mobile_manipulation_type_->state_discretization_);
    ims::roundStateToDiscretization(goal_state, action_space->mobile_manipulation_type_->state_discretization_);

    ims::wAStar planner(params);
    try {
        planner.initializePlanner(action_space, start_state, goal_state);
    }
    catch (std::exception& e) {
        ROS_INFO_STREAM(e.what() << std::endl);
    }

    std::vector<StateType> path_;
    if (!planner.plan(path_)) {
        ROS_INFO_STREAM(RED << "No path found" << RESET);
        return 1;
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
            file << "Experience," << path_.size() << "," << num_joints << std::endl;
            // write the path
            for (int j {0}; j < path_.size(); j++){
                for (int i {0}; i < num_joints; i++) {
                    file << path_[j][i] << ",";
                }
                file << std::endl;
            }
            file.close();
        }
    }

    // add the true start and goal states to the path
    path_.insert(path_.begin(), start_state_copy);
    path_.push_back(goal_state_copy);
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
    ROS_INFO_STREAM("\n" << GREEN << "\t Planning time: " << stats.time << " sec" << std::endl
                    << "\t cost: " << stats.cost << std::endl
                    << "\t Path length: " << path_.size() << std::endl
                    << "\t Number of nodes expanded: " << stats.num_expanded << std::endl
                    << "\t Suboptimality: " << stats.suboptimality << RESET);

    // profile and execute the path
    // @{
    std::vector<StateType> traj;
    ims::shortcutPath(path_,
        move_group,
        planning_scene,
        traj, 0.5);

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
