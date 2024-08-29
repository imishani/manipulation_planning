/*
 * Copyright (C) 2024, Hanlan Yang
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
 * \file   confspace_edge_test.cpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   8/10/24
 */

// C++ includes
#include <memory>

// project includes
#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include <manipulation_planning/action_space/manipulation_edge_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <manipulation_planning/common/utils.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <search/planners/parallel_search/epase.hpp>
#include <search/planners/parallel_search/pase.hpp>
#include <search/planners/parallel_search/qpase.hpp>
#include <search/planners/wastar.hpp>

// ROS includes
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <ros/ros.h>

#define EXPERIMENT 0
// #define EXPERIMENT 10

/// @brief Load the start and goal configuration from file
/// @param starts The start configurations
/// @param goals The goal configurations
/// @param num_runs The number of runs
///@param path The path to the file
void loadStartsGoalsFromFile(std::vector<std::vector<double>>& starts, std::vector<std::vector<double>>& goals, int num_runs, const std::string& path) {
    std::ifstream starts_fin(path + "panda_starts.txt");
    std::ifstream goals_fin(path + "panda_goals.txt");

    for (int j = 0; j < num_runs; ++j) {
        std::vector<double> start, goal;
        double val_start, val_goal;
        for (int i = 0; i < 7; ++i) {
            starts_fin >> val_start;
            goals_fin >> val_goal;
            start.push_back(val_start);
            goal.push_back(val_goal);
        }

        starts.emplace_back(start);
        goals.emplace_back(goal);
    }
}

/// @brief Log the statistics of a parallel planner to a file.
/// @param stats The statistics of the planner.
/// @param problem_ind The index of the problem.
/// @param planner_name The name of the planner.
void logStats(const std::map<int, ims::ParallelSearchPlannerStats>& stats,
              int map_index,
              const std::string& planner_name) {
    // save the logs to a file in current directory
    std::string log_file = "/home/hanlany/code/manipulation_ws/logs/exp/logs_prob" + std::to_string(map_index) + "_" + planner_name + ".csv";
    // save logs object to a file
    std::ofstream file(log_file);
    // header line
    file << "Problem,Time,Cost,Num_expanded,Num_evaluated" << std::endl;
    for (auto& log : stats) {
        file << log.first << "," << log.second.time << "," << log.second.cost << "," << log.second.num_expanded << "," << log.second.num_evaluated << std::endl;
    }
    file.close();
}

/// @brief Visualize the goal state
/// @param type The type of state (greedy, attractor, etc)
void VisualizePoint(ims::MoveitInterface& moveit_interface_, double x, double y, double z) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = moveit_interface_.planning_scene_->getPlanningFrame();
    marker.header.stamp = ros::Time();
    marker.ns = "graph";
    marker.id = 20000;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    // green
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;

    // Lifetime.
    marker.lifetime = ros::Duration(350.0);

    // visualize
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;
    ros::Duration(0.3).sleep();
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    ros::Duration(0.3).sleep();
    vis_pub_.publish(marker);
    ros::Duration(0.3).sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "configuration_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    std::string group_name = "manipulator_1";
    std::string planner_name = "wastar";
    bool verbose = false;
    int num_runs = 0;
    double discret = 1;
    bool save_experience = false;

    if (argc == 0) {
        ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
        ROS_INFO_STREAM("<group_name(string)> <planner_name(string)> <verbose(bool)> <num_runs(int)> <discretization(int)> <save_experience(bool int)>");
        ROS_INFO_STREAM("Using default values: manipulator_1 wastar 0 0 1 0" << RESET);
    }
    if (argc >= 2) {
        group_name = argv[1];
    }
    if (argc >= 3) {
        planner_name = argv[2];
    }
    if (argc >= 4) {
        verbose = std::stoi(argv[3]);
    }
    if (argc >= 5) {
        num_runs = std::stoi(argv[4]);
    }
    if (argc >= 6) {
        discret = std::stod(argv[5]);
    }
    if (argc >= 7) {
        save_experience = std::stoi(argv[6]);
    }

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    // get the number of joints
    int num_joints = (int)move_group.getVariableCount();

    std::string path_mprim = full_path.string() + "/config/manip_" + std::to_string(num_joints) + "dof_mprim.yaml";

    ROS_WARN_STREAM(path_mprim);

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
    auto heuristic = std::make_shared<ims::BFSHeuristic>(df, group_name);
    // auto heuristic = std::make_shared<ims::JointAnglesHeuristic>();
    auto a_heuristic = std::make_shared<ims::BFSHeuristic>(df, group_name);
    auto q_heuristic = std::make_shared<ims::JointAnglesHeuristic>();
    auto i_heuristic = std::make_shared<ims::JointAnglesHeuristic>();
    //    auto* heuristic = new ims::JointAnglesHeuristic;
    double weight = 100.0;
    // double i_weight = 3000*weight;
    double i_weight = 20 * weight;
    // int num_threads = 1;
    // int num_threads = 4;
    // int num_threads = 12;
    int num_threads = 14;
    // int num_threads = 24;

    int iter{1};
    std::map<int, ims::ParallelSearchPlannerStats> logs;
    if (num_runs > 0) {
        iter = num_runs;
    }

    std::string ros_path = ros::package::getPath("manipulation_planning");
    std::string scene_path = ros_path + "/domains/panda/scene1/";
    std::vector<StateType> starts;
    std::vector<StateType> goals;
    loadStartsGoalsFromFile(starts, goals, num_runs, scene_path);

    std::vector<StateType> traj;
    moveit_msgs::RobotTrajectory trajectory;
    for (int i{0}; i < iter; ++i) {
        std::shared_ptr<ims::PlannerParams> params = std::make_shared<ims::PlannerParams>();

        if (planner_name == "wastar") {
            params = std::make_shared<ims::wAStarParams>(heuristic.get(), weight);
            if (verbose)
                params->verbose = true;
        } else if (planner_name == "epase" || planner_name == "pase" || planner_name == "qpase") {
            params = std::make_shared<ims::ParallelSearchParams>(heuristic, i_heuristic, num_threads, weight, i_weight);
            params->time_limit_ = 10.0;
            if (verbose) {
                params->verbose = true;
                // std::dynamic_pointer_cast<ims::ParallelSearchParams>(params)->debug = true;
            }
        } else {
            ROS_ERROR_STREAM("Planner " << planner_name << " not recognized");
            return 0;
        }

        ims::MoveitInterface scene_interface(group_name);

        ims::ManipulationType action_type(path_mprim);
        StateType discretization(num_joints, discret);
        ims::deg2rad(discretization);
        action_type.Discretization(discretization);

        //    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);
        std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type,
                                                                                                                    heuristic.get());
        std::shared_ptr<ims::ManipulationEdgeActionSpace> edge_action_space = std::make_shared<ims::ManipulationEdgeActionSpace>(scene_interface, action_type,
                                                                                                                                 a_heuristic);
        edge_action_space->setVisualization(verbose);
        action_space->setVisualization(verbose);
        // std::shared_ptr<ims::ManipulationEdgeActionSpace> edge_action_space = std::make_shared<ims::ManipulationEdgeActionSpace>(scene_interface, action_type,
        //                                                                                                                          q_heuristic);

        // std::cout << "ROS path is : " << ros_path << std::endl;
        // At each emplace_back, use the full pathh and concatenate the map name
        // maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/hrt201n.map");
        // maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/den501d/den501d.map");
        // maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/den520d/den520d.map");
        // maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/ht_chantry.map");
        // maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/brc203d/brc203d.map");
        //
        // std::vector<std::string> starts_goals_path = {
        //     full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/",
        //     full_path.string() + "/../domains/2d_robot_nav/data/den501d/",
        //     full_path.string() + "/../domains/2d_robot_nav/data/den520d/",
        //     full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/",
        //     full_path.string() + "/../domains/2d_robot_nav/data/brc203d/",
        // };

        const std::vector<std::string>& joint_names = move_group.getVariableNames();
        StateType start_state(num_joints, 0);
        StateType goal_state = start_state;

        if (num_runs) {
            start_state = starts[i];
            goal_state = goals[i];
        } else {
            for (int i = 0; i < joint_names.size(); i++) {
                start_state[i] = current_state->getVariablePosition(joint_names[i]);
                ROS_INFO_STREAM("Joint " << joint_names[i] << " is " << start_state[i]);
            }
            goal_state[0] = 44;
            goal_state[1] = 7;
            goal_state[2] = 9;
            goal_state[3] = -30;
            goal_state[4] = 4;  // 73; //0;
            goal_state[5] = 34;
            goal_state[6] = 57;
        }

        // change the goal state
        // goal_state[0] = 1.5708*180/M_PI;// 78; //0;
        // goal_state[1] = 0.0698132*180/M_PI; //25; //30;
        // goal_state[2] = -0.9948*180/M_PI; //-18; //-30;
        // goal_state[3] = -1.5708*180/M_PI; //-147; //0;
        // goal_state[4] = 0; //73; //0;
        //  goal_state[5] += 10;//-66; //0;

        // Start state to investigate
        // 9 85 86 -36 164 91 -104

        // Exp start state
        // 17 52 -37 -47 28 92 36
        // 28 58 -31 -84 36 132 29
        // 41 62 -49 -72 -140 57 57

        ims::deg2rad(start_state);
        ims::deg2rad(goal_state);
        // normalize the start and goal states
        // get the joint limits
        std::vector<std::pair<double, double>> joint_limits;
        scene_interface.getJointLimits(joint_limits);
        ims::normalizeAngles(start_state, joint_limits);
        ims::normalizeAngles(goal_state, joint_limits);
        ims::roundStateToDiscretization(start_state, action_type.state_discretization_);
        ims::roundStateToDiscretization(goal_state, action_type.state_discretization_);

        a_heuristic->setGoal(goal_state);
        edge_action_space->setGoal(goal_state);

        std::shared_ptr<ims::Planner> planner;

        if (planner_name == "wastar") {
            planner = std::make_shared<ims::wAStar>(*std::dynamic_pointer_cast<ims::wAStarParams>(params));
        } else if (planner_name == "qpase") {
            planner = std::make_shared<ims::Qpase>(*std::dynamic_pointer_cast<ims::ParallelSearchParams>(params));
        } else if (planner_name == "epase") {
            planner = std::make_shared<ims::Epase>(*std::dynamic_pointer_cast<ims::ParallelSearchParams>(params));
        } else if (planner_name == "pase") {
            planner = std::make_shared<ims::Pase>(*std::dynamic_pointer_cast<ims::ParallelSearchParams>(params));
        } else {
            ROS_ERROR_STREAM("Planner " << planner_name << " not recognized");
            return 0;
        }

        if (planner_name == "wastar") {
            try {
                planner->initializePlanner(action_space, start_state, goal_state);
            } catch (std::exception& e) {
                ROS_INFO_STREAM(e.what() << std::endl);
            }
        } else if (planner_name == "qpase") {
            try {
                ROS_INFO_STREAM("Initializing qpase planners.");
                std::dynamic_pointer_cast<ims::Qpase>(planner)->initializePlanner(edge_action_space, start_state, goal_state);
            } catch (std::exception& e) {
                ROS_INFO_STREAM(e.what() << std::endl);
            }
        } else if (planner_name == "epase") {
            try {
                ROS_INFO_STREAM("Initializing epase planners.");
                std::dynamic_pointer_cast<ims::Epase>(planner)->initializePlanner(edge_action_space, start_state, goal_state);
            } catch (std::exception& e) {
                ROS_INFO_STREAM(e.what() << std::endl);
            }
        } else if (planner_name == "pase") {
            try {
                ROS_INFO_STREAM("Initializing pase planners.");
                std::dynamic_pointer_cast<ims::Pase>(planner)->initializePlanner(edge_action_space, start_state, goal_state);
            } catch (std::exception& e) {
                ROS_INFO_STREAM(e.what() << std::endl);
            }
        }

        // Print the goal configuration marker
        auto goal_state_mapped = goal_state;
        scene_interface.calculateFK(goal_state, goal_state_mapped);
        VisualizePoint(scene_interface, goal_state_mapped[0], goal_state_mapped[1], goal_state_mapped[2]);

        bool success = false;
        std::vector<StateType> path_;
        success = planner->plan(path_);
        if (!success) {
            ROS_INFO_STREAM(RED << "No path found" << RESET);
            // return 0;
        } else {
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
                for (int j{0}; j < path_.size(); j++) {
                    for (int i{0}; i < num_joints; i++) {
                        file << path_[j][i] << ",";
                    }
                    file << std::endl;
                }
                file.close();
            }
        }

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
        if (planner_name == "wastar") {
            PlannerStats stats = planner->reportStats();
            ROS_INFO_STREAM("\n"
                            << GREEN
                            << "\t Planner: " << planner_name << std::endl
                            << "\t Planning time: " << stats.time << " sec" << std::endl
                            << "\t cost: " << stats.cost << std::endl
                            << "\t Path length: " << path_.size() << std::endl
                            << "\t Number of nodes expanded: " << stats.num_expanded << std::endl
                            << "\t Suboptimality: " << stats.suboptimality << RESET);
        } else {
            ims::ParallelSearchPlannerStats stats = std::dynamic_pointer_cast<ims::ParallelSearch>(planner)->reportStats();
            ROS_INFO_STREAM("\n"
                            << GREEN
                            << "\t Planner: " << planner_name << std::endl
                            << "\t Planning time: " << stats.time << " sec" << std::endl
                            << "\t Evaluation time: " << stats.evaluation_time << " sec" << std::endl
                            << "\t H time: " << stats.h_time << " sec" << std::endl
                            << "\t Q time: " << stats.q_time << " sec" << std::endl
                            << "\t Lock time: " << stats.lock_time << std::endl
                            << "\t cost: " << stats.cost << std::endl
                            << "\t Path length: " << path_.size() << std::endl
                            << "\t Number of nodes expanded: " << stats.num_expanded << std::endl
                            << "\t Number of edges evaluated: " << stats.num_evaluated << std::endl
                            << "\t Suboptimality: " << stats.suboptimality << RESET);
            logs[i] = stats;
        }
        
        if (!success) {
            continue;
        }

        // profile and execute the path
        // @{
        for (auto& state : path_) {
            traj.push_back(state);
        }
        ims::profileTrajectory(start_state,
                               goal_state,
                               traj,
                               move_group,
                               trajectory);
    }
    if (num_runs) {
        logStats(logs, 1, planner_name);
    }
    ROS_INFO("Executing trajectory");
    // move_group.execute(trajectory);
    // @}
    return 0;
}
