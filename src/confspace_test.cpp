//
// Created by itamar on 4/3/23.
//

#include <memory>

#include <manipulationActionSpace.hpp>
#include <MoveitInterface.hpp>
#include <planners/AStar.hpp>
#include <planners/wAStar.hpp>
#include <manipHeuristics.hpp>
#include <heuristics/standardHeu.hpp>

#include <ros/ros.h>
#include <utils.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>


int ims::state::id_counter = 0;

void rad2deg(stateType& state) {
    for (auto& val : state) {
        val = val * 180 / M_PI;
    }
}

void deg2rad(stateType& state) {
    for (auto& val : state) {
        val = val * M_PI / 180;
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "configuration_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
    std::string path_mprim = full_path.string() + "/config/manip.mprim";

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // check for collision
    planning_scene::PlanningScenePtr planning_scene;
    planning_scene.reset(new planning_scene::PlanningScene(move_group.getRobotModel()));
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene->checkCollision(collision_request, collision_result, *current_state);

    auto df = getDistanceFieldMoveIt();
//        auto* heuristic = new BFSHeuristic(df, "manipulator_1");
    auto* heuristic = new ims::jointAnglesHeuristic;
    double weight = 10.0;
    ims::wAStarParams params(heuristic, weight);

    MoveitInterface scene_interface("manipulator_1");

    manipulationType action_type (path_mprim);
    stateType discretization {1, 1, 1, 1, 1, 1};
    deg2rad(discretization);
    action_type.Discretization(discretization);

    std::shared_ptr<ManipulationActionSpace> action_space = std::make_shared<ManipulationActionSpace>(scene_interface, action_type);

    stateType start_state {0, 0, 0, 0, 0, 0};
    auto joint_names = move_group.getJointNames();
    for (int i = 0; i < 6; i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
    }
    // make a goal_state a copy of start_state
    rad2deg(start_state);
    stateType goal_state = start_state;

    // change the goal state
    goal_state[0] = -18;
    goal_state[1] = 36;
    goal_state[2] = 52;
    goal_state[3] = 25;
    goal_state[4] = 55;
    goal_state[5] = -108;
//        goal_state[0] = -36; goal_state[1] = -118; goal_state[2] = 152;
//        goal_state[3] = -207; goal_state[4] = -90; goal_state[5] = 200;


    deg2rad(start_state); deg2rad(goal_state);
    // normalize the start and goal states
    action_space->NormalizeAngles(start_state);
    action_space->NormalizeAngles(goal_state);
    roundStateToDiscretization(start_state, action_type.mStateDiscretization);
    roundStateToDiscretization(goal_state, action_type.mStateDiscretization);

    ims::wAStar planner(params);
    try {
        planner.initializePlanner(action_space, start_state, goal_state);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    std::vector<ims::state*> path_;
    if (!planner.plan(path_)) {
        std::cout << "No path found" << std::endl;
        return 0;
    }
    else {
        std::cout << "Path found" << std::endl;
    }

    // Print nicely the path
    for (auto& state : path_) {
        std::cout << "state: " << state->getStateId() << std::endl;
        for (auto& val : state->getState()) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }

    // profile and execute the path
    // @{
    std::vector<stateType> traj;
    for (auto& state : path_) {
        traj.push_back(state->getState());
    }
    moveit_msgs::RobotTrajectory trajectory;
    profileTrajectory(start_state,
                      goal_state,
                      traj,
                      move_group,
                      trajectory);
    move_group.execute(trajectory);

    // rerport stats
    plannerStats stats = planner.reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Path length: " << path_.size() << std::endl;
    std::cout << "Number of nodes expanded: " << stats.numExpanded << std::endl;
    std::cout << "Suboptimality: " << stats.subOptimality << RESET << std::endl;
    // @}
    return 0;
}
