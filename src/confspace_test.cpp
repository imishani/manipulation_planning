//
// Created by itamar on 4/3/23.
//

#include <memory>

#include <manipulation_planning/manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_interface.hpp>
#include <search/planners/astar.hpp>
#include <search/planners/wastar.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <search/heuristics/standard_heuristics.hpp>

#include <ros/ros.h>
#include <manipulation_planning/common/utils.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>



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

    auto df = ims::getDistanceFieldMoveIt();
//    auto* heuristic = new ims::BFSHeuristic(df, "manipulator");
    auto* heuristic = new ims::JointAnglesHeuristic;
    double weight = 10.0;
    ims::wAStarParams params(heuristic, weight);

    ims::MoveitInterface scene_interface("manipulator_1");

    ims::manipulationType action_type (path_mprim);
    StateType discretization {1, 1, 1, 1, 1, 1};
    ims::deg2rad(discretization);
    action_type.Discretization(discretization);

    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);

    StateType start_state {0, 0, 0, 0, 0, 0};
    // auto joint_names = move_group.getVariableNames(); // NOTE(yoraish): Is the method below the same in Noetic?
    const std::vector<std::string>& joint_names = move_group.getJointNames();
    for (int i = 0; i < 6; i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
        ROS_INFO_STREAM("Joint " << joint_names[i] << " is " << start_state[i]);
    }
    // make a goal_state a copy of start_state
    ims::rad2deg(start_state);
    StateType goal_state = start_state;

    // change the goal state
    goal_state[0] = 0;
    goal_state[1] = 0;
    goal_state[2] = 0;
    goal_state[3] = 0;
    goal_state[4] = 0;
    goal_state[5] = 0;
//    goal_state[0] = -36; goal_state[1] = -118; goal_state[2] = 152;
//    goal_state[3] = -207; goal_state[4] = -90; goal_state[5] = 200;


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

    std::vector<ims::State*> path_;
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
    std::vector<StateType> traj;
    for (auto& state : path_) {
        traj.push_back(state->getState());
    }
    moveit_msgs::RobotTrajectory trajectory;
    ims::profileTrajectory(start_state,
                      goal_state,
                      traj,
                      move_group,
                      trajectory);

    std::cout << "Executing trajectory" << std::endl;
    move_group.execute(trajectory);

    // rerport stats
    PlannerStats stats = planner.reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Path length: " << path_.size() << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Suboptimality: " << stats.suboptimality << RESET << std::endl;
    // @}
    return 0;
}
