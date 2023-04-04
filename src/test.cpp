//
// Created by itamar on 4/3/23.
//

#include <manipulationActionSpace.hpp>
#include <MoveitInterface.hpp>
#include <planners/AStar.hpp>
#include <common/stdHeuristics.hpp>

#include <ros/ros.h>

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
     ros::init(argc, argv, "test");
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
        std::string path_mprim = full_path.string() + "/config/manip.mprim";

        // Define Robot inteface to give commands and get info from moveit:
        moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

        // Test AStar in configuration space
        // @{
        Heuristic heuristic = ims::jointAnglesHeuristic;
        ims::AStarParams params(heuristic);

        MoveitInterface scene_interface ("manipulator_1");
        manipulationType action_type (path_mprim);
        stateType discretization {1, 1, 1, 1, 1, 1};
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
        goal_state[0] += 12;

        deg2rad(start_state); deg2rad(goal_state);

        ims::AStar planner(params);
        try {
            planner.initializePlanner(action_space, start_state, goal_state);
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
        }
        std::vector<ims::state*> path_;
        if (!planner.plan(path_)) {
            std::cout << "No path found" << std::endl;
        }
        else {
            std::cout << "Path found" << std::endl;
            return 0;
        }

        plannerStats stats = planner.reportStats();
        std::cout << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;

        // @}
        // Print nicely the path
        for (auto& state : path_) {
            std::cout << "state: " << state->getStateId() << std::endl;
            for (auto& val : state->getState()) {
                std::cout << val << ", ";
            }
            std::cout << std::endl;
        }
        return 0;
}