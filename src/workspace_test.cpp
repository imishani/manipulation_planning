//
// Created by itamar on 4/3/23.
//

#include <manipulationActionSpace.hpp>
#include <MoveitInterface.hpp>
#include <planners/AStar.hpp>
#include <planners/wAStar.hpp>
#include <common/stdHeuristics.hpp>

#include <ros/ros.h>
// include tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

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

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Test AStar in configuration space
    // @{
    Heuristic heuristic = ims::SE3Heuristic;
    double weight = 10.0;
    ims::wAStarParams params(heuristic, weight);

    MoveitInterface scene_interface ("manipulator_1");
    manipulationType action_type;
    std::vector<double> angles_disc {1, 1, 1};
    deg2rad(angles_disc);
    stateType discretization {0.005, 0.005, 0.005, angles_disc[0], angles_disc[1], angles_disc[2]};
    action_type.Discretization(discretization);
    action_type.setSpaceType(manipulationType::spaceType::WorkSpace);
    std::shared_ptr<ManipulationActionSpace> action_space = std::make_shared<ManipulationActionSpace>(scene_interface, action_type);

    stateType start_state {0, 0, 0, 0, 0, 0};
    // get the current end effector pose
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();  // "arm_1tool0"

    start_state[0] = current_pose.pose.position.x;
    start_state[1] = current_pose.pose.position.y;
    start_state[2] = current_pose.pose.position.z;
    // get the current end effector orientation
    tf2::Quaternion q;
    tf2::fromMsg(current_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    start_state[3] = roll; start_state[4] = pitch; start_state[5] = yaw;
    std::cout << "current state " << start_state[0] << " " << start_state[1] << " " << start_state[2] << " " << start_state[3] << " " << start_state[4] << " " << start_state[5] << std::endl;
    stateType goal_state = start_state;

    // check if the inverse kinematics solution exists for the current pose and check if the solution is equal to the current joint state
    std::vector<double> current_joint_state = move_group.getCurrentJointValues();
    std::vector<double> ik_solution;
    if (!scene_interface.calculateIK(current_pose.pose, current_joint_state, ik_solution)) {
        std::cout << "No IK solution for the current pose" << std::endl;
        return 0;
    }
    else {
//        rad2deg(ik_solution); rad2deg(current_joint_state);
        std::cout << "IK solution for the current pose" << std::endl;
        for (int i = 0; i < ik_solution.size(); i++) {
            std::cout << "joint " << i << " " << ik_solution[i] << " " << current_joint_state[i] << std::endl;
        }

    }

    // change the goal state
    goal_state[0] += 0.02;
//    goal_state[1] -= 40;
//    goal_state[2] += 60;
//    goal_state[3] += 5;
//    goal_state[4] += 20;
//    goal_state[5] += 10;

    roundStateToDiscretization(start_state, action_type.mStateDiscretization);
    roundStateToDiscretization(goal_state, action_type.mStateDiscretization);
    std::cout << "goal state " << goal_state[0] << " " << goal_state[1] << " " << goal_state[2] << " " << goal_state[3] << " " << goal_state[4] << " " << goal_state[5] << std::endl;

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

    // profile and execute the path
    // @{
    std::vector<stateType> traj;
    for (auto& state : path_) {
        traj.push_back(state->getState());
    }
    // execute a waypoint in the workspace
    std::vector<geometry_msgs::Pose> waypoints;
    for (auto& state : traj) {
        geometry_msgs::Pose pose;
        pose.position.x = state[0];
        pose.position.y = state[1];
        pose.position.z = state[2];
        tf2::Quaternion q;
        q.setRPY(state[3], state[4], state[5]);
        pose.orientation = tf2::toMsg(q);
        waypoints.push_back(pose);
    }
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    std::cout << "fraction: " << fraction << std::endl;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group.execute(plan);
    return 0;
}