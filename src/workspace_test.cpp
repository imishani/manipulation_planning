//
// Created by itamar on 4/3/23.
//

#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/astar.hpp>
#include <search/planners/wastar.hpp>
#include <search/heuristics/standard_heuristics.hpp>

#include <ros/ros.h>
// include tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <manipulation_planning/common/utils.hpp>



int main(int argc, char** argv) {

    ros::init(argc, argv, "workspace_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
    std::string path_mprim = full_path.string() + "/config/ws_mprim.yaml";

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Test AStar in configuration space
    // @{
    auto* heuristic = new ims::SE3HeuristicRPY;
    double weight = 20.0;
    ims::wAStarParams params(heuristic, weight);

    ims::MoveitInterface scene_interface ("manipulator_1");
    ims::ManipulationType action_type(path_mprim);

    StateType discretization {0.02, 0.02, 0.02, M_PI/180, M_PI/180, M_PI/180};
    action_type.Discretization(discretization);
    action_type.setSpaceType(ims::ManipulationType::SpaceType::WorkSpace);

    std::shared_ptr<ims::ManipulationActionSpace> action_space = std::make_shared<ims::ManipulationActionSpace>(scene_interface, action_type);


    StateType start_state {0, 0, 0, 0, 0, 0};
    // get the current end effector pose
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();  // "arm_1tool0"

    start_state[0] = current_pose.pose.position.x;
    start_state[1] = current_pose.pose.position.y;
    start_state[2] = current_pose.pose.position.z;

    // If using hopf coordinates:
    Eigen::Quaterniond current_pose_eigen;
    tf::quaternionMsgToEigen(current_pose.pose.orientation, current_pose_eigen);

    ims::get_euler_zyx(current_pose_eigen, start_state[5], start_state[4], start_state[3]);
    ims::normalize_euler_zyx(start_state[5], start_state[4], start_state[3]);

    StateType goal_state = start_state;

    // change the goal state
    goal_state[0] = 0.76;
    goal_state[1] = -0.02;// 0.8;
    goal_state[2] = 0.86;// 1.28;
    goal_state[3] = 0.0; //4*M_PI/180;
    goal_state[4] = 0.0; //10*M_PI/180;
    goal_state[5] = 0.0;

    // discrtize the goal state
    for (int i = 0; i < 6; i++) {
        goal_state[i] = std::round(goal_state[i] / discretization[i]) * discretization[i];
        start_state[i] = std::round(start_state[i] / discretization[i]) * discretization[i];
    }
    Eigen::Quaterniond start_pose_eigen;
    ims::from_euler_zyx(start_state[5], start_state[4], start_state[3], start_pose_eigen);
    geometry_msgs::Pose pose_check;
    pose_check.position.x = start_state[0]; pose_check.position.y = start_state[1]; pose_check.position.z = start_state[2];
    tf::quaternionEigenToMsg(start_pose_eigen, pose_check.orientation);


    std::cout << "rounded pose " << pose_check.position.x << " " << pose_check.position.y << " " << pose_check.position.z << " "
    << pose_check.orientation.x << " " << pose_check.orientation.y << " " << pose_check.orientation.z << " " << pose_check.orientation.w << std::endl;

    std::cout << "original pose " << current_pose.pose.position.x << " " << current_pose.pose.position.y << " " << current_pose.pose.position.z << " "
    << current_pose.pose.orientation.x << " " << current_pose.pose.orientation.y << " " << current_pose.pose.orientation.z << " " << current_pose.pose.orientation.w << std::endl;

    // check if the inverse kinematics solution exists for the current pose and check if the solution is equal to the current joint state
    std::vector<double> current_joint_state = move_group.getCurrentJointValues();
    std::vector<double> ik_solution;

    if (!scene_interface.calculateIK(pose_check, current_joint_state, ik_solution)) {
        std::cout << "No IK solution for the current pose" << std::endl;
        return 0;
    }
    else {
        ims::rad2deg(ik_solution); ims::rad2deg(current_joint_state);
        std::cout << "IK solution for the current pose" << std::endl;
        for (int i = 0; i < ik_solution.size(); i++) {
            std::cout << "joint " << i << " " << ik_solution[i] << " " << current_joint_state[i] << std::endl;
        }
    }
    std::cout << "start state " << start_state[0] << " " << start_state[1] << " " << start_state[2] << " "
    << start_state[3] << " " << start_state[4] << " " << start_state[5] << std::endl;
    std::cout << "goal state " << goal_state[0] << " " << goal_state[1] << " " << goal_state[2] << " "
    << goal_state[3] << " " << goal_state[4] << " " << goal_state[5] << std::endl;

    ims::wAStar planner(params);
    try {
        planner.initializePlanner(action_space, start_state, goal_state);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    std::vector<StateType> path_;
    if (!planner.plan(path_)) {
        std::cout << "No path found" << std::endl;
        return 0;
    }
    else {
        std::cout << "Path found" << std::endl;
    }

    /// TODO: Fix this!

    // @}
//    // Print nicely the path
//    int counter = 0;
//    std::vector<StateType> traj;
//    for (auto& state : path_) {
//        std::cout << "State: " << counter++ << std::endl;
////        if (counter == 1){
////            ims::deg2rad(current_joint_state);
////            state->setMappedState(current_joint_state);
////        }
//        for (auto& val : state) {
//            std::cout << val << ", ";
//        }
//        std::cout << std::endl;
//        std::cout << "Joint state: " << std::endl;
//        for (auto& val : state->getMappedState()) {
//            std::cout << val << ", ";
//        }
//        std::cout << std::endl;
//        traj.push_back(state->getMappedState());
//    }

//    // profile and execute the path
//    // @{
//    std::vector<StateType> traj;
//    for (auto& state : path_) {
//        traj.push_back(state->getState());
//    }
//    // execute a waypoint in the workspace
//    std::vector<geometry_msgs::Pose> waypoints;
//    for (auto& state : traj) {
//        geometry_msgs::Pose pose;
//        pose.position.x = state[0];
//        pose.position.y = state[1];
//        pose.position.z = state[2];
//        Eigen::Quaterniond quat_res;
//        ims::from_euler_zyx(state[5], state[4], state[3], quat_res);
//        pose.orientation.x = quat_res.x(); pose.orientation.y = quat_res.y();
//        pose.orientation.z = quat_res.z(); pose.orientation.w = quat_res.w();
//        waypoints.push_back(pose);
//    }
//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
//    std::cout << "fraction: " << fraction << std::endl;
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    my_plan.trajectory_ = trajectory;
//    move_group_.execute(my_plan);

    // profile and execute the path
    // @{
//    std::vector<StateType> traj;
//    for (auto& state : path_) {
//        // assert if the size of the mapped state is not equal to the size of the joint state
//        std::cout << state->getMappedState().size() << std::endl;
//        assert(state->getMappedState().size() == move_group_.getJoints().size());
//        traj.push_back(state->getMappedState());
//    }
//    moveit_msgs::RobotTrajectory trajectory;
//    ims::profileTrajectory(start_state,
//                           goal_state,
//                           traj,
//                           move_group_,
//                           trajectory);
//    move_group_.execute(trajectory);
//
//    // rerport stats
//    PlannerStats stats = planner.reportStats();
//    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
//    std::cout << "cost: " << stats.cost << std::endl;
//    std::cout << "Path length: " << path_.size() << std::endl;
//    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
//    std::cout << "Suboptimality: " << stats.suboptimality << RESET << std::endl;

    return 0;
}