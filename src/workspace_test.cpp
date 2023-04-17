//
// Created by itamar on 4/3/23.
//

#include <manipulationActionSpace.hpp>
#include <MoveitInterface.hpp>
#include <planners/AStar.hpp>
#include <planners/wAStar.hpp>
#include <heuristics/standardHeu.hpp>

#include <ros/ros.h>
// include tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <utils.hpp>

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

    ros::init(argc, argv, "workspace_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
    std::string path_mprim = full_path.string() + "/config/ws.mprim";

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Test AStar in configuration space
    // @{
    auto* heuristic = new ims::SE3HeuristicHopf;
    double weight = 0.0;
    ims::wAStarParams params(heuristic, weight);

    MoveitInterface scene_interface ("manipulator_1");
    manipulationType action_type(path_mprim);

    stateType discretization {0.02, 0.02, 0.02};
    action_type.Discretization(discretization);
    action_type.setSpaceType(manipulationType::spaceType::WorkSpace);

    std::vector<std::vector<double>> hopf_points = readPointsFromFile(full_path.string() + "/config/hopf_dist_samples_healpix.txt");
    // create kdtree obj
    // first convert to std::vector<Eigen::Vector2d>
    std::vector<Eigen::Vector3d> hopf_points_eigen;
    std::vector<double> psi_vec;
    for (auto& point : hopf_points) {
        Eigen::Vector3d point_eigen;
        point_eigen << point[0], point[1], point[2];
        hopf_points_eigen.push_back(point_eigen);
        psi_vec.push_back(point[2]);
    }

    auto kdtree = createKDTree(hopf_points_eigen);
    action_type.DiscretizationWS(kdtree, psi_vec);

    std::shared_ptr<ManipulationActionSpace> action_space = std::make_shared<ManipulationActionSpace>(scene_interface, action_type);

    // I using quaternion for the orientation:
//    stateType start_state {0, 0, 0, 0, 0, 0, 0};
    // If using hopf coordinates:
    stateType start_state {0, 0, 0, 0, 0, 0};
    // get the current end effector pose
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();  // "arm_1tool0"

    start_state[0] = current_pose.pose.position.x;
    start_state[1] = current_pose.pose.position.y;
    start_state[2] = current_pose.pose.position.z;
//    start_state[3] = current_pose.pose.orientation.x;
//    start_state[4] = current_pose.pose.orientation.y;
//    start_state[5] = current_pose.pose.orientation.z;
//    start_state[6] = current_pose.pose.orientation.w;
    // If using hopf coordinates:
    Eigen::Quaterniond current_pose_eigen;
    tf::quaternionMsgToEigen(current_pose.pose.orientation, current_pose_eigen);
    Eigen::Vector3d start_hopf;
    quaternionToHopf(current_pose_eigen, start_hopf);
    start_state[3] = start_hopf[0]; start_state[4] = start_hopf[1]; start_state[5] = start_hopf[2];

    std::cout << "current state ";
    for (auto& s_ : start_state){
        std::cout << s_ << " ";
    }
    std::cout << std::endl;

    stateType goal_state = start_state;

    // change the goal state
//    goal_state[0] -= 0.2;
//    goal_state[1] -= 0.16;
//    goal_state[2] -= 0.08;


    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond goal_quat;
    goal_quat = current_pose_eigen * quat;
//    goal_state[3] = goal_quat.x();
//    goal_state[4] = goal_quat.y();
//    goal_state[5] = goal_quat.z();
//    goal_state[6] = goal_quat.w();
    // If using hopf coordinates:
    Eigen::Vector3d goal_hopf;
    quaternionToHopf(goal_quat, goal_hopf);
    goal_state[3] = goal_hopf[0]; goal_state[4] = goal_hopf[1]; goal_state[5] = goal_hopf[2];

    std::vector<double> start_state_vec(start_state.begin(), start_state.begin() + 3);
    std::vector<double> goal_state_vec(goal_state.begin(), goal_state.begin() + 3);
    roundStateToDiscretization(start_state_vec, discretization);
    roundStateToDiscretization(goal_state_vec, discretization);
    // copy the sliced vector to the start and goal state
    std::copy(start_state_vec.begin(), start_state_vec.end(), start_state.begin());
    std::copy(goal_state_vec.begin(), goal_state_vec.end(), goal_state.begin());
    // lets do the quaternions:
//    std::vector<double> start_state_quat(start_state.begin() + 3, start_state.begin() + 6);
//    std::vector<double> goal_state_quat(goal_state.begin() + 3, goal_state.begin() + 6);
//    getClosestQuaternion(start_state_quat, action_type.mKDTree, action_type.mPsiVector);
//    std::copy(start_state_quat.begin(), start_state_quat.end(), start_state.begin() + 3);
//    getClosestQuaternion(goal_state_quat, action_type.mKDTree, action_type.mPsiVector);
//    std::copy(goal_state_quat.begin(), goal_state_quat.end(), goal_state.begin() + 3);
    std::vector<int> check;
    query_disc(std::vector<double>(start_hopf[0], start_hopf[1]), M_PI/ 16.0, check);
    std::cout << action_type.mKDTree->getData()[check[0]] << std::endl;
    std::cout << action_type.mKDTree->getData()[check[1]] << std::endl;
    std::cout << action_type.mKDTree->getData()[check[2]] << std::endl;
    std::cout << action_type.mKDTree->getData()[check[3]] << std::endl;


    // If using hopf coordinates:
    getClosestHopf(start_hopf, action_type.mKDTree, action_type.mPsiVector);
    getClosestHopf(goal_hopf, action_type.mKDTree, action_type.mPsiVector);
    start_state[3] = start_hopf[0]; start_state[4] = start_hopf[1]; start_state[5] = start_hopf[2];
    goal_state[3] = goal_hopf[0]; goal_state[4] = goal_hopf[1]; goal_state[5] = goal_hopf[2];

// check if the inverse kinematics solution exists for the current pose and check if the solution is equal to the current joint state
    std::vector<double> current_joint_state = move_group.getCurrentJointValues();
    std::vector<double> ik_solution;

    if (!scene_interface.calculateIK(current_pose.pose, current_joint_state, ik_solution)) {
        std::cout << "No IK solution for the current pose" << std::endl;
        return 0;
    }
    else {
        rad2deg(ik_solution); rad2deg(current_joint_state);
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
    std::vector<ims::state*> path_;
    if (!planner.plan(path_)) {
        std::cout << "No path found" << std::endl;
        return 0;
    }
    else {
        std::cout << "Path found" << std::endl;
    }

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
        // If using quaternion coordinates:
//        pose.orientation.x = state[3];
//        pose.orientation.y = state[4];
//        pose.orientation.z = state[5];
//        pose.orientation.w = state[6];
        // If using hopf coordinates:
        Eigen::Vector3d hopf(state[3], state[4], state[5]);
        Eigen::Quaterniond quat_res;
        hopfToQuaternion(hopf, quat_res);
        pose.orientation.x = quat_res.x(); pose.orientation.y = quat_res.y();
        pose.orientation.z = quat_res.z(); pose.orientation.w = quat_res.w();
        waypoints.push_back(pose);
    }
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    std::cout << "fraction: " << fraction << std::endl;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);

    // rerport stats
    plannerStats stats = planner.reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Path length: " << path_.size() << std::endl;
    std::cout << "Number of nodes expanded: " << stats.numExpanded << std::endl;
    std::cout << "Suboptimality: " << stats.subOptimality << RESET << std::endl;

    return 0;
}