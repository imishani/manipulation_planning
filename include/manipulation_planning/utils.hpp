//
// Created by itamar on 4/3/23.
//

#ifndef MANIPULATION_PLANNING_UTILS_HPP
#define MANIPULATION_PLANNING_UTILS_HPP

#include <vector>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/// \brief A function that dealing with the discontinuity of the joint angles
/// \param state The state to check
/// \return The state with the joint angles in the range of [-pi, pi]
stateType normalizeAngles(const stateType& state){
    stateType normalized_state = state;
    for (int i = 0; i < state.size(); ++i) {
        if (normalized_state[i] > M_PI){
            normalized_state[i] = normalized_state[i] - 2*M_PI;
        }
        else if (normalized_state[i] < -M_PI){
            normalized_state[i] = normalized_state[i] + 2*M_PI;
        }
    }
    return normalized_state;
}

/// \brief A function that takes the discretization vector and a state, and round it to the nearest discretization value
/// \param state The state to check
/// \param discretization The discretization vector
void roundStateToDiscretization(stateType& state, const stateType& discretization){
    for (int i = 0; i < state.size(); ++i) {
        state[i] = round(state[i]/discretization[i])*discretization[i];
    }
}

/// \brief Profile the trajectory
/// \param start The start joint state. type: stateType
/// \param goal The goal joint state. type: stateType
/// \param trajectory a vector of joint states. type: std::vector<stateType>
/// \param move_group_ The move group object. type: moveit::planning_interface::MoveGroupInterface
/// \param trajectory_msg The output trajectory. type: moveit_msgs::RobotTrajectory
/// \return success bool
bool profileTrajectory(const stateType & start,
                       const stateType & goal,
                       const std::vector<stateType> & trajectory,
                       const moveit::planning_interface::MoveGroupInterface & move_group_,
                       moveit_msgs::RobotTrajectory& trajectory_msg){

    trajectory_msg.joint_trajectory.header.frame_id = move_group_.getPlanningFrame();
    trajectory_msg.joint_trajectory.joint_names = move_group_.getActiveJoints();
    trajectory_msg.joint_trajectory.points.resize(trajectory.size());
    for (int i = 0; i < trajectory.size(); ++i) {
        trajectory_msg.joint_trajectory.points[i].positions = trajectory[i];
    }

    // Create a RobotTrajectory object
    robot_trajectory::RobotTrajectory robot_trajectory(move_group_.getRobotModel(), move_group_.getName());
    // convert the trajectory vector to trajectory message
    moveit::core::RobotState start_state_moveit(move_group_.getRobotModel());
    start_state_moveit.setJointGroupPositions(move_group_.getName(), start);
    robot_trajectory.setRobotTrajectoryMsg(start_state_moveit, trajectory_msg);

    // Trajectory processing
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    if (!time_param.computeTimeStamps(robot_trajectory)){
        ROS_ERROR("Failed to compute timestamps for trajectory");
        return false;
    }
    robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

    return true;
}



#endif //MANIPULATION_PLANNING_UTILS_HPP
