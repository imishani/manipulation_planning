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
 * \file   utils.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/3/23
 */

#ifndef MANIPULATION_PLANNING_UTILS_HPP
#define MANIPULATION_PLANNING_UTILS_HPP

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/distance_field/distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/distance_field/voxel_grid.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>

#include <search/common/collisions.hpp>
#include <search/common/types.hpp>
#include <vector>

namespace ims {
/// \brief A function that converts a state from radians to degrees
/// \param state The state
template<typename T>
void rad2deg(std::vector<T> &state, const std::vector<bool> &valid_mask = std::vector<bool>()) {
    bool is_valid_mask_passed = !valid_mask.empty();

    for (size_t i = 0; i < state.size(); ++i) {
        // If the valid mask is passed, then check if the dimension is valid.
        if (is_valid_mask_passed) {
            if (!valid_mask[i]) {
                continue;
            }
        }

        // Convert the dimension, if it is valid according to the mask.
        state[i] = state[i] * 180 / M_PI;
    }
}

/// \brief A function that converts a state from degrees to radians
/// \param state The state
/// \param valid_mask The valid mask of the state. Dimensions that are true, get converted. Others are not.
template<typename T>
void deg2rad(std::vector<T> &state, const std::vector<bool> &valid_mask = std::vector<bool>()) {
    // If the mask is not passed, then all the dimensions are assumed to be valid.
    bool is_valid_mask_passed = !valid_mask.empty();

    for (size_t i = 0; i < state.size(); ++i) {
        // If the valid mask is passed, then check if the dimension is valid.
        if (is_valid_mask_passed) {
            if (!valid_mask[i]) {
                continue;
            }
        }

        // Convert the dimension, if it is valid according to the mask.
        state[i] = state[i] * M_PI / 180;
    }
}

template<typename T>
T radianDifference(T a, T b) {
    T diff = a - b;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

inline void negateState(StateType &state, const std::vector<bool> &valid_mask = std::vector<bool>()) {
    // If the mask is not passed, then all the dimensions are assumed to be valid.
    bool is_valid_mask_passed = !valid_mask.empty();

    for (size_t i = 0; i < state.size(); ++i) {
        // If the valid mask is passed, then check if the dimension is valid.
        if (is_valid_mask_passed) {
            if (!valid_mask[i]) {
                continue;
            }
        }

        // Convert the dimension, if it is valid according to the mask.
        state[i] = -state[i];
    }
}

/// \brief A function that dealing with the discontinuity of the joint angles
/// \param state The state to check
/// \param joint_limits The joint limits
/// \param valid_mask The valid mask of the state. Dimensions that are true, get normalized. Others are not.
/// \return The state with the joint angles in the range of the joint limits
template<typename T>
inline void normalizeAngles(std::vector<T> &state,
                            std::vector<std::pair<T, T> > &joint_limits,
                            const std::vector<bool> &valid_mask = std::vector<bool>()) {
    // If the mask is not passed, then all the dimensions are assumed to be valid.
    bool is_valid_mask_passed = !valid_mask.empty();

    // Iterate through the state and normalize the dimensions that are valid.
    for (int i = 0; i < state.size(); ++i) {
        // If the valid mask is passed, then check if the dimension is valid.
        if (is_valid_mask_passed) {
            if (!valid_mask[i]) {
                continue;
            }
        }

        // Normalize the dimension to the range [joint_limits[i].first, joint_limits[i].second] and request angles to be in the range of [-pi, pi], if it is not in the range of the joint limits and it is valid according to the mask.
        while (state[i] > joint_limits[i].second) {
            state[i] = state[i] - 2 * M_PI;
        }
        while (state[i] < joint_limits[i].first) {
            state[i] = state[i] + 2 * M_PI;
        }
    }
}

/// \brief A function that dealing with the discontinuity of the joint angles
/// \note This function assumes that the joint limits are [-pi, pi]
/// \param state The state to check
/// \param valid_mask The valid mask of the state. Dimensions that are true, get normalized. Others are not.
/// \return The state with the joint angles in the range of [-pi, pi]
template<typename T>
inline void normalizeAngles(std::vector<T> &state, const std::vector<bool> &valid_mask = std::vector<bool>()) {
    bool is_valid_mask_passed = !valid_mask.empty();
    // Iterate through the state and normalize the dimensions that are valid.
    for (int i = 0; i < state.size(); ++i) {
        // If the valid mask is passed, then check if the dimension is valid.
        if (is_valid_mask_passed) {
            if (!valid_mask[i]) {
                continue;
            }
        }

        // Normalize the dimension, if it is not in the range of the joint limits and it is valid according to the mask.
        while (state[i] > M_PI) {
            state[i] = state[i] - 2 * M_PI;
        }
        while (state[i] < -M_PI) {
            state[i] = state[i] + 2 * M_PI;
        }
    }
}

template<typename T>
inline void checkFixGimbalLock(T y, T p, T r) {
    // check if current state in gimbal lock
    /// TODO: Implement
}

template<typename T>
inline void get_euler_zyx(const Eigen::Matrix<T, 3, 3> &rot, T &y, T &p, T &r) {
    y = std::atan2(rot(1, 0), rot(0, 0));
    p = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
    r = std::atan2(rot(2, 1), rot(2, 2));
}

template<typename T>
inline void get_euler_zyx(const Eigen::Quaternion<T> &rot, T &y, T &p, T &r) {
    Eigen::Matrix<T, 3, 3> R(rot);
    get_euler_zyx(R, y, p, r);
}

template<typename T>
inline void from_euler_zyx(T y, T p, T r, Eigen::Matrix<T, 3, 3> &rot) {
    rot = Eigen::AngleAxis<T>(y, Eigen::Matrix<T, 3, 1>::UnitZ()) *
        Eigen::AngleAxis<T>(p, Eigen::Matrix<T, 3, 1>::UnitY()) *
        Eigen::AngleAxis<T>(r, Eigen::Matrix<T, 3, 1>::UnitX());
}

template<typename T>
inline void from_euler_zyx(T y, T p, T r, Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T, 3, 3> R;
    from_euler_zyx(y, p, r, R);
    q = Eigen::Quaternion<T>(R);
}

template<typename T>
inline void from_euler_zyx(T y, T p, T r, geometry_msgs::Pose &q) {
    Eigen::Matrix<T, 3, 3> R;
    from_euler_zyx(y, p, r, R);
    Eigen::Quaternion<T> quat(R);
    tf::quaternionEigenToMsg(quat, q.orientation);
}

template<typename T>
inline void normalize_euler_zyx(T &y, T &p, T &r) {
    Eigen::Matrix<T, 3, 3> rot;
    from_euler_zyx(y, p, r, rot);
    get_euler_zyx(rot, y, p, r);
}

template<typename T>
inline void normalize_euler_zyx(T *angles) // in order r, p, y
{
    Eigen::Matrix<T, 3, 3> rot;
    from_euler_zyx(angles[2], angles[1], angles[0], rot);
    get_euler_zyx(rot, angles[2], angles[1], angles[0]);
}

/// \brief Convert hopf coordinates to quaternion
/// \param hopf The hopf coordinates vector
/// \param q The quaternion (by reference)
inline void hopfToQuaternion(const Eigen::Vector3d &hopf, Eigen::Quaterniond &q) {
    double theta = hopf[0];
    double phi = hopf[1];
    double psi = hopf[2];
    q.w() = cos(theta / 2.0) * cos(psi / 2.0);
    q.x() = cos(theta / 2.0) * sin(psi / 2.0);
    q.y() = sin(theta / 2.0) * cos(phi + psi / 2.0);
    q.z() = sin(theta / 2.0) * sin(phi + psi / 2.0);
}

/// \brief Convert quaternion to hopf coordinates
/// \param q The quaternion
/// \param hopf The hopf coordinates vector (by reference)
/// The range of the hopf coordinates is the following:
/// theta: [0, pi]
/// phi: [0, 2pi]
/// psi: [0, 2pi]
inline void quaternionToHopf(const Eigen::Quaterniond &q, Eigen::Vector3d &hopf) {
    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();
    // Make sure that w >= 0 and if not, negate the quaternion
    if (w < 0.0) {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
    hopf[2] = 2.0 * atan2(x, w);
    hopf[1] = atan2(z, y) - hopf[2] / 2.0;
    if (hopf[1] < 0.0) {
        hopf[1] += 2.0 * M_PI;
    }
    hopf[0] = 2.0 * atan2(sqrt(y * y + z * z), sqrt(x * x + w * w));
}

/// \brief A function that takes the discretization vector and a state, and round it to the nearest discretization value
/// \param state The state to check
/// \param discretization The discretization vector
inline void roundStateToDiscretization(StateType &state, const StateType &discretization) {
    for (int i = 0; i < state.size(); ++i) {
        state[i] = round(state[i] / discretization[i]) * discretization[i];
    }
}

/// \brief Read a txt file and return a vector of points
/// \param file_path The path to the file
/// \return The vector of points
inline std::vector<std::vector<double> > readPointsFromFile(const std::string &file_path) {
    std::vector<std::vector<double> > points;
    std::ifstream file(file_path);
    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> point;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            point.push_back(std::stod(value));
        }
        points.push_back(point);
    }
    return points;
}

/// \brief Search for the nearest point in a 1d vector in the most efficient way assuming the vector is sorted
/// \param point The point to search for (int/double)
/// \param points The vector of points (vector of int/double)
/// \return The index of the nearest point
template<typename T>
inline int findNearestPoint(const T &point, const std::vector<T> &points) {
    int index = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < points.size(); ++i) {
        double distance = std::abs(point - points[i]);
        if (distance < min_distance) {
            min_distance = distance;
            index = i;
        }
    }
    return index;
}

/// \brief geodesic distance
/// \param q1 The first hopf coordinates
/// \param q2 The second hopf coordinates
/// \return The geodesic distance
inline double geodesicDistance(const Eigen::Vector3d &q1, const Eigen::Vector3d &q2) {
    double theta1 = q1[0];
    double phi1 = q1[1];
    double psi1 = q1[2];
    double theta2 = q2[0];
    double phi2 = q2[1];
    double psi2 = q2[2];
    double distance = acos(cos(theta1) * cos(theta2) + sin(theta1) * sin(theta2) * cos(phi1 - phi2 + psi1 - psi2));
    return distance;
}

/// \brief Profile the trajectory
/// \param start The start joint state. type: StateType
/// \param goal The goal joint state. type: StateType
/// \param trajectory a vector of joint states. type: std::vector<StateType>
/// \param move_group_ The move group object. type: moveit::planning_interface::MoveGroupInterface
/// \param trajectory_msg The output trajectory. type: moveit_msgs::RobotTrajectory
/// \return success bool
inline bool profileTrajectory(const StateType &start,
                              const StateType &goal,
                              const std::vector<StateType> &trajectory,
                              const moveit::planning_interface::MoveGroupInterface &move_group_,
                              moveit_msgs::RobotTrajectory &trajectory_msg,
                              double velocity_scaling_factor = 0.1,
                              double acceleration_scaling_factor = 0.1) {
    trajectory_msg.joint_trajectory.header.frame_id = move_group_.getPlanningFrame();
    trajectory_msg.joint_trajectory.joint_names = move_group_.getActiveJoints();
    trajectory_msg.joint_trajectory.points.resize(trajectory.size());

    // check if current robot state is the same as the start state
    auto current_state = move_group_.getCurrentState();
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(move_group_.getName(), joint_values);
    for (int i = 0; i < trajectory.size(); ++i) {
        trajectory_msg.joint_trajectory.points[i].positions = trajectory[i];
    }

    // Create a RobotTrajectory object
    robot_trajectory::RobotTrajectory robot_trajectory(move_group_.getRobotModel(), move_group_.getName());
    // convert the trajectory vector to trajectory message
    moveit::core::RobotState start_state_moveit(robot_trajectory.getRobotModel());
    // set start_state_moveit to the start state of the trajectory
    start_state_moveit.setJointGroupPositions(move_group_.getName(), trajectory[0]);
    robot_trajectory.setRobotTrajectoryMsg(start_state_moveit, trajectory_msg);

    // Trajectory processing
    trajectory_processing::IterativeParabolicTimeParameterization time_param(true);
    // scale the velocity of the trajectory
    // get the velocity scaling factor
    if (!time_param.computeTimeStamps(robot_trajectory,
                                      velocity_scaling_factor,
                                      acceleration_scaling_factor)) {
        ROS_ERROR("Failed to compute timestamps for trajectory");
        return false;
    }
    robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

    return true;
}

/// @brief Try to shortcut and smooth the path
/// @param path The path to shortcut and smooth
/// @param move_group The move group object
/// @return bool success
/// @TODO: Needs to be edited. Currently it is way too heavy to run.
inline bool ShortcutSmooth(PathType &path,
                           moveit::planning_interface::MoveGroupInterface &move_group,
                           const planning_scene::PlanningScenePtr &planning_scene,
                           double timeout = 1.0) {
    // Iteratively moving further and further down the path, attempting to replace the original path with
    // shortcut paths. The idea is that each action only moves one angle, and we want to move few angles together if possie
    // to reduce the number of actions
    double start_time = ros::Time::now().toSec();
    if (path.empty())
        return false;
    else if (path.size() == 1)
        return true;
    PathType shortcut_path;
    auto current_state = path.front();
    int current_state_index = 0;
    shortcut_path.push_back(current_state);
    int last_state_index = static_cast<int>(path.size() - 1);
    // iteratively try to connect the current state to the closest state to the last state
    collision_detection::CollisionRequest collision_request;
    collision_request.verbose = true;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState robot_state = planning_scene->getCurrentStateNonConst();
    // print the robot state
    bool is_timeout = false;
    double step_size = 0.05;
    // start from the start state (current_state) and try to connect it to the next states along the path.
    // while collision free, try to connect to the next state. if not, then use the last collision-free shortcut,
    // and start the same process of connecting from the last collision free state
    while (current_state_index < last_state_index - 1) {
        for (int i{current_state_index + 2}; i <= last_state_index; ++i) {
            // check if the timeout is reached
            if (ros::Time::now().toSec() - start_time > timeout) {
                is_timeout = true;
                break;
            }
            // interpolate between the current state and the next state
            double distance = 0.0;
            std::vector<double> to_state = path[i];
            for (int j = 0; j < current_state.size(); ++j) {
                distance += std::pow(current_state[j] - to_state[j], 2);
            }
            distance = std::sqrt(distance);
            int num_steps = static_cast<int>(distance / step_size);
            // interpolate between the current state and the next state
            for (int j = 0; j < num_steps; ++j) {
                std::vector<double> state;
                for (int k = 0; k < current_state.size(); ++k) {
                    state.push_back(current_state[k] + (to_state[k] - current_state[k]) * (j + 1) / num_steps);
                }
                // check if the state is in collision
                robot_state.setJointGroupPositions(move_group.getName(), state);
                collision_result.clear();
                planning_scene->checkCollision(collision_request, collision_result, robot_state);
                if (collision_result.collision) {
                    // add the last collision-free state to the shortcut path
                    shortcut_path.push_back(path[i - 1]);
                    // update the current state
                    current_state = path[i - 1];
                    current_state_index = i - 1;
                    break;
                } else if ((ros::Time::now().toSec() - start_time) > timeout) {
                    is_timeout = true;
                    break;
                }
            }
            // check if the timeout is reached
            if (collision_result.collision) {
                break;
            } else if ((ros::Time::now().toSec() - start_time) > timeout) {
                is_timeout = true;
                break;
            }
        }
        // check if the timeout is reached
        if (is_timeout) {
            break;
        }
    }
    // check if the timeout is reached
    if (is_timeout) {
        ROS_INFO_STREAM("Timeout reached! "
            << "Shortcut path size: " << shortcut_path.size());
        // add the rest of the path from the current state to the last state
        PathType rest_of_path;
        // loop in reverse order
        for (size_t i = path.size() - 1; i > 0; --i) {
            if (i == current_state_index) {
                break;
            }
            rest_of_path.push_back(path[i]);
        }
        // reverse the rest of the path
        std::reverse(rest_of_path.begin(), rest_of_path.end());
        // add the rest of the path to the shortcut path
        for (auto &state : rest_of_path) {
            shortcut_path.push_back(state);
        }
    }
    shortcut_path.push_back(path.back());
    ROS_DEBUG_STREAM_NAMED("SHORTCUT: ", "Shortcut path size: " << shortcut_path.size());
    ROS_DEBUG_STREAM_NAMED("SHORTCUT: ", "Path size: " << path.size());
    // interpolate between the states in the shortcut path to make sure the maximum distance between two states is 0.1
    PathType shortcut_path_interpolated;
    shortcut_path_interpolated.push_back(shortcut_path[0]);
    for (int i = 0; i < shortcut_path.size() - 1; ++i) {
        double distance = 0.0;
        std::vector<double> from_state = shortcut_path[i];
        std::vector<double> to_state = shortcut_path[i + 1];
        for (int j = 0; j < from_state.size(); ++j) {
            distance += std::pow(from_state[j] - to_state[j], 2);
        }
        distance = std::sqrt(distance);
        int num_steps = static_cast<int>(distance / 0.1);
        // interpolate between the current state and the next state
        for (int j = 0; j < num_steps; ++j) {
            std::vector<double> state;
            for (int k = 0; k < from_state.size(); ++k) {
                state.push_back(from_state[k] + (to_state[k] - from_state[k]) * (j + 1) / num_steps);
            }
            shortcut_path_interpolated.push_back(state);
        }
    }
    path = shortcut_path;
    return true;
}

/// \brief Get an empty distance field
/// \param df_size_x The size of the distance field in x
/// \param df_size_y The size of the distance field in y
/// \param df_size_z The size of the distance field in z
/// \param df_res The resolution of the distance field
/// \param df_origin_x The origin of the distance field in x
/// \param df_origin_y The origin of the distance field in y
/// \param df_origin_z The origin of the distance field in z
/// \param max_distance The maximum distance of the distance field
/// \return The distance field
inline std::shared_ptr<distance_field::PropagationDistanceField> getEmptyDistanceField(double df_size_x = 3.0,
    double df_size_y = 3.0,
    double df_size_z = 3.0,
    double df_res = 0.02,
    double df_origin_x = -0.75,
    double df_origin_y = -1.5,
    double df_origin_z = 0.0,
    double max_distance = 1.8) {
    std::shared_ptr<distance_field::PropagationDistanceField> df(new distance_field::PropagationDistanceField(
        df_size_x,
        df_size_y,
        df_size_z,
        df_res,
        df_origin_x,
        df_origin_y,
        df_origin_z,
        max_distance));
    return df;
}

/// \brief Get the distance field
/// \param df_size_x The size of the distance field in x
/// \param df_size_y The size of the distance field in y
/// \param df_size_z The size of the distance field in z
/// \param df_res The resolution of the distance field
/// \param df_origin_x The origin of the distance field in x
/// \param df_origin_y The origin of the distance field in y
/// \param df_origin_z The origin of the distance field in z
/// \param max_distance The maximum distance of the distance field
/// \return The distance field
inline std::shared_ptr<distance_field::PropagationDistanceField> getDistanceFieldMoveIt(double df_size_x = 3.0,
    double df_size_y = 3.0,
    double df_size_z = 3.0,
    double df_res = 0.02,
    double df_origin_x = -0.75,
    double df_origin_y = -1.5,
    double df_origin_z = 0.0,
    double max_distance = 1.8) {
    auto df = std::make_shared<distance_field::PropagationDistanceField>(df_size_x,
                                                                         df_size_y,
                                                                         df_size_z,
                                                                         df_res,
                                                                         df_origin_x,
                                                                         df_origin_y,
                                                                         df_origin_z,
                                                                         max_distance);
    // get the planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // get the collision objects
    auto objects = planning_scene_interface.getObjects();
    std::vector<moveit_msgs::CollisionObject> objs;
    for (auto &obj : objects) {
        objs.push_back(obj.second);
    }
    // fill the distance field
    for (auto &obj : objs) {
        Eigen::Isometry3d pose;
        // convert from pose to eigen
        geometry_msgs::Pose pose_msg = obj.pose;
        tf::poseMsgToEigen(pose_msg, pose);
        shapes::Shape *shape;
        // get shape from a collision object
        if (obj.primitives.empty() && obj.meshes.empty() && obj.planes.empty()) {
            // raise exception
            std::cout << "No shape in collision object" << std::endl;
            std::exception e;
        }
        if (!obj.primitives.empty()) {
            // cnsrtuct shape from primitive;
            for (auto &primitive : obj.primitives) {
                shape = shapes::constructShapeFromMsg(primitive);
                df->addShapeToField(shape, pose);
            }
        }
        if (!obj.planes.empty()) {
            // construct shape from plane
            for (auto &plane : obj.planes) {
                shape = shapes::constructShapeFromMsg(plane);
                df->addShapeToField(shape, pose);
            }
        }
        if (!obj.meshes.empty()) {
            // construct shape from mesh
            for (auto &mesh : obj.meshes) {
                EigenSTL::vector_Vector3d points;
                for (auto &vertices : mesh.vertices) {
                    // transform the vertices to the world frame (using pose)
                    // rotate the vertices and then translate them

                    auto rot = pose.rotation();
                    Eigen::Vector3d p1(rot(0, 0) * vertices.x + rot(0, 1) * vertices.y + rot(0, 2) * vertices.z,
                                       rot(1, 0) * vertices.x + rot(1, 1) * vertices.y + rot(1, 2) * vertices.z,
                                       rot(2, 0) * vertices.x + rot(2, 1) * vertices.y + rot(2, 2) * vertices.z);
                    p1.x() += pose.translation().x();
                    p1.y() += pose.translation().y();
                    p1.z() += pose.translation().z();
                    points.push_back(p1);
                }
                df->addPointsToField(points);
                // for (auto &triangels : mesh.triangles) {
                //     // get the points of the mesh
                //     EigenSTL::vector_Vector3d points;
                //     for (int i = 0; i < triangels.vertex_indices.size(); i += 3) {
                //         // generate a bounding box around the mesh
                //         // Eigen::Vector3d p1(mesh.vertices[triangels.vertex_indices[i]].x,
                //         //                    mesh.vertices[triangels.vertex_indices[i]].y,
                //         //                    mesh.vertices[triangels.vertex_indices[i]].z);
                //
                //         Eigen::Vector3d p1(rot(0, 0) * mesh.vertices[triangels.vertex_indices[i]].x +
                //                            rot(0, 1) * mesh.vertices[triangels.vertex_indices[i]].y +
                //                            rot(0, 2) * mesh.vertices[triangels.vertex_indices[i]].z,
                //                            rot(1, 0) * mesh.vertices[triangels.vertex_indices[i]].x +
                //                            rot(1, 1) * mesh.vertices[triangels.vertex_indices[i]].y +
                //                            rot(1, 2) * mesh.vertices[triangels.vertex_indices[i]].z,
                //                            rot(2, 0) * mesh.vertices[triangels.vertex_indices[i]].x +
                //                            rot(2, 1) * mesh.vertices[triangels.vertex_indices[i]].y +
                //                            rot(2, 2) * mesh.vertices[triangels.vertex_indices[i]].z);
                //         p1.x() += pose.translation().x();
                //         p1.y() += pose.translation().y();
                //         p1.z() += pose.translation().z();
                //         geometry_msgs::Point p1_msg;
                //         p1_msg.x = p1.x(); p1_msg.y = p1.y(); p1_msg.z = p1.z();
                //
                //         Eigen::Vector3d p2(rot(0, 0) * mesh.vertices[triangels.vertex_indices[i+1]].x +
                //                            rot(0, 1) * mesh.vertices[triangels.vertex_indices[i+1]].y +
                //                            rot(0, 2) * mesh.vertices[triangels.vertex_indices[i+1]].z,
                //                            rot(1, 0) * mesh.vertices[triangels.vertex_indices[i+1]].x +
                //                            rot(1, 1) * mesh.vertices[triangels.vertex_indices[i+1]].y +
                //                            rot(1, 2) * mesh.vertices[triangels.vertex_indices[i+1]].z,
                //                            rot(2, 0) * mesh.vertices[triangels.vertex_indices[i+1]].x +
                //                            rot(2, 1) * mesh.vertices[triangels.vertex_indices[i+1]].y +
                //                            rot(2, 2) * mesh.vertices[triangels.vertex_indices[i+1]].z);
                //         p2.x() += pose.translation().x();
                //         p2.y() += pose.translation().y();
                //         p2.z() += pose.translation().z();
                //         geometry_msgs::Point p2_msg;
                //         p2_msg.x = p2.x(); p2_msg.y = p2.y(); p2_msg.z = p2.z();
                //
                //         Eigen::Vector3d p3(rot(0, 0) * mesh.vertices[triangels.vertex_indices[i+2]].x +
                //                            rot(0, 1) * mesh.vertices[triangels.vertex_indices[i+2]].y +
                //                            rot(0, 2) * mesh.vertices[triangels.vertex_indices[i+2]].z,
                //                            rot(1, 0) * mesh.vertices[triangels.vertex_indices[i+2]].x +
                //                            rot(1, 1) * mesh.vertices[triangels.vertex_indices[i+2]].y +
                //                            rot(1, 2) * mesh.vertices[triangels.vertex_indices[i+2]].z,
                //                            rot(2, 0) * mesh.vertices[triangels.vertex_indices[i+2]].x +
                //                            rot(2, 1) * mesh.vertices[triangels.vertex_indices[i+2]].y +
                //                            rot(2, 2) * mesh.vertices[triangels.vertex_indices[i+2]].z);
                //         p3.x() += pose.translation().x();
                //         p3.y() += pose.translation().y();
                //         p3.z() += pose.translation().z();
                //         geometry_msgs::Point p3_msg;
                //         p3_msg.x = p3.x(); p3_msg.y = p3.y(); p3_msg.z = p3.z();
                //
                //         shape_msgs::Mesh triangle;
                //         triangle.vertices.push_back(p1_msg);
                //         triangle.vertices.push_back(p2_msg);
                //         triangle.vertices.push_back(p3_msg);
                //
                //         boost::array<uint32_t, 3> indices{};
                //         indices[0] = triangle.vertices.size() - 3;
                //         indices[1] = triangle.vertices.size() - 2;
                //         indices[2] = triangle.vertices.size() - 1;
                //
                //         triangle.triangles.resize(triangle.triangles.size() + 1);
                //         triangle.triangles.back().vertex_indices = indices;
                //         // triangle.triangles
                //         shape = shapes::constructShapeFromMsg(triangle);
                //         df->addShapeToField(shape, pose);
                //     }
                // }
            }

            // for (auto& submesh : mesh.triangles) {
            //     // get the points of the mesh
            //     EigenSTL::vector_Vector3d points;
            //     for (int i = 0; i < submesh.vertex_indices.size(); i += 1) {
            //         Eigen::Vector3d p1(mesh.vertices[submesh.vertex_indices[i]].x,
            //                            mesh.vertices[submesh.vertex_indices[i]].y,
            //                            mesh.vertices[submesh.vertex_indices[i]].z);
            //         points.push_back(p1);
            //     }
            //     df->addPointsToField(points);
            // }
            // }
        }
    }
    return df;
}

/// \brief Check if a point is in collision
/// \param df The distance field
/// \param point The point to check
/// \return True if the point is in collision
inline bool isPointInCollision(const distance_field::PropagationDistanceField &df, const Eigen::Vector3d &point) {
    int x, y, z;
    df.worldToGrid(point[0], point[1], point[2], x, y, z);
    return df.getCell(x, y, z).distance_square_ == 0;
}

/// \brief Check if cell is occupied
/// \param df The distance field
/// \param cell The cell to check
/// \return True if the cell is occupied
inline bool isCellOccupied(const distance_field::PropagationDistanceField &df, const Eigen::Vector3i &cell) {
    return df.getCell(cell[1], cell[1], cell[2]).distance_square_ == 0;
}

/// \brief Count occupied cells in the distance field
/// \param df The distance field
/// \return The number of occupied cells
inline unsigned int countOccupiedCells(const distance_field::PropagationDistanceField &df) {
    unsigned int count = 0;
    for (int z = 0; z < df.getZNumCells(); z++) {
        for (int x = 0; x < df.getXNumCells(); x++) {
            for (int y = 0; y < df.getYNumCells(); y++) {
                if (df.getCell(x, y, z).distance_square_ == 0) {
                    count++;
                }
            }
        }
    }
    return count;
}

/// \brief Get the shape occupancy in the distance field
/// \param df The distance field
/// \param shape The shape to check
/// \param occupied_cells The vector of occupied cells
/// \return A vector of occupied cells
inline void getShapeOccupancy(const std::shared_ptr<distance_field::PropagationDistanceField> &df,
                              const shapes::Shape &shape,
                              const Eigen::Transform<double, 3, 1> &pose,
                              std::vector<std::vector<int> > &occupied_cells) {
    /// TODO: Its not working well.

    double radius;
    Eigen::Vector3d center;
    computeShapeBoundingSphere(&shape, center, radius);
    // get the bounding cylinder of the shape
    visualization_msgs::Marker marker;
    constructMarkerFromShape(&shape, marker);
    // loop through the points and add them to the distance field as occupied
    for (auto &i : marker.points) {
        // transform point to world frame
        Eigen::Vector3d point(i.x, i.y, i.z);
        point = pose * point;
        // get the grid coordinates
        int x, y, z;
        df->worldToGrid(point.x(), point.y(), point.z(), x, y, z);
        // If the cell is already occupied, skip it
        if (df->getCell(x, y, z).distance_square_ == 0) {
            continue;
        }
        // add the cell to the occupied cells vector
        std::vector<int> cell;
        cell.push_back(x);
        cell.push_back(y);
        cell.push_back(z);
        occupied_cells.push_back(cell);
    }
}

/// \brief Get the occupied cells by the arm's robot_state in the distance field
/// \param df The distance field
/// \param robot_state The robot state
/// \param move_group_ The move group object
/// \param occupied_cells The vector of occupied cells
/// \return A vector of occupied cells
inline void getRobotOccupancy(
    const std::shared_ptr<distance_field::PropagationDistanceField> &df,
    moveit::core::RobotState &robot_state,
    const std::unique_ptr<moveit::planning_interface::MoveGroupInterface> &move_group_,
    std::vector<std::vector<int> > &occupied_cells) {
    // get the collision models of the robot
    std::vector<const robot_model::LinkModel *> link_models =
        robot_state.getJointModelGroup(move_group_->getName())->getLinkModels();
    // delete all link models besides the first two:
    link_models.erase(link_models.begin(), link_models.begin() + 2);

    // get all the occupied cells
    for (auto &link : link_models) {
        if (link->getName() == "world") {
            continue;
        }
        if (link->getShapes().empty())
            continue;
        // for each shape in the link model get the occupied cells
        for (auto &shape : link->getShapes()) {
            // get the link pose in the world frame
            //                // get the occupied cells
            robot_state.updateLinkTransforms();
            Eigen::Isometry3d transform = robot_state.getGlobalLinkTransform(link);
            df->addShapeToField(shape.get(), transform);
            std::vector<std::vector<int> > link_occupied_cells;
            getShapeOccupancy(df, *shape, transform, link_occupied_cells);
            // add the occupied cells to the vector
            occupied_cells.insert(occupied_cells.end(),
                                  link_occupied_cells.begin(),
                                  link_occupied_cells.end());
        }
    }
}

/// \brief Add the robot shape, as specified by its links model and state, to the distance field.
/// \param df The distance field.
/// \param robot_state The robot state.
/// \param move_group The move group object.
/// \return Nothing.
// TODO(yoraish): have this also add the end-effector and any attached objects.
inline void addRobotToDistanceField(
    std::shared_ptr<distance_field::PropagationDistanceField> &df,
    moveit::core::RobotState &robot_state,
    const std::vector<std::string> &move_group_names) {
    // Get the collision models of the robot
    std::vector<const moveit::core::LinkModel *> link_models;

    for (std::string move_group_name : move_group_names) {
        std::vector<const moveit::core::LinkModel *> link_models_group = robot_state.getJointModelGroup(move_group_name)
            ->getLinkModels();
        link_models.insert(link_models.end(), link_models_group.begin(), link_models_group.end());
    }

    // TODO(yoraish): Get the collision models of the links in the attached objects.

    // get all the occupied cells
    for (auto &link : link_models) {
        if (link->getName() == "world") {
            continue;
        }
        if (link->getShapes().empty())
            continue;
        // for each shape in the link model get the occupied cells
        for (auto &shape : link->getShapes()) {
            // get the link pose in the world frame
            //                // get the occupied cells
            robot_state.updateLinkTransforms();
            Eigen::Isometry3d transform = robot_state.getGlobalLinkTransform(link);
            df->addShapeToField(shape.get(), transform);
        }
    }
}

/// \brief Visualize the occupied cells in the distance field
/// \param df The distance field
/// \param publisher The publisher object
/// \param frame_id The frame id
/// \param id The marker id
inline void visualizeOccupancy(const std::shared_ptr<distance_field::PropagationDistanceField> &df,
                               const ros::Publisher &publisher,
                               const std::string &frame_id,
                               int id = 1) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "occupied_cells";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = df->getResolution();
    marker.scale.y = df->getResolution();
    marker.scale.z = df->getResolution();
    marker.color.a = 0.3;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    size_t num_occupied_cells = 0;
    for (int x{0}; x < df->getXNumCells(); x++) {
        for (int y{0}; y < df->getYNumCells(); y++) {
            for (int z{0}; z < df->getZNumCells(); z++) {
                if (df->getCell(x, y, z).distance_square_ == 0) {
                    geometry_msgs::Point p;
                    df->gridToWorld(x, y, z, p.x, p.y, p.z);
                    marker.points.push_back(p);
                    num_occupied_cells++;
                }
            }
        }
    }
    ROS_DEBUG_STREAM("Added " << num_occupied_cells << " occupied cells to the marker array" << std::endl);
    publisher.publish(marker);
}

/// \brief Visualize the distance field bounding box in rviz
/// \param df The distance field
/// \param marker_pub The marker publisher
/// \param frame_id The frame id
inline void visualizeBoundingBox(std::shared_ptr<distance_field::PropagationDistanceField> &df,
                                 ros::Publisher &marker_pub,
                                 const std::string &frame_id) {
    int id{1};
    int x_min{0}, x_max{df->getXNumCells()}, y_min{0}, y_max{df->getYNumCells()}, z_min{0}, z_max{df->getZNumCells()};
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "bounding_box";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    geometry_msgs::Point p;
    ros::Duration(1).sleep();
    df->gridToWorld(x_min, y_min, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_min, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_min, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_min, y_max, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_min, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_min, y_min, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_max, y_min, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_min, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_max, y_min, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_max, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_max, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_max, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_max, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_min, y_max, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_max, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_max, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_max, y_max, z_min, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_max, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_max, y_min, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_max, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_min, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_max, y_min, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);

    marker.points.clear();

    df->gridToWorld(x_min, y_min, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    df->gridToWorld(x_min, y_max, z_max, p.x, p.y, p.z);
    marker.points.push_back(p);
    marker.header.stamp = ros::Time();
    marker.id = id;
    ++id;
    marker_pub.publish(marker);
}

inline void moveitCollisionResultToCollisionsCollective(const collision_detection::CollisionResult &collision_result,
                                                        CollisionsCollective &collisions,
                                                        std::string agent_name_prefix = "",
                                                        // The name of the agent that we are checking collisions for. Must be a prefix of the body names.
                                                        std::vector<std::string> other_agent_name_prefixes = {})
// The names of the other agents.
{
    // Add "base" to the list of other agents prefixes.
    // TODO(yoraish): this is a hack to remove. The reason is that the base is not considered an agent in the collision detection.
    other_agent_name_prefixes.emplace_back("base");

    collisions.clear();
    for (const auto &collision_pair_and_contacts : collision_result.contacts) {
        const auto &contacts = collision_pair_and_contacts.second;
        Collision c;
        c.body_name_0 = collision_pair_and_contacts.first.first;
        c.body_name_1 = collision_pair_and_contacts.first.second;

        // A collision may be discarded if it is between two agents that we are not checking collisions for.
        bool discard_collision = false;

        for (const auto &contact : contacts) {
            // Create a contact objects and add it to the collision object. Here, we need to change the body type from moveit to our new type.

            c.contacts.emplace_back(Contact(contact.body_name_1,
                                            contact.body_name_2,
                                            contact.body_type_1,
                                            contact.body_type_2,
                                            contact.pos,
                                            contact.normal));

            // Update the collision type of the collision object. This information is unfortunately only available in the contact object, so we take it from here.
            if ((c.body_name_0 == c.contacts.back().body_name_0 && c.body_name_1 == c.contacts.back().body_name_1)) {
                c.body_type_0 = c.contacts.back().body_type_0;
                c.body_type_1 = c.contacts.back().body_type_1;
            } else if (c.body_name_0 == c.contacts.back().body_name_1 && c.body_name_1 == c.contacts.back().
                body_name_0) {
                c.body_type_0 = c.contacts.back().body_type_1;
                c.body_type_1 = c.contacts.back().body_type_0;
            } else {
                // Raise exception.
                ROS_ERROR_STREAM(
                    "moveitCollisionResultToCollisionsCollective: The collision object names " << c.body_name_0 <<
                    " and " << c.body_name_1 << " do not match the contact object names " << c.contacts.back().
                    body_name_0 << " and " << c.contacts.back().body_name_1 << ".");
            }

            // Check whether this collision is to be kept. First, if the self agent is in one of the collision objects.
            std::string body_name_0_prefix = c.body_name_0.substr(0, c.body_name_0.find('_'));
            std::string body_name_1_prefix = c.body_name_1.substr(0, c.body_name_1.find('_'));
            if (agent_name_prefix != "") {
                // Check for self-collision.
                if (body_name_0_prefix == agent_name_prefix && body_name_1_prefix == agent_name_prefix) {
                    // This is a self collision. Keep it.
                    continue;
                }

                // Now check if this is a collision between the self agent and a world object.
                if ((body_name_0_prefix == agent_name_prefix && c.body_type_1 == BodyType::WORLD_OBJECT) ||
                    (body_name_1_prefix == agent_name_prefix && c.body_type_0 == BodyType::WORLD_OBJECT)) {
                    // This is a collision between the self agent and a world object. Keep it.
                    continue;
                }

                // Now check if this is a collision between the self agent and another agent.
                if ((body_name_0_prefix == agent_name_prefix && std::find(
                        other_agent_name_prefixes.begin(),
                        other_agent_name_prefixes.end(),
                        body_name_1_prefix) != other_agent_name_prefixes.end()) ||
                    (body_name_1_prefix == agent_name_prefix && std::find(
                        other_agent_name_prefixes.begin(),
                        other_agent_name_prefixes.end(),
                        body_name_0_prefix) != other_agent_name_prefixes.end())) {
                    // This is a collision between the self agent and another agent. Keep it.
                    continue;
                }

                // If we got here, this is a collision between two agents that we are not checking collisions for. Discard it.
                discard_collision = true;
            }

            // If the self agent is not specified but only the others, check that all collision bodies are either in the other agents or are world objects.
            else if (other_agent_name_prefixes.size() > 0) {
                // Check if this is a collision between two other agents.
                if (std::find(other_agent_name_prefixes.begin(), other_agent_name_prefixes.end(), body_name_0_prefix) !=
                    other_agent_name_prefixes.end() &&
                    std::find(other_agent_name_prefixes.begin(), other_agent_name_prefixes.end(), body_name_1_prefix) !=
                    other_agent_name_prefixes.end()) {
                    // This is a collision between two other agents. Keep it.
                    continue;
                }

                // Now check if this is a collision between an other agent and a world object.
                if ((std::find(other_agent_name_prefixes.begin(), other_agent_name_prefixes.end(), body_name_0_prefix)
                        != other_agent_name_prefixes.end() && c.body_type_1 == BodyType::WORLD_OBJECT) ||
                    (std::find(other_agent_name_prefixes.begin(), other_agent_name_prefixes.end(), body_name_1_prefix)
                        != other_agent_name_prefixes.end() && c.body_type_0 == BodyType::WORLD_OBJECT)) {
                    // This is a collision between an other agent and a world object. Keep it.
                    continue;
                }

                // If we got here, this is a collision between two agents that we are not checking collisions for. Discard it.
                discard_collision = true;
            }
        }

        if (!discard_collision) {
            collisions.addCollision(c);
        }
    }

    return;
}

/// @brief Convert a mapping between robot names to states to a composite state given an ordering of the robots.
inline void namedMultiAgentStateToStackedState(std::unordered_map<std::string, StateType> multi_state,
                                               std::vector<std::string> robot_names,
                                               StateType &stacked_state) {
    stacked_state.clear();
    for (const auto &robot_name : robot_names) {
        stacked_state.insert(stacked_state.end(), multi_state[robot_name].begin(), multi_state[robot_name].end());
    }
}

inline void densifyPath(PathType &path, int num_intermediate_states = 3) {
    // Add intermediate states to the path. These are interpolated between the states of the individual paths to verify transitions are okay.
    PathType path_dense;

    for (int t = 0; t < path.size() - 1; t++) {
        StateType state_t = path[t];
        StateType state_t1 = path[t + 1];

        // Add the intermediate states.
        for (int i = 0; i <= num_intermediate_states; i++) {
            StateType state_t_intermediate = state_t;
            for (int j = 0; j < state_t.size(); j++) {
                state_t_intermediate[j] = state_t[j] + (state_t1[j] - state_t[j]) * i / (num_intermediate_states + 1);
            }
            path_dense.push_back(state_t_intermediate);
        }
    }

    // Add the last state.
    path_dense.push_back(path.back());
    path = path_dense;
}

inline void densifyMultiAgentPaths(MultiAgentPaths &paths, int num_intermediate_states = 3) {
    // Iterate over the paths and densify them.
    for (auto &agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        PathType &path = agent_id_and_path.second;
        densifyPath(path, num_intermediate_states);
    }
}

std::unordered_map<int, bool> isMultiAgentPathValid(MultiAgentPaths paths,
                                                    const moveit::planning_interface::MoveGroupInterface &
                                                    move_group_multi,
                                                    std::unordered_map<int, std::string> agent_names,
                                                    const planning_scene::PlanningScenePtr &planning_scene,
                                                    int num_intermediate_states = 3) {
    // Create a composite-state path. This is of length t_max, and each entry is a vector of the stacked states, in order 0,1,2,...,n.
    PathType agent_multi_path;

    for (auto agent_id{0}; agent_id < paths.size(); agent_id++) {
        PathType agent_path = paths[agent_id];

        // Stack the states of all robots.
        for (int t = 0; t < agent_path.size(); t++) {
            if (agent_multi_path.size() <= t) {
                // Add a new entry to the path.
                StateType agent_multi_state;
                agent_multi_path.push_back(agent_multi_state);
            }
            StateType agent_multi_state = agent_multi_path[t];

            StateType state_wo_time = agent_path[t];
            state_wo_time.pop_back(); // Remove the time.

            // Add the state to the agent multi state.
            agent_multi_state.insert(agent_multi_state.end(), state_wo_time.begin(), state_wo_time.end());
            agent_multi_path[t] = agent_multi_state;
        }
    }

    // Add intermediate states to the path. These are interpolated between the states of the individual paths to verify transitions are okay.
    densifyPath(agent_multi_path, num_intermediate_states);

    // Iterate over the composite state path and check for validity.
    collision_detection::CollisionRequest collision_request;
    collision_request.verbose = false;
    collision_request.group_name = move_group_multi.getName();
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState robot_state = planning_scene->getCurrentStateNonConst();
    std::unordered_map<int, bool> agent_validity;

    // Set all the agents to valid.
    for (int agent_id = 0; agent_id < paths.size(); agent_id++) {
        agent_validity[agent_id] = true;
    }

    // Iterate over the states and mark any agents that is in collision as invalid.
    for (int t = 0; t < agent_multi_path.size(); t++) {
        StateType agent_multi_state = agent_multi_path[t];

        // Check if the state is valid.
        robot_state.setJointGroupPositions(move_group_multi.getName(), agent_multi_state);
        collision_result.clear();
        planning_scene->checkCollision(collision_request, collision_result, robot_state);

        // If there is no collision, then the the state is valid for all agents.
        if (!collision_result.collision) {
            continue;
        }

        // Check which agents are in collision.
        else {
            for (auto contact : collision_result.contacts) {
                std::string contact_body_name0 = contact.first.first;
                std::string contact_body_name1 = contact.first.second;

                // Check which agent / agents are in collision.
                for (int agent_id = 0; agent_id < paths.size(); agent_id++) {
                    std::string agent_name = agent_names[agent_id];

                    // Check if this is a substring of the contact body name.
                    if (contact_body_name0.find(agent_name) != std::string::npos) {
                        agent_validity[agent_id] = false;
                    }

                    if (contact_body_name1.find(agent_name) != std::string::npos) {
                        agent_validity[agent_id] = false;
                    }
                }
            }
        }
    }

    return agent_validity;
}

/// @brief Smooth a set of paths, one for each agent.
inline bool smoothMultiAgentPaths(MultiAgentPaths paths,
                                  const moveit::planning_interface::MoveGroupInterface &move_group_multi,
                                  const planning_scene::PlanningScenePtr &planning_scene,
                                  std::unordered_map<int, std::string> agent_names,
                                  MultiAgentPaths &smoothed_paths,
                                  double timeout = 1.0) {
    // Keep track of the current t0 and t1 for each agent. These are indices pointing to the current start and end times of the tentative addition to the smoothed path. This addition is an interpolation between the current t0 and t1 on the individual path.
    std::unordered_map<int, int> agent_t0s;
    std::unordered_map<int, int> agent_t1s;
    for (auto agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        agent_t0s[agent_id] = 0;
        agent_t1s[agent_id] = 1;
    }

    // Get the maximal time of the paths.
    double t_max = 0;
    std::unordered_map<int, int> agent_t_max;
    for (auto agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        PathType path = agent_id_and_path.second;
        double path_time = path.back().back();
        if (path_time > t_max) {
            t_max = path_time;
        }
        agent_t_max[agent_id] = path_time;
    }

    // Pad all the paths to the same length.
    for (auto &agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        PathType &path = agent_id_and_path.second;
        double path_time = path.back().back();
        if (path_time < t_max) {
            // Pad the path.
            StateType last_state = path.back();
            for (double t = path_time + 1; t <= t_max; t++) {
                StateType state = last_state;
                state.back() = t;
                path.push_back(state);
            }
        }
    }

    // If the length of the paths is less than or equal to 2, then there is nothing to smooth.
    if (t_max <= 2) {
        smoothed_paths = paths;
        return true;
    }

    // Keep track of the current shortcut segment for each agent.
    std::unordered_map<int, PathType> agent_shortcut_segments;

    // Iterate over the timesteps.
    for (TimeType t{2}; t < t_max; t++) {
        std::cout << "\n========\nStarting smoothing iteration at t= " << t << "\n";

        // Iterate over the agents and check if the transition from min(agent_t0s) to max(agent_t1s) is valid for the multi-agent transition. For each agent that fails, add the transition from t0 to t1-1 to the smoothed path and set the t0 and t1 to t1 and t1+1, respectively.
        int t0 = std::numeric_limits<int>::max();
        int t1 = t;
        // Set all the agent_t1s to the current t.
        for (auto agent_id_and_path : paths) {
            int agent_id = agent_id_and_path.first;
            agent_t1s[agent_id] = t;

            if (agent_t0s[agent_id] < t0) {
                t0 = agent_t0s[agent_id];
            }
        }

        // Construct the multi-agent path to test for validity.
        MultiAgentPaths paths_to_test;
        for (auto agent_id_and_path : paths) {
            int agent_id = agent_id_and_path.first;
            PathType agent_path_to_test;

            // There are two portions to the path to test. A portion from the smoothed path (the portion that has already been added), and a portion interpolating between the poses at t0 and t1 on the individual path.

            // If the smoothed path for this agent is longer than t0, then add the portion of the smoothed path to the path to test. This includes the states up to t0 and not including t0 of this agent.
            if (agent_t0s[agent_id] > t0) {
                for (int tt = t0; tt < agent_t0s[agent_id]; tt++) {
                    // std::cout << "Adding smoothed path state t= " << tt << " to test for agent " << agent_id << "\n";
                    agent_path_to_test.push_back(smoothed_paths[agent_id][tt]);
                }
                std::cout << ">>> Agent " << agent_id << " uses smooth path between t0= " << t0 << " and agent_t0= " <<
                    agent_t0s[agent_id] << "\n";
            }

            // For the portion interpolating between t0 and t1, add the interpolated state to the path to test. This includes the state at t0 and t1.
            for (int tt = agent_t0s[agent_id]; tt <= agent_t1s[agent_id]; tt++) {
                // Get the state at t0 and t1.
                StateType state_t0 = paths[agent_id][agent_t0s[agent_id]];
                StateType state_t1 = paths[agent_id][agent_t1s[agent_id]];

                // Interpolate between the states.
                StateType state_t = state_t0;
                for (int i = 0; i < state_t.size(); i++) {
                    state_t[i] = state_t0[i] + (state_t1[i] - state_t0[i]) * (tt - agent_t0s[agent_id]) / (agent_t1s[
                        agent_id] - agent_t0s[agent_id]);
                }

                // Add the state to the path to test.
                agent_path_to_test.push_back(state_t);
            }
            std::cout << ">>> Agent " << agent_id << " uses interpolated path between agent_t0= " << agent_t0s[agent_id]
                << " and t1= " << agent_t1s[agent_id] << "\n";

            paths_to_test[agent_id] = agent_path_to_test;
        }

        // Test the validity of this multi-agent transition. Do this by asking for paths conflicts.
        std::cout << "testing a transition from t0= " << t0 << " to t1= " << t1 << "\n";
        std::unordered_map<int, bool> agent_validity = isMultiAgentPathValid(paths_to_test,
                                                                             move_group_multi,
                                                                             agent_names,
                                                                             planning_scene,
                                                                             3);

        // For each agent that succeeded, Do nothing. Store the current path extension as the recent successful path extension.
        for (auto agent_id_and_path : paths) {
            int agent_id = agent_id_and_path.first;
            if (agent_validity[agent_id]) {
                // Store the current segment as successful. This is the paths to test segment between the agent t0 and t1.
                PathType agent_successful_segment;
                for (StateType state : paths_to_test[agent_id]) {
                    if (state.back() >= agent_t0s[agent_id] && state.back() <= agent_t1s[agent_id]) {
                        agent_successful_segment.push_back(state);
                    }
                }
                agent_shortcut_segments[agent_id] = agent_successful_segment;
            }

            // For each agent that failed, add the transition from t0 to t1-1 to the smoothed path (should be stored already) and set the t0 and t1 to t1 and t1+1, respectively.
            else {
                // If there has been no successful transition before, then add the steps on the individual path to the smoothed path.
                if (agent_shortcut_segments[agent_id].size() == 0) {
                    for (int tt = agent_t0s[agent_id]; tt < agent_t1s[agent_id]; tt++) {
                        smoothed_paths[agent_id].push_back(paths[agent_id][tt]);
                    }
                }

                // Otherwise, add the successful segment to the smoothed path.
                else {
                    for (StateType state : agent_shortcut_segments[agent_id]) {
                        smoothed_paths[agent_id].push_back(state);
                    }
                    agent_shortcut_segments[agent_id].clear();
                }

                // Set the t0 and t1 to t1 and t1+1, respectively.
                agent_t0s[agent_id] = t1;
                agent_t1s[agent_id] = t1 + 1;

                std::cout << ">>> Agent " << agent_id << " failed, setting t0= " << agent_t0s[agent_id] << " and t1= "
                    << agent_t1s[agent_id] << "\n";
            }
        }
    }

    // If the smoothed path segments are non-empty, add them to the smoothed paths.
    for (auto agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        if (agent_shortcut_segments[agent_id].size() > 0) {
            for (StateType state : agent_shortcut_segments[agent_id]) {
                smoothed_paths[agent_id].push_back(state);
            }
        }
    }

    // Add the last state of the paths to the smoothed paths.
    for (auto agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        PathType path = agent_id_and_path.second;
        smoothed_paths[agent_id].push_back(path.back());
    }

    return true;
}

inline bool isPathInCollision(PathType path,
                              const moveit::planning_interface::MoveGroupInterface &move_group,
                              const planning_scene::PlanningScenePtr &planning_scene) {
    // Densify the path.
    densifyPath(path, 200);

    // Iterate over the states in the path and check for validity.
    collision_detection::CollisionRequest collision_request;
    collision_request.verbose = false;
    collision_request.group_name = move_group.getName();
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState robot_state = planning_scene->getCurrentStateNonConst();

    // Iterate over the states.
    for (int t = 0; t < path.size(); t++) {
        StateType agent_state = path[t];

        // Check if the state is valid.
        robot_state.setJointGroupPositions(move_group.getName(), agent_state);
        collision_result.clear();
        planning_scene->checkCollision(collision_request, collision_result, robot_state);

        // If there is no collision, then the state is valid.
        if (!collision_result.collision) {
            continue;
        } else {
            return true;
        }
    }
    // If we got here, then there is no collision.
    return false;
}

/// @brief Shortcut a path with a context of other agents: not changing the path of other agents or colliding with it.
inline bool shortcutPath(std::string agent_name,
                         MultiAgentPaths paths,
                         const moveit::planning_interface::MoveGroupInterface &move_group_multi,
                         const planning_scene::PlanningScenePtr &planning_scene,
                         std::unordered_map<int, std::string> agent_names,
                         MultiAgentPaths &smoothed_paths,
                         double timeout = 1.0) {
    // Keep track of the current t0 and t1 for the agent. These are indices pointing to the current start and end times of the tentative addition to the smoothed path. This addition is an interpolation between the current t0 and t1 on the individual path.
    int agent_t0 = 0;
    int agent_t1 = 1;

    smoothed_paths.clear();

    // Get the agent id, and also populate the smoothed paths with the initial path for all others.
    int agent_id = -1;
    for (auto agent_id_and_name : agent_names) {
        if (agent_id_and_name.second == agent_name) {
            agent_id = agent_id_and_name.first;
        } else {
            smoothed_paths[agent_id_and_name.first] = paths[agent_id_and_name.first];
        }
    }

    // Get the maximal time of the paths.
    double t_max = paths[agent_id].back().back();

    // Pad all the paths to the same length.
    for (auto &agent_id_and_path : paths) {
        int agent_id = agent_id_and_path.first;
        PathType &path = agent_id_and_path.second;
        double path_time = path.back().back();
        if (path_time < t_max) {
            // Pad the path.
            StateType last_state = path.back();
            for (double t = path_time + 1; t <= t_max; t++) {
                StateType state = last_state;
                state.back() = t;
                path.push_back(state);
            }
        }
    }

    // If the length of the paths is less than or equal to 2, then there is nothing to smooth.
    if (t_max <= 2) {
        smoothed_paths = paths;
        return true;
    }

    // Keep track of the current shortcut segment for the agent.
    PathType agent_shortcut_segment;

    // Iterate over the timesteps.
    for (TimeType t{2}; t < t_max; t++) {
        // std::cout << "\n========\nStarting smoothing iteration at t= " << t << "\n";

        // Iterate over the agents and check if the transition from min(agent_t0s) to max(agent_t1s) is valid for the multi-agent transition. For each agent that fails, add the transition from t0 to t1-1 to the smoothed path and set the t0 and t1 to t1 and t1+1, respectively.
        int t0 = agent_t0;
        int t1 = t;

        // Construct the multi-agent path to test for validity.
        MultiAgentPaths paths_to_test;
        for (auto agent_id_and_path : paths) {
            int agent_id1 = agent_id_and_path.first;
            PathType agent_path = agent_id_and_path.second;
            PathType agent_path_to_test;

            // If this is not the agent of interest, then add the portion of the `paths` that was passed for this agent.
            if (agent_id != agent_id1) {
                for (int tt = t0; tt <= t1; tt++) {
                    agent_path_to_test.push_back(paths[agent_id1][tt]);
                }
                // std::cout << ">>> Agent " << agent_id1 << " uses orig path between t0= " << t0 << " and t1= " << t1 << "\n";
            }

            // Otherwise, look at the shortcut path of the agent of interest. This is a portion interpolating between the poses at t0 and t1 on the individual path.
            else {
                // For the portion interpolating between t0 and t1, add the interpolated state to the path to test. This includes the state at t0 and t1.
                for (int tt = t0; tt <= t1; tt++) {
                    // Get the state at t0 and t1.
                    StateType state_t0 = paths[agent_id][t0];
                    StateType state_t1 = paths[agent_id][t1];

                    // Interpolate between the states.
                    StateType state_t = state_t0;
                    for (int i = 0; i < state_t.size(); i++) {
                        state_t[i] = state_t0[i] + (state_t1[i] - state_t0[i]) * (tt - t0) / (t1 - t0);
                    }

                    // Add the state to the path to test.
                    agent_path_to_test.push_back(state_t);
                }
                // std::cout << ">>> Agent " << agent_id << " uses interpolated path between agent_t0= " << t0 << " and t1= " << t1 << "\n";
            }
            paths_to_test[agent_id1] = agent_path_to_test;
        }

        // Test the validity of this multi-agent transition. Do this by asking for paths conflicts.
        // std::cout << "testing a transition from t0= " << t0 << " to t1= " << t1 << "\n";
        std::unordered_map<int, bool> agent_validity = isMultiAgentPathValid(paths_to_test,
                                                                             move_group_multi,
                                                                             agent_names,
                                                                             planning_scene,
                                                                             3);

        // Check if the shortcut is valid. If so, increase t1 by 1. Else, add the shortcut segment to the smoothed path and set t0 to t1 and t1 to t1+1.
        if (agent_validity[agent_id]) {
            // Store the current segment as successful. This is the paths to test segment between the agent t0 and t1.
            PathType agent_successful_segment;
            for (StateType state : paths_to_test[agent_id]) {
                if (state.back() >= t0 && state.back() <= t1) {
                    agent_successful_segment.push_back(state);
                }
            }
            agent_shortcut_segment = agent_successful_segment;
        }

        // If the shortcut failed, add the transition from t0 to t1-1 to the smoothed path (should be stored already) and set the t0 and t1 to t1 and t1+1, respectively.
        else {
            // If there has been no successful transition before, then add the steps on the individual path to the smoothed path.
            if (agent_shortcut_segment.size() == 0) {
                for (int tt = t0; tt < t1; tt++) {
                    smoothed_paths[agent_id].push_back(paths[agent_id][tt]);
                }
            }

            // Otherwise, add the successful segment to the smoothed path.
            else {
                for (StateType state : agent_shortcut_segment) {
                    smoothed_paths[agent_id].push_back(state);
                }
                agent_shortcut_segment.clear();
            }

            // Set the t0 and t1 to t1 and t1+1, respectively.
            agent_t0 = t1;
            agent_t1 = t1 + 1;

            // std::cout << ">>> Agent " << agent_id << " failed, setting t0= " << agent_t0 << " and t1= " << agent_t1 << "\n";
        }
    }

    // If the smoothed path segment is non-empty, add it to the smoothed paths.
    if (agent_shortcut_segment.size() > 0) {
        for (StateType state : agent_shortcut_segment) {
            smoothed_paths[agent_id].push_back(state);
        }
    }

    // Add the last state of the paths to the smoothed paths.
    smoothed_paths[agent_id].push_back(paths[agent_id].back());

    return true;
}

/// @brief Shortcut paths for multiple agents. Going one at a time around.
inline bool shortcutMultiAgentPathsIterative(
    MultiAgentPaths paths,
    const moveit::planning_interface::MoveGroupInterface &move_group_multi,
    const planning_scene::PlanningScenePtr &planning_scene,
    std::unordered_map<int, std::string> agent_names,
    MultiAgentPaths &smoothed_paths,
    double timeout = 1.0) {
    // Keep a copy of the paths, which will be passed to the shortcutPath function.
    MultiAgentPaths paths_copy = paths;

    // Iterate over the agent names.
    for (auto agent_id_and_name : agent_names) {
        int agent_id = agent_id_and_name.first;
        std::string agent_name = agent_id_and_name.second;

        // Shortcut the path for this agent.
        std::cout << "Shortcutting path for agent " << agent_id << "\n";
        MultiAgentPaths smoothed_paths_agent;
        bool success = shortcutPath(agent_name,
                                    paths_copy,
                                    move_group_multi,
                                    planning_scene,
                                    agent_names,
                                    smoothed_paths_agent,
                                    timeout);
        if (!success) {
            std::cout << "Failed to shortcut path for agent " << agent_id << "\n";
            return false;
        }

        // Set the paths_copy to the smoothed paths for this agent.
        paths_copy = smoothed_paths_agent;
    }

    // Set the smoothed paths to the paths_copy.
    smoothed_paths = paths_copy;

    return true;
}

/// @brief Shortcut a path with a context of other agents: not changing the path of other agents or colliding with it.
inline bool shortcutPath(const PathType &path,
                         const moveit::planning_interface::MoveGroupInterface &move_group,
                         const planning_scene::PlanningScenePtr &planning_scene,
                         PathType &smoothed_path,
                         double timeout = 1.0) {
    // start clock
    auto start_time = std::chrono::high_resolution_clock::now();
    // auto object_names = planning_scene->getWorld()->getObjectIds();
    // Keep track of the current t0 and t1 for the agent. These are indices pointing to the current start and end times of the tentative addition to the smoothed path. This addition is an interpolation between the current t0 and t1 on the individual path.
    int agent_t0 = 0;
    int agent_t1 = 1;

    smoothed_path.clear();

    // Get the maximal "time" of the path. This is the number of elements in the path.
    double t_max = path.size();

    // If the length of the paths is less than or equal to 2, then there is nothing to smooth.
    if (t_max <= 2) {
        smoothed_path = path;
        return true;
    }

    // Keep track of the current shortcut segment for the agent.
    PathType agent_shortcut_segment;

    // Iterate over the timesteps.
    for (TimeType t{2}; t < t_max; t++) {
        if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count() > timeout) {
            std::cout << "Timeout reached. Returning the current smoothed path.\n";
            // add the rest of the path to the smoothed path
            for (int tt = agent_t0; tt < t_max; tt++) {
                smoothed_path.push_back(path[tt]);
            }
            return true;
        }
        //         std::cout << "\n========\nStarting smoothing iteration at t= " << t << "\n";
        // Check if the transition from min(agent_t0s) to max(agent_t1s) is valid for the multi-agent transition. For each agent that fails, add the transition from t0 to t1-1 to the smoothed path and set the t0 and t1 to t1 and t1+1, respectively.
        int t0 = agent_t0;
        int t1 = t;

        // Construct the path to test for validity.
        PathType path_to_test;

        // Look at the shortcut path of the agent of interest. This is a portion interpolating between the poses at t0 and t1 on the individual path.
        // For the portion interpolating between t0 and t1, add the interpolated state to the path to test. This includes the state at t0 and t1.
        for (int tt = t0; tt <= t1; tt++) {
            // Get the state at t0 and t1.
            const StateType &state_t0 = path[t0];
            const StateType &state_t1 = path[t1];
            // Interpolate between the states.
            StateType state_t = state_t0;
            for (int i = 0; i < state_t.size(); i++) {
                state_t[i] = state_t0[i] + (state_t1[i] - state_t0[i]) * (tt - t0) / (t1 - t0);
            }
            // Add the state to the path to test.
            path_to_test.push_back(state_t);
        }

        // Test the validity of this multi-agent transition.
        bool is_agent_in_collision = isPathInCollision(path_to_test,
                                                       move_group,
                                                       planning_scene);

        // Check if the shortcut is valid. If so, increase t1 by 1. Else, add the shortcut segment to the smoothed path and set t0 to t1 and t1 to t1+1.
        if (!is_agent_in_collision) {
            // Store the current segment as successful. This is the paths to test segment between the agent t0 and t1.
            PathType agent_successful_segment;
            for (StateType state : path_to_test) {
                if (state.back() >= t0 && state.back() <= t1) {
                    agent_successful_segment.push_back(state);
                }
            }
            agent_shortcut_segment = agent_successful_segment;
        }

        // If the shortcut failed, add the transition from t0 to t1-1 to the smoothed path (should be stored already) and set the t0 and t1 to t1 and t1+1, respectively.
        else {
            // If there has been no successful transition before, then add the steps on the individual path to the smoothed path.
            if (agent_shortcut_segment.empty()) {
                for (int tt = t0; tt < t1; tt++) {
                    smoothed_path.push_back(path[tt]);
                }
            }

            // Otherwise, add the successful segment to the smoothed path.
            else {
                for (const StateType &state : agent_shortcut_segment) {
                    smoothed_path.push_back(state);
                }
                agent_shortcut_segment.clear();
            }

            // Set the t0 and t1 to t1 and t1+1, respectively.
            agent_t0 = t1;
            agent_t1 = t1 + 1;

            //             std::cout << ">>> Agent failed, setting t0= " << agent_t0 << " and t1= " << agent_t1 << "\n";
        }
    }

    // If the smoothed path segment is non-empty, add it to the smoothed paths.
    if (!agent_shortcut_segment.empty()) {
        for (StateType state : agent_shortcut_segment) {
            smoothed_path.push_back(state);
        }
    }

    // Add the last state of the path to the smoothed path.
    smoothed_path.push_back(path.back());

    return true;
}

/// @brief Interpolate path between two states
/// @param start The start state
/// @param end The end state
/// @param resolution The resolution of the path (default: 0.005 rad)
/// @return The interpolated path
inline PathType interpolatePath(const StateType &start,
                                const StateType &end,
                                const double resolution = 0.05) {
    // TODO: Currently only works for configuration space
    assert(start.size() == end.size());
    PathType path;
    // get the maximum distance between the two states
    double max_distance{0.0};
    for (int i{0}; i < start.size(); i++) {
        double distance = std::abs(start[i] - end[i]);
        if (distance > max_distance) {
            max_distance = distance;
        }
    }
    // calculate the number of steps
    int steps = std::ceil(max_distance / resolution);
    // interpolate the path
    for (int i{0}; i < steps; i++) {
        StateType state;
        for (int j{0}; j < start.size(); j++) {
            state.push_back(start[j] + (end[j] - start[j]) * i / steps);
        }
        path.push_back(state);
    }
    return path;
}
} // namespace ims

#endif  // MANIPULATION_PLANNING_UTILS_HPP
