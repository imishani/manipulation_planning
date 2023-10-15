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

#include <vector>

#include <ros/ros.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/distance_field/distance_field.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <search/common/types.hpp>
#include <search/common/collisions.hpp>

namespace ims {

    /// \brief A function that converts a state from radians to degrees
    /// \param state The state
    template <typename T>
    void rad2deg(std::vector<T>& state, const std::vector<bool> & valid_mask = std::vector<bool>()){
        bool is_valid_mask_passed = !valid_mask.empty();

        for (size_t i = 0; i < state.size(); ++i) {
            // If the valid mask is passed, then check if the dimension is valid.
            if (is_valid_mask_passed){
                if (!valid_mask[i]){
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
    template <typename T>
    void deg2rad(std::vector<T>& state, const std::vector<bool> & valid_mask = std::vector<bool>()){

        // If the mask is not passed, then all the dimensions are assumed to be valid.
        bool is_valid_mask_passed = !valid_mask.empty();
    
        for (size_t i = 0; i < state.size(); ++i) {
            // If the valid mask is passed, then check if the dimension is valid.
            if (is_valid_mask_passed){
                if (!valid_mask[i]){
                    continue;
                }
            }

            // Convert the dimension, if it is valid according to the mask.
            state[i] = state[i] * M_PI / 180;
        }
    }

    inline void negateState(StateType& state, const std::vector<bool> & valid_mask = std::vector<bool>()){
        // If the mask is not passed, then all the dimensions are assumed to be valid.
        bool is_valid_mask_passed = !valid_mask.empty();

        for (size_t i = 0; i < state.size(); ++i) {
            // If the valid mask is passed, then check if the dimension is valid.
            if (is_valid_mask_passed){
                if (!valid_mask[i]){
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
    template <typename T>
    inline void normalizeAngles(std::vector<T>& state, std::vector<std::pair<T, T>>& joint_limits, const std::vector<bool> & valid_mask = std::vector<bool>()){
        // If the mask is not passed, then all the dimensions are assumed to be valid.
        bool is_valid_mask_passed = !valid_mask.empty();

        // Iterate through the state and normalize the dimensions that are valid.
        for (int i = 0; i < state.size(); ++i) {
            // If the valid mask is passed, then check if the dimension is valid.
            if (is_valid_mask_passed){
                if (!valid_mask[i]){
                    continue;
                }
            }

            // Normalize the dimension, if it is not in the range of the joint limits and it is valid according to the mask.
            if (state[i] > joint_limits[i].second){
                state[i] = state[i] - 2*M_PI;
            }
            else if (state[i] < joint_limits[i].first){
                state[i] = state[i] + 2*M_PI;
            }
        }
    }

    /// \brief A function that dealing with the discontinuity of the joint angles
    /// \note This function assumes that the joint limits are [-pi, pi]
    /// \param state The state to check
    /// \param valid_mask The valid mask of the state. Dimensions that are true, get normalized. Others are not.
    /// \return The state with the joint angles in the range of [-pi, pi]
    template <typename T>
    inline void normalizeAngles(std::vector<T>& state, const std::vector<bool> & valid_mask = std::vector<bool>()){
        bool is_valid_mask_passed = !valid_mask.empty();
        // Iterate through the state and normalize the dimensions that are valid.
        for (int i = 0; i < state.size(); ++i) {

            // If the valid mask is passed, then check if the dimension is valid.
            if (is_valid_mask_passed){
                if (!valid_mask[i]){
                    continue;
                }
            }
            
            // Normalize the dimension, if it is not in the range of the joint limits and it is valid according to the mask.
            if (state[i] > M_PI){
                state[i] = state[i] - 2*M_PI;
            }
            else if (state[i] < -M_PI){
                state[i] = state[i] + 2*M_PI;
            }
        }
    }


    template <typename T>
    void checkFixGimbalLock(T y, T p, T r){
        // check if current state in gimbal lock
        /// TODO: Implement

    }

    template <typename T>
    void get_euler_zyx(const Eigen::Matrix<T, 3, 3>& rot, T& y, T& p, T& r)
    {
        y = std::atan2(rot(1, 0), rot(0, 0));
        p  = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
        r = std::atan2(rot(2, 1), rot(2, 2));
    }

    template <typename T>
    void get_euler_zyx(const Eigen::Quaternion<T>& rot, T& y, T& p, T& r)
    {
        Eigen::Matrix<T, 3, 3> R(rot);
        get_euler_zyx(R, y, p, r);
    }

    template <typename T>
    void from_euler_zyx(T y, T p, T r, Eigen::Matrix<T, 3, 3>& rot)
    {
        rot = Eigen::AngleAxis<T>(y, Eigen::Matrix<T, 3, 1>::UnitZ()) *
              Eigen::AngleAxis<T>(p, Eigen::Matrix<T, 3, 1>::UnitY()) *
              Eigen::AngleAxis<T>(r, Eigen::Matrix<T, 3, 1>::UnitX());
    }

    template <typename T>
    void from_euler_zyx(T y, T p, T r, Eigen::Quaternion<T>& q)
    {
        Eigen::Matrix<T, 3, 3> R;
        from_euler_zyx(y, p, r, R);
        q = Eigen::Quaternion<T>(R);
    }

    template <typename T>
    void from_euler_zyx(T y, T p, T r, geometry_msgs::Pose& q)
    {
        Eigen::Matrix<T, 3, 3> R;
        from_euler_zyx(y, p, r, R);
        Eigen::Quaternion<T> quat(R);
        tf::quaternionEigenToMsg(quat, q.orientation);
    }

    template <typename T>
    void normalize_euler_zyx(T& y, T& p, T& r)
    {
        Eigen::Matrix<T, 3, 3> rot;
        from_euler_zyx(y, p, r, rot);
        get_euler_zyx(rot, y, p, r);
    }

    template <typename T>
    void normalize_euler_zyx(T* angles) // in order r, p, y
    {
        Eigen::Matrix<T, 3, 3> rot;
        from_euler_zyx(angles[2], angles[1], angles[0], rot);
        get_euler_zyx(rot, angles[2], angles[1], angles[0]);
    }

    /// \brief Convert hopf coordinates to quaternion
    /// \param hopf The hopf coordinates vector
    /// \param q The quaternion (by reference)
    inline void hopfToQuaternion(const Eigen::Vector3d& hopf, Eigen::Quaterniond& q){
        double theta = hopf[0];
        double phi = hopf[1];
        double psi = hopf[2];
        q.w() = cos(theta/2.0)*cos(psi/2.0);
        q.x() = cos(theta/2.0)*sin(psi/2.0);
        q.y() = sin(theta/2.0)*cos(phi + psi/2.0);
        q.z() = sin(theta/2.0)*sin(phi + psi/2.0);
    }

    /// \brief Convert quaternion to hopf coordinates
    /// \param q The quaternion
    /// \param hopf The hopf coordinates vector (by reference)
    /// The range of the hopf coordinates is the following:
    /// theta: [0, pi]
    /// phi: [0, 2pi]
    /// psi: [0, 2pi]
    inline void quaternionToHopf(const Eigen::Quaterniond& q, Eigen::Vector3d& hopf){
        double x = q.x();
        double y = q.y();
        double z = q.z();
        double w = q.w();
        // Make sure that w >= 0 and if not, negate the quaternion
        if (w < 0.0){
            x = -x;
            y = -y;
            z = -z;
            w = -w;
        }
        hopf[2] = 2.0*atan2(x, w);
        hopf[1] = atan2(z, y) - hopf[2]/2.0;
        if (hopf[1] < 0.0){
            hopf[1] += 2.0*M_PI;
        }
        hopf[0] = 2.0*atan2(sqrt(y*y + z*z), sqrt(x*x + w*w));
    }


    /// \brief A function that takes the discretization vector and a state, and round it to the nearest discretization value
    /// \param state The state to check
    /// \param discretization The discretization vector
    inline void roundStateToDiscretization(StateType& state, const StateType& discretization){
        for (int i = 0; i < state.size(); ++i) {
            state[i] = round(state[i]/discretization[i])*discretization[i];
        }
    }


    /// \brief Read a txt file and return a vector of points
    /// \param file_path The path to the file
    /// \return The vector of points
    inline std::vector<std::vector<double>> readPointsFromFile(const std::string& file_path){
        std::vector<std::vector<double>> points;
        std::ifstream file(file_path);
        std::string line;
        while (std::getline(file, line)){
            std::vector<double> point;
            std::stringstream ss(line);
            std::string value;
            while (std::getline(ss, value, ',')){
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
    template <typename T>
    inline int findNearestPoint(const T& point, const std::vector<T>& points){
        int index = 0;
        double min_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < points.size(); ++i) {
            double distance = std::abs(point - points[i]);
            if (distance < min_distance){
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
    inline double geodesicDistance(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2){
        double theta1 = q1[0];
        double phi1 = q1[1];
        double psi1 = q1[2];
        double theta2 = q2[0];
        double phi2 = q2[1];
        double psi2 = q2[2];
        double distance = acos(cos(theta1)*cos(theta2) + sin(theta1)*sin(theta2)*cos(phi1 - phi2 + psi1 - psi2));
        return distance;
    }


    /// \brief Profile the trajectory
    /// \param start The start joint state. type: StateType
    /// \param goal The goal joint state. type: StateType
    /// \param trajectory a vector of joint states. type: std::vector<StateType>
    /// \param move_group_ The move group object. type: moveit::planning_interface::MoveGroupInterface
    /// \param trajectory_msg The output trajectory. type: moveit_msgs::RobotTrajectory
    /// \return success bool
    inline bool profileTrajectory(const StateType & start,
                                  const StateType & goal,
                                  const std::vector<StateType> & trajectory,
                                  const moveit::planning_interface::MoveGroupInterface & move_group_,
                                  moveit_msgs::RobotTrajectory& trajectory_msg,
                                  double velocity_scaling_factor = 0.2,
                                  double acceleration_scaling_factor = 0.2){

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
        if (!time_param.computeTimeStamps(robot_trajectory, velocity_scaling_factor,
                                          acceleration_scaling_factor)){
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
                               const planning_scene::PlanningScenePtr& planning_scene,
                               double timeout = 0.5){
        // iteratively moving further and further down the path, attempting to replace the original path with
        // shortcut paths. The idea is that each action only moves one angle, and we want to move few angles together if possie
        // to reduce the number of actions
        double start_time = ros::Time::now().toSec();
        if (path.empty())
            return false;
        PathType shortcut_path;
        auto last_state = path.back();
        auto current_state = path.front();
        shortcut_path.push_back(current_state);
        // iteratively try to connect the current state to the closest state to the last state
        collision_detection::CollisionRequest collision_request;
        collision_request.verbose = true;
        collision_detection::CollisionResult collision_result;
        robot_state::RobotState robot_state = planning_scene->getCurrentStateNonConst();
        bool is_timeout = false;
        while (current_state != last_state && !is_timeout){
            for (size_t i = path.size() - 1; true ; i--) {
                StateType state_to_shortcut = path[i];
                if (state_to_shortcut == current_state) {
                    current_state = path[i + 1];
                    break;
                }
                // try to check if the current state can be connected to the state to shortcut by interpolating
                PathType interpolated_path;
                // step size 0.05
                double step_size = 0.1;
                // get the number of steps
                int num_steps = std::ceil(1.0 / step_size);
                // interpolate between the current state and the state to shortcut
                for (int j = 1; j <= num_steps; ++j) {
                    StateType interpolated_state;
                    for (size_t k = 0; k < current_state.size(); k++) {
                        interpolated_state.push_back(
                            current_state[k] + step_size * j * (state_to_shortcut[k] - current_state[k]));
                    }
                    // check if the interpolated state is in collision
                    robot_state.setJointGroupPositions(move_group.getName(), interpolated_state);
                    // reset the collision result
                    collision_result.clear();
                    planning_scene->checkCollision(collision_request, collision_result, robot_state);
                    if (collision_result.collision) {
                        // if the interpolated state is in collision, then break
                        break;
                    } else {
                        interpolated_path.push_back(interpolated_state);
                    }
                }
                // if the interpolated path is not empty, then add it to the shortcut path
                if (!collision_result.collision) {
                    // add the interpolated path to the shortcut path
                    for (auto& state : interpolated_path) {
                        shortcut_path.push_back(state);
                    }
                    // set the current state to the last state of the interpolated path
                    current_state = state_to_shortcut;
                    break;
                }
            }
            if (ros::Time::now().toSec() - start_time > timeout) {
                is_timeout = true;
            }
        }
        // check if the timeout is reached
        if (is_timeout){
            // add the rest of the path from current state to the last state
            PathType rest_of_path;
            // loop in reverse order
            for (size_t i = path.size() - 1; i > 0; --i) {
                if (path[i] == current_state){
                    break;
                }
                rest_of_path.push_back(path[i]);
            }
            // reverse the rest of the path
            std::reverse(rest_of_path.begin(), rest_of_path.end());
            // add the rest of the path to the shortcut path
            for (auto& state : rest_of_path) {
                shortcut_path.push_back(state);
            }
        }
        shortcut_path.push_back(last_state);
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
                                                                                      double max_distance = 1.8){
        std::shared_ptr<distance_field::PropagationDistanceField> df(new distance_field::PropagationDistanceField(
                df_size_x, df_size_y, df_size_z,
                df_res, df_origin_x, df_origin_y, df_origin_z,
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
                                                                                            double max_distance = 1.8){

        auto df = std::make_shared<distance_field::PropagationDistanceField>(df_size_x, df_size_y, df_size_z,
                                                                             df_res, df_origin_x, df_origin_y, df_origin_z,
                                                                             max_distance);
        // get the planning scene interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // get the collision objects
        auto objects = planning_scene_interface.getObjects();
        std::vector<moveit_msgs::CollisionObject> objs;
        for (auto& obj : objects) {
            objs.push_back(obj.second);
        }
        // fill the distance field
        for (auto& obj : objs) {

            Eigen::Isometry3d pose;
            // convert from pose to eigen
            geometry_msgs::Pose pose_msg = obj.pose;
            tf::poseMsgToEigen(pose_msg, pose);
            shapes::Shape* shape;
            // get shape from collision object
            if (obj.primitives.empty() && obj.meshes.empty()) {
                // raise exception
                std::cout << "No shape in collision object" << std::endl;
                std::exception e;
            }
            else if (!obj.primitives.empty()){
                // cnsrtuct shape from primitive;
                shape = shapes::constructShapeFromMsg(obj.primitives[0]);
            }
            else {
                shape = shapes::constructShapeFromMsg(obj.meshes[0]);
            }
            df->addShapeToField(shape, pose);
        }
        return df;
    }

    /// \brief Check if a point is in collision
    /// \param df The distance field
    /// \param point The point to check
    /// \return True if the point is in collision
    inline bool isPointInCollision(const distance_field::PropagationDistanceField &df, const Eigen::Vector3d &point){
        int x, y, z;
        df.worldToGrid(point[0], point[1], point[2], x, y, z);
        return df.getCell(x, y, z).distance_square_ == 0;
    }

    /// \brief Check if cell is occupied
    /// \param df The distance field
    /// \param cell The cell to check
    /// \return True if the cell is occupied
    inline bool isCellOccupied(const distance_field::PropagationDistanceField &df, const Eigen::Vector3i &cell){
        return df.getCell(cell[1], cell[1], cell[2]).distance_square_ == 0;
    }

    /// \brief Count occupied cells in the distance field
    /// \param df The distance field
    /// \return The number of occupied cells
    inline unsigned int countOccupiedCells(const distance_field::PropagationDistanceField &df){
        unsigned int count = 0;
        for (int z = 0; z < df.getZNumCells(); z++)
        {
            for (int x = 0; x < df.getXNumCells(); x++)
            {
                for (int y = 0; y < df.getYNumCells(); y++)
                {
                    if (df.getCell(x, y, z).distance_square_ == 0)
                    {
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
    inline void getShapeOccupancy(const std::shared_ptr<distance_field::PropagationDistanceField>& df,
                                  const shapes::Shape &shape,
                                  const Eigen::Transform<double, 3, 1> &pose,
                                  std::vector<std::vector<int>> &occupied_cells){
        /// TODO: Its not working well.

        double radius;
        Eigen::Vector3d center;
        computeShapeBoundingSphere(&shape, center, radius);
        // get the bounding cylinder of the shape
        visualization_msgs::Marker marker;
        constructMarkerFromShape(&shape, marker);
        // loop through the points and add them to the distance field as occupied
        for (auto & i : marker.points)
        {
            // transform point to world frame
            Eigen::Vector3d point(i.x, i.y, i.z);
            point = pose * point;
            // get the grid coordinates
            int x, y, z;
            df->worldToGrid(point.x(), point.y(), point.z(), x, y, z);
            // If the cell is already occupied, skip it
            if (df->getCell(x, y, z).distance_square_ == 0)
            {
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
            const std::shared_ptr<distance_field::PropagationDistanceField>& df,
            moveit::core::RobotState &robot_state,
            const std::unique_ptr<moveit::planning_interface::MoveGroupInterface> &move_group_,
            std::vector<std::vector<int>> &occupied_cells){
        // get the collision models of the robot
        std::vector<const robot_model::LinkModel*> link_models =
                robot_state.getJointModelGroup(move_group_->getName())->getLinkModels();
        // delete all link models besides the first two:
        link_models.erase(link_models.begin(), link_models.begin()+2);

        // get all the occupied cells
        for (auto& link : link_models){
            if (link->getName() == "world"){
                continue;
            }
            if (link->getShapes().empty())
                continue;
            // for each shape in the link model get the occupied cells
            for (auto& shape : link->getShapes()){
                // get the link pose in the world frame
//                // get the occupied cells
                robot_state.updateLinkTransforms();
                Eigen::Isometry3d transform = robot_state.getGlobalLinkTransform(link);
                df->addShapeToField(shape.get(), transform);
                std::vector<std::vector<int>> link_occupied_cells;
                getShapeOccupancy(df, *shape, transform, link_occupied_cells);
                // add the occupied cells to the vector
                occupied_cells.insert(occupied_cells.end(), link_occupied_cells.begin(),
                                      link_occupied_cells.end());
            }
        }
    }

    /// \brief Visualize the occupied cells in the distance field
    /// \param df The distance field
    /// \param publisher The publisher object
    /// \param frame_id The frame id
    /// \param id The marker id
    inline void visualizeOccupancy(const std::shared_ptr<distance_field::PropagationDistanceField>& df,
                                   const ros::Publisher &publisher,
                                   const std::string &frame_id,
                                   int id=1){
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
        for (int x {0} ; x < df->getXNumCells(); x++){
            for (int y{0}; y < df->getYNumCells(); y++){
                for (int z {0} ; z < df->getZNumCells(); z++ ){
                    if (df->getCell(x, y, z).distance_square_ == 0){
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
                              const std::string &frame_id){
        int id {1};
        int x_min {0}, x_max {df->getXNumCells()}, y_min {0}, y_max {df->getYNumCells()}, z_min {0}, z_max {df->getYNumCells()};
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.ns = "bounding_box";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02; marker.scale.y = 0.02; marker.scale.z = 0.02;
        marker.color.a = 0.5; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;

        geometry_msgs::Point p;
        ros::Duration(1).sleep();
        df->gridToWorld(x_min, y_min, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_min, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_min, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_min, y_max, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_min, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_min, y_min, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_max, y_min, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_min, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_max, y_min, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_max, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_max, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_max, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_max, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_min, y_max, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_max, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_max, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_max, y_max, z_min, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_max, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_max, y_min, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_max, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_min, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_max, y_min, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

        marker.points.clear();

        df->gridToWorld(x_min, y_min, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        df->gridToWorld(x_min, y_max, z_max, p.x, p.y, p.z);
        marker.points.push_back(p);
        marker.header.stamp = ros::Time();
        marker.id = id; ++id;
        marker_pub.publish(marker);

    }
        
    inline void moveitCollisionResultToCollisionsCollective(const collision_detection::CollisionResult& collision_result,
                                                            CollisionsCollective& collisions)
    {
        collisions.clear();
        for (const auto& collision_pair_and_contacts : collision_result.contacts)
        {
            const auto& contacts = collision_pair_and_contacts.second;
            Collision c;
            c.body_name_0 = collision_pair_and_contacts.first.first;
            c.body_name_1 = collision_pair_and_contacts.first.second;

            for (const auto& contact : contacts){
                // Create a contact objects and add it to the collision object. Here, we need to change the body type from moveit to our new type.

                c.contacts.emplace_back(Contact(contact.body_name_1, contact.body_name_2, contact.body_type_1, contact.body_type_2, contact.pos, contact.normal));

                // Update the collision type of the collision object. This information is unfortunately only available in the contact object, so we take it from here.
                if ((c.body_name_0 == c.contacts.back().body_name_0 && c.body_name_1 == c.contacts.back().body_name_1) || (c.body_name_0 == c.contacts.back().body_name_1 && c.body_name_1 == c.contacts.back().body_name_0))
                {
                    c.body_type_0 = c.contacts.back().body_type_0;
                    c.body_type_1 = c.contacts.back().body_type_1;
                }
                else{
                    // Raise exception.
                    ROS_ERROR_STREAM("moveitCollisionResultToCollisionsCollective: The collision object names " << c.body_name_0 << " and " << c.body_name_1 << " do not match the contact object names " << c.contacts.back().body_name_0 << " and " << c.contacts.back().body_name_1 << ".");
                }
            }
            collisions.addCollision(c);
        }
    }

} // namespace ims

#endif //MANIPULATION_PLANNING_UTILS_HPP
