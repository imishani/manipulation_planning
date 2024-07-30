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
 * \file   moveit_scene_interface.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/3/23
 */

#ifndef MANIPULATION_PLANNING_MOVEITINTERFACE_HPP
#define MANIPULATION_PLANNING_MOVEITINTERFACE_HPP

// include standard libraries
#include <iostream>
#include <vector>

// include ROS libraries
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>

// project includes
#include <manipulation_planning/common/utils.hpp>
#include <search/common/collisions.hpp>
#include <search/common/scene_interface.hpp>
#include <search/common/types.hpp>
#include <search/common/world_objects.hpp>

namespace ims {

/// @class MoveitInterface
/// @brief A class that implements the SceneInterface for Moveit
/// @details This class implements the SceneInterface for Moveit. It is used to get the current state of the robot and to
/// get all information needed about the scene (e.g. obstacles, robot, etc.)
class MoveitInterface : public SceneInterface {
private:
    bool verbose_ = false;

    // The number of collision checks carried out.
    int num_collision_checks_ = 0;

public:
    /// @brief Constructor
    explicit MoveitInterface(const std::string &group_name) {
        // planning scene monitor
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->requestPlanningSceneState();
        ros::Duration(0.3).sleep();
        planning_scene_ = planning_scene_monitor_->getPlanningScene();
        group_name_ = group_name;

        kinematic_state_ = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
        // joint model group
        joint_model_group_ = kinematic_state_->getJointModelGroup(group_name_);
        // get the joint names
        joint_names_ = joint_model_group_->getVariableNames();
        // get the number of joints
        num_joints_ = joint_names_.size();

        auto object_names = planning_scene_->getWorld()->getObjectIds();

        // Store the joint limits.
        getJointLimits(joint_limits_);

        // Store the frame id.
        frame_id_ = planning_scene_->getPlanningFrame();

        // Instantiate a planning scene interface.
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Get the names for all the move groups in the scene.
        scene_move_group_names_ = planning_scene_->getRobotModel()->getJointModelGroupNames();

        for (auto &obj : object_names) {
            std::cout << "Object name: " << obj << std::endl;
        }
        if (object_names.empty()) {
            std::cout << "No collision objects in the scene" << std::endl;
        }
    };

    /// @brief Constructor with the option to set the planning scene.
    MoveitInterface(const std::string &group_name, planning_scene::PlanningScenePtr &planning_scene) {
        // planning scene monitor
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->requestPlanningSceneState();
        ros::Duration(0.3).sleep();
        planning_scene_ = planning_scene;
        group_name_ = group_name;
        frame_id_ = planning_scene_->getPlanningFrame();

        kinematic_state_ = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
        // joint model group
        joint_model_group_ = kinematic_state_->getJointModelGroup(group_name_);
        // get the joint names
        joint_names_ = joint_model_group_->getVariableNames();
        // get the number of joints
        num_joints_ = joint_names_.size();

        // Store the joint limits.
        getJointLimits(joint_limits_);

        auto object_names = planning_scene_->getWorld()->getObjectIds();

        for (auto &obj : object_names) {
            std::cout << "Object name: " << obj << std::endl;
        }
        if (object_names.empty()) {
            std::cout << "No collision objects in the scene" << std::endl;
        }
    };

    /// @brief Destructor
    ~MoveitInterface() override = default;

    /// @brief check if the state is valid
    /// @param state The state to check
    /// @return True if the state is valid, false otherwise
    bool isStateValid(const StateType &state) {
        moveit_msgs::RobotState robotState;
        // copy the values
        robotState.joint_state.position = state;
        robotState.joint_state.name = joint_names_;

        // Check for the joint limits and if the scene is valid.
        num_collision_checks_++;
        return planning_scene_->isStateValid(robotState, group_name_);
    }

    /// @brief check if the state is valid w.r.t. all other bodies. Robot and non-robot.
    /// @param state the configuration to check
    /// @param collisions_collective the collisions that have been found, potentially.
    /// @return true if the state is valid, false otherwise.
    /// @note This function checks for the current state of the robot against the current state of the scene. The scene is not reset and not other robots' states are changed. If you want to test againts other robots, use the other isStateValid function.
    bool isStateValid(const StateType &state, CollisionsCollective &collisions_collective) {
        // Check the passed state for joint limits.
        if (!isStateWithinJointLimits(state)) {
            return false;
        }

        // Set the state of the ego robot and reset the states of all others.
        robot_state::RobotState &current_scene_state = planning_scene_->getCurrentStateNonConst();
        // Reset the scene.
        current_scene_state.setToDefaultValues();

        // Set the state of the ego robot.
        current_scene_state.setJointGroupPositions(group_name_, state);

        // Check if this configuration is in collision. Whether any robot collides with any other.
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = true;
        collision_request.max_contacts = 1000;
        collision_request.max_contacts_per_pair = 1;
        collision_request.group_name = group_name_;
        planning_scene_->checkCollision(collision_request, collision_result);
        num_collision_checks_++;

        // Convert the collision result to a collision collective.
        std::string agent_name_prefix = group_name_.substr(0, group_name_.find("_"));
        moveitCollisionResultToCollisionsCollective(collision_result, collisions_collective, agent_name_prefix);

        return collisions_collective.size() == 0;
    }

    /// @brief check if the state is valid w.r.t. other bodies.
    /// @param state the configuration to check
    /// @param other_move_group_names the names of the robots to check against.
    /// @param other_move_group_states the states of the robots to check against.
    /// @param collisions_collective the collisions that have been found, potentially.
    /// @return true if the state is valid, false otherwise.
    bool isStateValid(const StateType &state,
                      const std::vector<std::string> &other_move_group_names,
                      const std::vector<StateType> &other_move_group_states,
                      CollisionsCollective &collisions_collective) {
        // Check the passed state for joint limits.
        if (!isStateWithinJointLimits(state)) {
            return false;
        }

        // Set the state of all robots.
        robot_state::RobotState &current_scene_state = planning_scene_->getCurrentStateNonConst();
        current_scene_state.setToDefaultValues();  // TODO(yoraish): do we need this?
        int num_move_groups = other_move_group_names.size();
        for (int i = 0; i < num_move_groups; i++) {
            current_scene_state.setJointGroupPositions(other_move_group_names[i], other_move_group_states[i]);
        }

        // Set the state of the ego robot.
        current_scene_state.setJointGroupPositions(group_name_, state);

        // Check if this configuration is in collision. Whether any robot collides with any other.
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = true;
        collision_request.max_contacts = 1000;
        collision_request.max_contacts_per_pair = 1;
        collision_request.verbose = verbose_;
        collision_request.group_name = group_name_;

        planning_scene_->checkCollision(collision_request, collision_result);
        num_collision_checks_++;

        // Convert the collision result to a collision collective.
        std::string agent_name_prefix = group_name_.substr(0, group_name_.find("_"));
        std::vector<std::string> other_agent_name_prefixes;
        for (auto &other_move_group_name : other_move_group_names) {
            std::string other_agent_name_prefix = other_move_group_name.substr(0, other_move_group_name.find("_"));
            other_agent_name_prefixes.push_back(other_agent_name_prefix);
        }
        moveitCollisionResultToCollisionsCollective(collision_result, collisions_collective, agent_name_prefix, other_agent_name_prefixes);

        return collisions_collective.size() == 0;
    }

    /// @brief check if the state is valid w.r.t. other bodies and other robots in given configurations.
    /// @param state
    /// @param other_move_group_names
    /// @param other_move_group_states
    /// @param sphere_world_objects
    /// @param collisions_collective
    /// @return
    bool isStateValid(const StateType &state,
                      const std::vector<std::string> &other_move_group_names,
                      const std::vector<StateType> &other_move_group_states,
                      const std::vector<SphereWorldObject> &sphere_world_objects,
                      CollisionsCollective &collisions_collective) {
        // Add the objects to the scene. Keep track of the objects.
        std::vector<moveit_msgs::CollisionObject> object_msgs;
        addSpheresToScene(sphere_world_objects, object_msgs);

        // Call the other isStateValid function.
        bool is_valid = isStateValid(state, other_move_group_names, other_move_group_states, collisions_collective);

        // Remove the objects from the scene.
        removeObjectsFromScene(object_msgs);

        return is_valid;
    }

    /// @brief check if the state is valid w.r.t. spheres in it.
    /// @param state
    /// @param other_move_group_names
    /// @param other_move_group_states
    /// @param sphere_world_objects
    /// @param collisions_collective
    /// @return
    bool isStateValid(const StateType &state,
                      const std::vector<SphereWorldObject> &sphere_world_objects,
                      CollisionsCollective &collisions_collective) {
        // Add the objects to the scene. Keep track of the objects.
        std::vector<moveit_msgs::CollisionObject> object_msgs;
        addSpheresToScene(sphere_world_objects, object_msgs);

        // Call the other isStateValid function.
        bool is_valid = isStateValid(state, collisions_collective);

        // Remove the objects from the scene.
        removeObjectsFromScene(object_msgs);

        return is_valid;
    }

    /// @brief A utility function for adding sphere collision object to a planning scene.
    /// @param sphere_world_objects
    /// @param object_msgs the collision objects that were added to the scene.
    /// @return nothing.
    void addSpheresToScene(const std::vector<SphereWorldObject> &sphere_world_objects, std::vector<moveit_msgs::CollisionObject> &object_msgs) {
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();  // TODO(yorais): this should be initialized elsewhere.

        for (auto &obj : sphere_world_objects) {
            // Add the object to the scene with a new name.
            moveit_msgs::CollisionObject collision_object;
            collision_object.id = "world_sphere" + std::to_string(object_msgs.size());
            collision_object.header.frame_id = frame_id_;
            shapes::ShapeMsg collision_object_shape_msg;
            auto *shape = new shapes::Sphere(obj.radius);
            shapes::constructMsgFromShape(shape, collision_object_shape_msg);
            geometry_msgs::Pose collision_object_pose;
            collision_object_pose.position.x = obj.origin.x();
            collision_object_pose.position.y = obj.origin.y();
            collision_object_pose.position.z = obj.origin.z();

            collision_object_pose.orientation.x = 0.0;
            collision_object_pose.orientation.y = 0.0;
            collision_object_pose.orientation.z = 0.0;
            collision_object_pose.orientation.w = 1.0;

            collision_object.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(collision_object_shape_msg));
            collision_object.primitive_poses.push_back(collision_object_pose);
            collision_object.operation = moveit_msgs::CollisionObject::ADD;

            // Update the planning scene.
            planning_scene_interface_->applyCollisionObject(collision_object);  // For visualization. TODO(yoraish): remove.
            planning_scene_->processCollisionObjectMsg(collision_object);       // For collision checking.

            // Add the object to the list of objects.
            object_msgs.push_back(collision_object);
        }
    }

    /// @brief Remove collision objects from the planning scene.
    /// @param object_msgs
    void removeObjectsFromScene(std::vector<moveit_msgs::CollisionObject> object_msgs) {
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();  // TODO(yorais): this should be initialized elsewhere.

        for (auto &obj_msg : object_msgs) {
            obj_msg.operation = obj_msg.REMOVE;
            planning_scene_interface_->applyCollisionObject(obj_msg);  // TODO(yoraish): For viz. Remove this.
            planning_scene_->processCollisionObjectMsg(obj_msg);       // For collision checking.
        }
    }

    /// @brief check if the given state of the specified groups is valid.
    /// @param state the configuration to check
    /// @param other_move_group_names the names of the robots to check against.
    /// @param other_move_group_states the states of the robots to check against.
    /// @param collisions_collective the collisions that have been found, potentially.
    /// @return true if the state is in collision, false otherwise.
    bool checkCollision(const std::vector<std::string> &other_move_group_names,
                        const std::vector<StateType> &other_move_group_states,
                        CollisionsCollective &collisions_collective) {
        // Set the state of all robots.
        robot_state::RobotState &current_scene_state = planning_scene_->getCurrentStateNonConst();
        int num_move_groups = static_cast<int>(other_move_group_names.size());
        for (int i = 0; i < num_move_groups; i++) {
            current_scene_state.setJointGroupPositions(other_move_group_names[i], other_move_group_states[i]);
        }

        // Check if this configuration is in collision. Whether any robot collides with any other.
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = true;
        collision_request.max_contacts = 1000;
        collision_request.max_contacts_per_pair = 1;
        collision_request.verbose = verbose_;

        planning_scene_->checkCollision(collision_request, collision_result);
        num_collision_checks_++;

        // Convert the collision result to a collision collective.
        std::vector<std::string> other_agent_name_prefixes;
        for (auto &other_move_group_name : other_move_group_names) {
            std::string other_agent_name_prefix = other_move_group_name.substr(0, other_move_group_name.find("_"));
            other_agent_name_prefixes.push_back(other_agent_name_prefix);
        }
        moveitCollisionResultToCollisionsCollective(collision_result, collisions_collective, "", other_agent_name_prefixes);

        return collisions_collective.size() > 0;
    }

    /// @brief Check if a path is valid
    /// @param path The path to check
    /// @return True if the path is valid, false otherwise
    bool isPathValid(const PathType &path) {
        // TODO: Is this ok or should i use isPathValid instead? This is great. And I think this should be moved to the action space such that the isStateValid function could be customized.
        return std::all_of(path.begin(), path.end(), [this](const StateType &state_val) { return isStateValid(state_val); });
        // return true;
    }

    /// @brief Calculate IK for a given pose
    /// @param pose The pose to calculate IK for
    /// @param joint_state_seed The joint state to use as a seed.
    /// @param joint_state The joint state to store the IK solution
    /// @param timeout The timeout for the IK calculation
    /// @return True if IK was found, false otherwise
    bool calculateIK(const Eigen::Isometry3d &pose,
                     const StateType &joint_state_seed,
                     StateType &joint_state,
                     double timeout = 0.1) {
        // Convert the input to a geometry_msgs::Pose.
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        // Call the other calculateIK function.
        return calculateIK(pose_msg, joint_state_seed, joint_state, 1.0, timeout);
    }

    /// @brief Calculate IK for a given pose
    /// @param pose The pose to calculate IK for
    /// @param joint_state The joint state to store the IK solution
    /// @param timeout The timeout for the IK calculation
    /// @return True if IK was found, false otherwise
    bool calculateIK(const geometry_msgs::Pose &pose,
                     StateType &joint_state,
                     double timeout = 0.1) {
        // resize the joint state
        joint_state.resize(num_joints_);
        // set joint model group as random, only the relevant kinematic group
        kinematic_state_->setToRandomPositions(joint_model_group_);
        // update
        planning_scene_monitor_->updateFrameTransforms();
        planning_scene_monitor_->updateSceneWithCurrentState();  // TODO: Is this needed?
        // set the pose
        if (kinematic_state_->setFromIK(joint_model_group_, pose, timeout)) {
            // get the joint values
            kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_state);
            return true;
        }
        else {
            ROS_INFO("No IK solution found without using seed");
            return false;
        }
    }

    /// @brief Calculate IK for a given pose and a given seed
    /// @param pose The pose to calculate IK for
    /// @param seed The seed to use for the IK
    /// @param joint_state The joint state to store the IK solution
    /// @param consistency_limit The consistency limit to use for the IK
    /// @param timeout The timeout for the IK calculation
    /// @return True if IK was found, false otherwise
    bool calculateIK(const geometry_msgs::Pose &pose,
                     const StateType &seed,
                     StateType &joint_state,
                     double consistency_limit = 1.0,
                     double timeout = 0.1) {
        // resize the joint state
        joint_state.resize(num_joints_);
        // set the pose
        kinematic_state_->setJointGroupPositions(joint_model_group_, seed);
        // update
        planning_scene_monitor_->updateFrameTransforms();
        planning_scene_monitor_->updateSceneWithCurrentState();
        // add a consistency_limits of 0.1 to the seed
        std::vector<double> consistency_limits(num_joints_, consistency_limit);
        // get the tip link
        const std::string &tip_link = joint_model_group_->getLinkModelNames().back();
        Eigen::Isometry3d pose_eigen;
        tf::poseMsgToEigen(pose, pose_eigen);

        if (kinematic_state_->setFromIK(joint_model_group_, pose_eigen,
                                        tip_link, consistency_limits,
                                        timeout)) {
            kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_state);
            return true;
        }
        else {
            ROS_INFO("No IK solution found using seed");
            return false;
        }
    }

    /// @brief Calculate FK for a given joint state
    /// @param joint_state The joint state to calculate FK for
    /// @param pose The pose to store the FK solution
    /// @return True if FK was found, false otherwise
    bool calculateFK(const StateType &joint_state,
                     StateType &pose) {
        // set the joint state
        kinematic_state_->setJointGroupPositions(joint_model_group_, joint_state);
        // update
        planning_scene_monitor_->updateFrameTransforms();
        planning_scene_monitor_->updateSceneWithCurrentState();
        // get the tip link
        const std::string &tip_link = joint_model_group_->getLinkModelNames().back();
        // get the pose
        const Eigen::Isometry3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(tip_link);
        // push to pose as a vector of x, y, z, r, p, y
        pose.resize(6);
        pose[0] = end_effector_state.translation().x();
        pose[1] = end_effector_state.translation().y();
        pose[2] = end_effector_state.translation().z();
        ims::get_euler_zyx(end_effector_state.rotation(), pose[5], pose[4], pose[3]);
        ims::normalize_euler_zyx(pose[5], pose[4], pose[3]);
        return true;
    }

    /// @brief get the joint limits
    /// @param joint_limits The joint limits to store the limits
    void getJointLimits(std::vector<std::pair<double, double>> &joint_limits) const {  // TODO: check if this is the right way to do it
        const auto &bounds = joint_model_group_->getActiveJointModelsBounds();
        std::vector<double> min_vals, max_vals;
        for (int i = 0; i < num_joints_; i++) {
            const auto *jb = bounds[i];
            for (auto &b : *jb) {
                max_vals.push_back(b.max_position_);
                min_vals.push_back(b.min_position_);
            }
        }
        // resize the joint limits
        joint_limits.resize(num_joints_);
        // assert if the number of joint is not equal to the size of the bounds
        assert(num_joints_ == min_vals.size());
        for (int i = 0; i < num_joints_; i++) {
            joint_limits[i] = std::make_pair(min_vals[i], max_vals[i]);
        }
    }

    bool isStateWithinJointLimits(const StateType &state) const {
        for (int i = 0; i < num_joints_; i++) {
            if (state.at(i) < joint_limits_.at(i).first || state.at(i) > joint_limits_.at(i).second) {
                return false;
            }
        }
        return true;
    }

    inline int getNumCollisionChecks() const {
        return num_collision_checks_;
    }

    inline std::vector<std::string> getJointNames() const {
        return joint_names_;
    }

    void getCurrentState(StateType& state_val) const {
        kinematic_state_->copyJointGroupPositions(joint_model_group_, state_val);
    }

    void getCurrentRobotStateMoveIt(moveit::core::RobotStatePtr& robot_state_ptr) const {
        robot_state_ptr = kinematic_state_;
    }

    /// @brief Get the distance of a point to the robot.
    /// @param state The state of the robot.
    /// @param point The point to get the distance to, specified in the world frame (the planning frame of the scene).
    /// @param max_distance The maximum distance to check for. If a distance larger than this is found, the function returns this value.
    /// @param distance The distance to the robot to be populated by the function.
    void getDistanceToRobot(const StateType& state, const Eigen::Vector3d& point, double max_distance, double& distance) {
        // Set the state of the ego robot and reset the states of all others.
        moveit::core::RobotState &current_scene_state = planning_scene_->getCurrentStateNonConst();
        // Reset the scene.
        current_scene_state.setToDefaultValues();
        // Set the state of the ego robot.
        current_scene_state.setJointGroupPositions(group_name_, state);

        // Create a distance field around the point with a maximum distance of the radius.
        std::string robot_base_link_name = kinematic_state_->getJointModelGroup(group_name_)->getLinkModelNames().at(0);
        Eigen::Isometry3d robot_base_link_to_world = kinematic_state_->getFrameTransform(robot_base_link_name);
        double df_extent_x = max_distance * 2;
        double df_extent_y = max_distance * 2;
        double df_extent_z = max_distance * 2;
        double df_x_corner = point.x() - df_extent_x / 2.0;
        double df_y_corner = point.y() - df_extent_y / 2.0;
        double df_z_corner = point.z() - df_extent_z / 2.0;

        std::shared_ptr<distance_field::PropagationDistanceField> df = ims::getEmptyDistanceField(df_extent_x, df_extent_y, df_extent_z, 0.05, df_x_corner, df_y_corner, df_z_corner, max_distance);
        // Get the robot occupancy.
        moveit::core::RobotState current_state = planning_scene_->getCurrentState();

        assert (group_name_ee_ != "");

        std::vector<std::string> move_group_names {group_name_, group_name_ee_};
        ims::addRobotToDistanceField(df, current_state, move_group_names);
        std::chrono::steady_clock::time_point end_df = std::chrono::steady_clock::now();

        // Evaluate the distance of the point in the df.
        distance = df->getDistance(point.x(), point.y(), point.z());

        // If verbose then also visualize the distance field.
//        if (verbose_) {
//            ims::visualizeOccupancy(df, marker_pub, frame_id_);
//        }
    }

    /// @brief Get the distance of a point to the robot.
    /// @param state The state of the robot.
    /// @param point The point to get the distance to, specified in the world frame (the planning frame of the scene).
    /// @param max_distance The maximum distance to check for. If a distance larger than this is found, the function returns this value.
    /// @param distance The distance to the robot to be populated by the function.
    bool isRobotCollidingWithSphere(const StateType& state, const Eigen::Vector3d& point, double radius) {
        // Set the state of the ego robot and reset the states of all others.
        moveit::core::RobotState &current_scene_state = planning_scene_->getCurrentStateNonConst();
        // Reset the scene.
        current_scene_state.setToDefaultValues();
        // Set the state of the ego robot.
        current_scene_state.setJointGroupPositions(group_name_, state);

        // Add a sphere to the scene.
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = "constraint_sphere";
        collision_object.header.frame_id = frame_id_;
        shapes::ShapeMsg collision_object_shape_msg;
        auto *shape = new shapes::Sphere(radius);
        shapes::constructMsgFromShape(shape, collision_object_shape_msg);
        geometry_msgs::Pose collision_object_pose;
        collision_object_pose.position.x = point.x();
        collision_object_pose.position.y = point.y();
        collision_object_pose.position.z = point.z();

        collision_object_pose.orientation.x = 0.0;
        collision_object_pose.orientation.y = 0.0;
        collision_object_pose.orientation.z = 0.0;
        collision_object_pose.orientation.w = 1.0;

        collision_object.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(collision_object_shape_msg));
        collision_object.primitive_poses.push_back(collision_object_pose);
        collision_object.operation = moveit_msgs::CollisionObject::ADD;

        // Update the planning scene.
        // planning_scene_interface_->applyCollisionObject(collision_object);  // For visualization. TODO: remove.
        planning_scene_->processCollisionObjectMsg(collision_object);       // For collision checking.

        // Check if the sphere is in collision.
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = true;
        collision_request.max_contacts = 1;
        collision_request.max_contacts_per_pair = 1;
        collision_request.verbose = verbose_;
        collision_request.group_name = group_name_;

        planning_scene_->checkCollision(collision_request, collision_result);
        num_collision_checks_++;

        // Remove the sphere from the scene.
        collision_object.operation = collision_object.REMOVE;
        // planning_scene_interface_->applyCollisionObject(collision_object);  // For visualization. TODO: remove.
        planning_scene_->processCollisionObjectMsg(collision_object);       // For collision checking.

        return collision_result.collision;

    }

    void setEndEffectorMoveGroupName(const std::string& group_name_ee) {
        group_name_ee_ = group_name_ee;
    }
    std::string getEndEffectorLinkName() const {
        return joint_model_group_->getLinkModelNames().back();
    }

    std::shared_ptr<planning_scene::PlanningScene> getPlanningSceneMoveit() {
        return planning_scene_;
    }

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::string group_name_;
    std::string group_name_ee_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup *joint_model_group_;
    std::vector<std::string> joint_names_;
    std::vector<std::pair<double, double>> joint_limits_;
    size_t num_joints_;
    std::string frame_id_;

    /// @brief Ordered names of the move groups in the scene. Agent0 is at the zero index, agent1 at the first index, etc.
    std::vector<std::string> scene_move_group_names_;
};
}  // namespace ims

#endif  // MANIPULATION_PLANNING_MOVEITINTERFACE_HPP
