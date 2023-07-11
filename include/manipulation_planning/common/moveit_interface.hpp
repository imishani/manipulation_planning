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
 * \file   moveit_interface.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/3/23
*/

#ifndef MANIPULATION_PLANNING_MOVEITINTERFACE_HPP
#define MANIPULATION_PLANNING_MOVEITINTERFACE_HPP

// include standard libraries
#include <iostream>
#include <vector>

// include ROS libraries
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <eigen_conversions/eigen_msg.h>

// project includes
#include <search/common/types.hpp>
#include <search/common/scene_interface.hpp>
#include <manipulation_planning/common/utils.hpp>


namespace ims{

    /// @class MoveitInterface
    /// @brief A class that implements the SceneInterface for Moveit
    /// @details This class implements the SceneInterface for Moveit. It is used to get the current state of the robot and to
    /// get all information needed about the scene (e.g. obstacles, robot, etc.)
    class MoveitInterface : public SceneInterface {
    public:
        /// @brief Constructor
        explicit MoveitInterface(const std::string& group_name) {
            // planning scene monitor
            mPlanningSceneMonitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
            mPlanningSceneMonitor->startSceneMonitor();
            mPlanningSceneMonitor->startStateMonitor();
            mPlanningSceneMonitor->startWorldGeometryMonitor();
            mPlanningSceneMonitor->requestPlanningSceneState();
            ros::Duration(1.0).sleep();
            mPlanningScene = mPlanningSceneMonitor->getPlanningScene();
            mGroupName = group_name;

            m_kinematic_state = std::make_shared<moveit::core::RobotState>(mPlanningScene->getCurrentState());
            // joint model group
            joint_model_group = m_kinematic_state->getJointModelGroup(mGroupName);
            // get the joint names
            joint_names = joint_model_group->getVariableNames();
            // get the number of joints
            num_joints = joint_names.size();

            auto object_names = mPlanningScene->getWorld()->getObjectIds();

            for (auto& obj : object_names) {
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
            robotState.joint_state.name = joint_names;
            // check if the state is valid
            return mPlanningScene->isStateValid(robotState, mGroupName);
        }

        /// @brief Check if a path is valid
        /// @param path The path to check
        /// @return True if the path is valid, false otherwise
        bool isPathValid(const PathType &path) {
            // TODO: Is this ok or should i use isPathValid instead?
            return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
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
            joint_state.resize(num_joints);
            // set joint model group as random, only the relevant kinematic group
            m_kinematic_state->setToRandomPositions(joint_model_group);
            // update
            mPlanningSceneMonitor->updateFrameTransforms();
            mPlanningSceneMonitor->updateSceneWithCurrentState(); // TODO: Is this needed?
            // set the pose
            if (m_kinematic_state->setFromIK(joint_model_group, pose, timeout)) {
                // get the joint values
                m_kinematic_state->copyJointGroupPositions(joint_model_group, joint_state);
                return true;
            } else {
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
                         double consistency_limit = 0.3,
                         double timeout = 0.05) {
            // resize the joint state
            joint_state.resize(num_joints);
            // set the pose
            m_kinematic_state->setJointGroupPositions(joint_model_group,seed);
            // update
            mPlanningSceneMonitor->updateFrameTransforms();
            mPlanningSceneMonitor->updateSceneWithCurrentState();
            // add a consistency_limits of 0.1 to the seed
            std::vector<double> consistency_limits(num_joints, consistency_limit);
            // get the tip link
            const std::string &tip_link = joint_model_group->getLinkModelNames().back();
            Eigen::Isometry3d pose_eigen;
            tf::poseMsgToEigen(pose, pose_eigen);

            if (m_kinematic_state->setFromIK(joint_model_group, pose_eigen,
                                             tip_link, consistency_limits,
                                             timeout)) {
                m_kinematic_state->copyJointGroupPositions(joint_model_group, joint_state);
                return true;
            } else {
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
            m_kinematic_state->setJointGroupPositions(joint_model_group, joint_state);
            // update
            mPlanningSceneMonitor->updateFrameTransforms();
            mPlanningSceneMonitor->updateSceneWithCurrentState();
            // get the tip link
            const std::string &tip_link = joint_model_group->getLinkModelNames().back();
            // get the pose
            const Eigen::Isometry3d &end_effector_state = m_kinematic_state->getGlobalLinkTransform(tip_link);
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
        void getJointLimits(std::vector<std::pair<double, double>> &joint_limits) const { // TODO: check if this is the right way to do it
            const auto &bounds = joint_model_group->getActiveJointModelsBounds();
            std::vector<double> min_vals, max_vals;
            for(int i = 0; i < num_joints;i++)
            {
                const auto *jb = bounds[i];
                for(auto& b: *jb)
                {
                    max_vals.push_back(b.max_position_);
                    min_vals.push_back(b.min_position_);
                }
            }
            // resize the joint limits
            joint_limits.resize(num_joints);
            // assert if the number of joint is not equal to the size of the bounds
            assert(num_joints == min_vals.size());
            for(int i = 0; i < num_joints;i++)
            {
                joint_limits[i] = std::make_pair(min_vals[i], max_vals[i]);
            }
        }

        planning_scene_monitor::PlanningSceneMonitorPtr mPlanningSceneMonitor;
        std::shared_ptr<planning_scene::PlanningScene> mPlanningScene;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> mPlanningSceneInterface;
        std::string mGroupName;
        moveit::core::RobotStatePtr m_kinematic_state;
        const moveit::core::JointModelGroup *joint_model_group;
        std::vector<std::string> joint_names;
        size_t num_joints;

    };
}

#endif //MANIPULATION_PLANNING_MOVEITINTERFACE_HPP
