//
// Created by itamar on 4/3/23.
//

#ifndef MANIPULATION_PLANNING_MOVEITINTERFACE_HPP
#define MANIPULATION_PLANNING_MOVEITINTERFACE_HPP

// include standard libraries
#include <iostream>
#include <vector>

// include ROS libraries
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// project includes
#include <common/types.hpp>
#include <common/state.hpp>
#include <common/SceneInterface.hpp>
#include <common/actionSpace.hpp>


/// @class MoveitInterface
/// @brief A class that implements the SceneInterface for Moveit
/// @details This class implements the SceneInterface for Moveit. It is used to get the current state of the robot and to
/// get all information needed about the scene (e.g. obstacles, robot, etc.)
class MoveitInterface : public ims::SceneInterface {
public:
    /// @brief Constructor
    MoveitInterface(const std::string& group_name="manipulator") {
        // planning scene monitor
        mPlanningSceneMonitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        mPlanningSceneMonitor->startSceneMonitor();
        mPlanningSceneMonitor->startStateMonitor();
        mPlanningSceneMonitor->startWorldGeometryMonitor();
        mPlanningScene = mPlanningSceneMonitor->getPlanningScene();
        mGroupName = group_name;

        m_kinematic_state = std::make_shared<moveit::core::RobotState>(mPlanningScene->getCurrentState());
        // joint model group
        joint_model_group = m_kinematic_state->getJointModelGroup(mGroupName);
        // get the joint names
        joint_names = joint_model_group->getVariableNames();
        // get the number of joints
        num_joints = joint_names.size();
    };

    /// @brief Destructor
    ~MoveitInterface() override = default;

    /// @brief check if the state is valid
    /// @param state The state to check
    /// @return True if the state is valid, false otherwise
    bool isStateValid(const stateType &state) {
        moveit_msgs::RobotState robotState;
        robotState.joint_state.position = state;
        return mPlanningScene->isStateValid(robotState, mGroupName);
    }

    /// @brief Check if a path is valid
    /// @param path The path to check
    /// @return True if the path is valid, false otherwise
    bool isPathValid(const pathType &path) {
        // TODO: Is this ok or should i use isPathValid instead?
        return std::all_of(path.begin(), path.end(), [this](const stateType& state_val){return isStateValid(state_val);});
    }

    /// @brief Calculate IK for a given pose
    /// @param pose The pose to calculate IK for
    /// @param joint_state The joint state to store the IK solution
    /// @return True if IK was found, false otherwise
    bool calculateIK(const geometry_msgs::Pose &pose, stateType &joint_state) {
        // resize the joint state
        joint_state.resize(num_joints);
        // set the pose
        if (m_kinematic_state->setFromIK(joint_model_group, pose)) {
            // get the joint values
            m_kinematic_state->copyJointGroupPositions(joint_model_group, joint_state);
            return true;
        } else {
            ROS_INFO("No IK solution found");
            return false;
        }
    }

    /// @brief Calculate IK for a given pose and a given seed
    /// @param pose The pose to calculate IK for
    /// @param seed The seed to use for the IK
    /// @param joint_state The joint state to store the IK solution
    /// @return True if IK was found, false otherwise
    bool calculateIK(const geometry_msgs::Pose &pose, const stateType &seed, stateType &joint_state) {
        // resize the joint state
        joint_state.resize(num_joints);
        // set the pose
        // TODO: check if its really seeding
        m_kinematic_state->setJointGroupPositions(joint_model_group, seed);
        if (m_kinematic_state->setFromIK(joint_model_group, pose)) {
            // get the joint values
            m_kinematic_state->copyJointGroupPositions(joint_model_group, joint_state);
            return true;
        } else {
            ROS_INFO("No IK solution found");
            return false;
        }
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
        // assert if the number of joint is not equal ti the size of the bounds
        assert(num_joints == min_vals.size());
        for(int i = 0; i < num_joints;i++)
        {
            joint_limits[i] = std::make_pair(min_vals[i], max_vals[i]);
        }
    }

    planning_scene_monitor::PlanningSceneMonitorPtr mPlanningSceneMonitor;
    std::shared_ptr<planning_scene::PlanningScene> mPlanningScene;
    std::string mGroupName;
    moveit::core::RobotStatePtr m_kinematic_state;
    const moveit::core::JointModelGroup *joint_model_group;
    std::vector<std::string> joint_names;
    int num_joints;

};

#endif //MANIPULATION_PLANNING_MOVEITINTERFACE_HPP
