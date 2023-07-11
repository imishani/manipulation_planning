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
 * \file   manipulation_action_space.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/3/23
 */

#ifndef MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP
#define MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP

// include standard libraries
#include <iostream>
#include <utility>
#include <vector>

// include ROS libraries
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
// include tf to convert euler angles to quaternions
#include <tf/transform_datatypes.h>

// project includes
#include <manipulation_planning/common/moveit_interface.hpp>
#include <manipulation_planning/common/utils.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <search/common/action_space.hpp>


namespace ims
{

    struct manipulationType : ActionType
    {

        /// @brief Constructor
        /// @param[in] bfs_heuristic A pointer to a BFSHeuristic object. Default = nullptr
        explicit manipulationType(BFSHeuristic* bfs_heuristic = nullptr) : action_type_(ActionType::MOVE),
                             space_type_(spaceType::ConfigurationSpace),
                             prim_file_name_("../config/manip.mprim"),
                             max_action_(0.0){
            readMPfile();
        };

        /// @brief Constructor with motion primitives file given
        /// @param[in] mprimFile The path to the motion primitives file
        /// @param[in] bfs_heuristic A pointer to a BFSHeuristic object
        explicit manipulationType(std::string mprimFile,
                                  BFSHeuristic* bfs_heuristic = nullptr) :
                                  action_type_(ActionType::MOVE),
                                  space_type_(spaceType::ConfigurationSpace),
                                  prim_file_name_(std::move(mprimFile)),
                                  max_action_(0.0){
            readMPfile();
        };

        /// @brief Constructor with adaptive motion primitives given


        /// @brief Destructor
        ~manipulationType() override = default;

        /// @brief The type of the action
        enum class ActionType
        {
            MOVE,
            GRASP,
            RELEASE
        };

        enum class spaceType
        {
            ConfigurationSpace,
            WorkSpace
        };
        /// @{ getters and setters
        /// @brief Get the action type
        /// @return The action type
        ActionType getActionType() const
        {
            return action_type_;
        }

        /// @brief Set the action type
        /// @param ActionType The action type
        void setActionType(ActionType ActionType)
        {
            action_type_ = ActionType;
        }

        /// @brief Get the space type
        /// @return The space type
        spaceType getSpaceType() const
        {
            return space_type_;
        }

        /// @brief Set the space type
        /// @param spaceType The space type
        void setSpaceType(spaceType spaceType)
        {
            space_type_ = spaceType;
        }
        /// @}

        void Discretization(StateType &state_des) override
        {
            state_discretization_ = state_des;
        }

        void readMPfile()
        {
            std::ifstream file(prim_file_name_);
            std::string line;
            std::vector<std::vector<double>> mprim;
            int tot_prim {0}, dof{0}, num_short_prim{0};
            int i{0};
            while (std::getline(file, line))
            {
                if (i == 0)
                {
                    // First line being with: "Motion_Primitives(degrees): " and then three numbers. Make sure the line begins with the string and then get the numbers
                    std::string first_line = "Motion_Primitives(degrees): ";
                    // Check if the line begins with the string
                    if (line.find(first_line) != 0)
                    {
                        ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                    }
                    // Get the numbers
                    std::istringstream iss(line.substr(first_line.size()));
                    if (!(iss >> tot_prim >> dof >> num_short_prim))
                    {
                        ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                    }
                    i++;
                    continue;
                }
                std::istringstream iss(line);
                std::vector<double> line_;
                double num;
                while (iss >> num)
                {
                    line_.push_back(num);
                    if (abs(num * M_PI / 180.0) > max_action_)
                    {
                        max_action_ = abs(num * M_PI / 180.0);
                    }
                }
                // Check if short or long primitive (the last num_short_prim lines are short)
                if (i > tot_prim - num_short_prim)
                {
                    short_mprim_.push_back(line_);
                }
                else
                {
                    long_mprim_.push_back(line_);
                }
            }
        }

        /// @brief Get the possible actions
        /// @return A vector of all possible actions
        std::vector<Action> getActions() override
        {
            if (actions_.empty()) {
                switch (action_type_) {
                    case ActionType::MOVE:
                        switch (space_type_) {
                            case spaceType::ConfigurationSpace: {
                                std::vector<std::vector<double>> mprim;
                                mprim.insert(mprim.end(), short_mprim_.begin(), short_mprim_.end());
                                for (auto &action_ : mprim) {
                                    // convert from degrees to radians
                                    for (auto &num : action_) {
                                        num = num * M_PI / 180.0;
                                    }
                                    actions_.push_back(action_);
                                    // get the opposite action
                                    for (auto &num : action_) {
                                        num = -num;
                                    }
                                    actions_.push_back(action_);
                                }
                            }
                                break;
                            case spaceType::WorkSpace: {
                                std::vector<std::vector<double>> mprim;
                                mprim.insert(mprim.end(), short_mprim_.begin(), short_mprim_.end());
                                for (auto &action_ : mprim) {
                                    // make an inverted action
                                    Action inverted_action(action_.size());
                                    inverted_action[0] = -action_[0];
                                    inverted_action[1] = -action_[1];
                                    inverted_action[2] = -action_[2];
                                    inverted_action[3] = -action_[3];
                                    inverted_action[4] = -action_[4];
                                    inverted_action[5] = -action_[5];
                                    // convert from euler angles to quaternions
                                    tf::Quaternion q;
                                    q.setRPY(action_[3] * M_PI / 180.0,
                                             action_[4] * M_PI / 180,
                                             action_[5] * M_PI / 180);
                                    // check from antipodal quaternions
                                    int sign = 1;
                                    if (q.w() < 0) {
                                        sign = -1;
                                    }
                                    action_.resize(7);
                                    action_[3] = sign * q.x();
                                    action_[4] = sign * q.y();
                                    action_[5] = sign * q.z();
                                    action_[6] = sign * q.w();
                                    actions_.push_back(action_);
                                    // get the opposite action
                                    q.setRPY(inverted_action[3] * M_PI / 180,
                                             inverted_action[4] * M_PI / 180,
                                             inverted_action[5] * M_PI / 180);
                                    // check from antipodal quaternions
                                    sign = 1;
                                    if (q.w() < 0) {
                                        sign = -1;
                                    }
                                    inverted_action.resize(7);
                                    inverted_action[3] = sign * q.x();
                                    inverted_action[4] = sign * q.y();
                                    inverted_action[5] = sign * q.z();
                                    inverted_action[6] = sign * q.w();
                                    actions_.push_back(inverted_action);
                                }
                            }
                                break;
                        }
                        break;
                    case ActionType::GRASP:break;
                    case ActionType::RELEASE:break;
                }
                return actions_;
            } else {
                return actions_;
            }
        }

        /// @brief Get adaptive motion primitives
        /// @param start_dist The distance from the start
        /// @param goal_dist The distance from the goal
        /// @return A vector of actions
        std::vector<Action> getAdaptiveActions(double &start_dist, double &goal_dist) {
            actions_.clear();
            if (mprim_active_type_.long_dist.first) // insert long distance primitive and convert to radians
                for (auto &action_ : long_mprim_) {
                    std::vector<double> action, action_rev;
                    for (auto &num : action_) {
                        action.push_back(num * M_PI / 180.0);
                        action_rev.push_back(-num * M_PI / 180.0);
                    }
                    actions_.push_back(action);
                    actions_.push_back(action_rev);
                }
            if (mprim_active_type_.short_dist.first && (start_dist < mprim_active_type_.short_dist.second || goal_dist < mprim_active_type_.short_dist.second))
                for (auto &action_ : short_mprim_) {
                    std::vector<double> action, action_rev;
                    for (auto &num : action_) {
                        action.push_back(num * M_PI / 180.0);
                        action_rev.push_back(-num * M_PI / 180.0);
                    }
                    actions_.push_back(action);
                    actions_.push_back(action_rev);
                }
            if (mprim_active_type_.snap_xyz.first && goal_dist < mprim_active_type_.snap_xyz.second)
                actions_.push_back({0, 0, 0, 0, 0, 0, 0});  // TODO: Fix this to make it better designed
            if (mprim_active_type_.snap_rpy.first && goal_dist < mprim_active_type_.snap_rpy.second)
                actions_.push_back({0, 0, 0, 0, 0, 0, 0});  // TODO: Fix this to make it better designed
            if (mprim_active_type_.snap_xyzrpy.first && goal_dist < mprim_active_type_.snap_xyzrpy.second)
                {
                actions_.push_back({0, 0, 0, 0, 0, 0, 0});  // TODO: Fix this to make it better designed
                ROS_INFO_NAMED("adaptive_mprim", "snap xyzrpy");
                ROS_INFO_STREAM("goal_dist: " << goal_dist);
                }
            return actions_;
        }

        /// @brief Motion primitive active type: Used for adaptive motion primitives, given a few motion primitives,
        /// which one is active at a given time and it's threshold
        struct MotionPrimitiveActiveType
        {
            std::pair<bool, double> short_dist = std::make_pair(true, 0.1);
            std::pair<bool, double> long_dist = std::make_pair(true, 0.6);
            std::pair<bool, double> snap_xyz = std::make_pair(false, 0.2);
            std::pair<bool, double> snap_rpy = std::make_pair(false, 0.2);
            std::pair<bool, double> snap_xyzrpy = std::make_pair(true, 0.4);
        };

        ActionType action_type_;
        spaceType space_type_;
        std::string prim_file_name_;
        MotionPrimitiveActiveType mprim_active_type_;

        std::vector<Action> actions_;
        std::vector<Action> short_mprim_;
        std::vector<Action> long_mprim_;

        std::vector<bool> mprim_enabled_;
        std::vector<double> mprim_thresh_;
        double max_action_;
    };

    /// @class ManipulationActionSpace
    /// @brief A class that implements the ActionSpace for Moveit
    class ManipulationActionSpace : public ActionSpace
    {
    protected:
        /// @brief Manipulation type
        std::shared_ptr<manipulationType> mManipulationType;
        /// @brief Moveit interface
        std::shared_ptr<MoveitInterface> mMoveitInterface;
        /// @brief joint limits
        std::vector<std::pair<double, double>> mJointLimits;
        /// @brief Joint states seed
        //    std::vector<double> mJointStatesSeed {0, 0, 0, 0, 0, 0};
        /// @brief The BFS heuristic
        BFSHeuristic* bfs_heuristic_;

        // TODO: delete: temp
        int m_vis_id = 0;
        ros::NodeHandle m_nh;
        ros::Publisher m_vis_pub;


    public:

        /// @brief Constructor
        /// @param moveitInterface The moveit interface
        /// @param manipulationType The manipulation type
        ManipulationActionSpace(const MoveitInterface &env,
                                const manipulationType &actions_ptr,
                                BFSHeuristic* bfs_heuristic = nullptr) : ActionSpace(), bfs_heuristic_(bfs_heuristic)
        {
            mMoveitInterface = std::make_shared<MoveitInterface>(env);
            mManipulationType = std::make_shared<manipulationType>(actions_ptr);
            // get the joint limits
            mMoveitInterface->getJointLimits(mJointLimits);
            m_vis_pub = m_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

        }

        /// @brief Set the manipulation space type
        /// @param spaceType The manipulation type
        void setManipActionType(manipulationType::spaceType spaceType)
        {
            mManipulationType->setSpaceType(spaceType);
        }

        manipulationType::spaceType getManipActionType()
        {
            return mManipulationType->getSpaceType();
        }

        /// @brief Get current joint states
        /// @param joint_states The joint states
        void getCurrJointStates(StateType &joint_states)
        {
            auto joints = mMoveitInterface->mPlanningScene->getCurrentState();
            joints.copyJointGroupPositions(mMoveitInterface->mGroupName,
                                           joint_states);
        }

        /// @brief Get the workspace state
        /// @param ws_state The workspace state
        void getCurrWorkspaceState(StateType &ws_state)
        {
            // get the tip link name
            auto tip_link = mMoveitInterface->mPlanningScene->getRobotModel()->getJointModelGroup(mMoveitInterface->mGroupName)->getLinkModelNames().back();
            // get the end-effector pose
            auto ee_pose = mMoveitInterface->mPlanningScene->getCurrentState().getGlobalLinkTransform(tip_link);
            // get the euler angles
            ws_state.resize(6);
            ws_state[0] = ee_pose.translation().x();
            ws_state[1] = ee_pose.translation().y();
            ws_state[2] = ee_pose.translation().z();
            Eigen::Vector3d euler_angles = ee_pose.rotation().eulerAngles(2, 1, 0);
            ws_state[3] = euler_angles[2];
            ws_state[4] = euler_angles[1];
            ws_state[5] = euler_angles[0];
            normalize_euler_zyx(ws_state[5], ws_state[4], ws_state[3]);
            roundStateToDiscretization(ws_state, mManipulationType->state_discretization_);
        }

        bool isStateValid(const StateType &state_val) override
        {
            // check if the state is valid
            switch (mManipulationType->getSpaceType())
            {
            case manipulationType::spaceType::ConfigurationSpace:
                return mMoveitInterface->isStateValid(state_val);
            case manipulationType::spaceType::WorkSpace:
                // check if state exists with IK solution already
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0];
                pose.position.y = state_val[1];
                pose.position.z = state_val[2];
                // Euler angles to quaternion
                Eigen::Quaterniond q;
                from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                StateType joint_state;
                bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                if (!succ)
                {
                    return false;
                }
                else
                {
                    return mMoveitInterface->isStateValid(joint_state);
                }
            }
            return false;
        }

        /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
        /// @param state_val The state to check
        /// @param joint_state The ik solution
        /// @return True if the state is valid, false otherwise
        bool isStateValid(const StateType &state_val,
                          StateType &joint_state)
        {
            switch (mManipulationType->getSpaceType())
            {
            case manipulationType::spaceType::ConfigurationSpace:
                return mMoveitInterface->isStateValid(state_val);
            case manipulationType::spaceType::WorkSpace:
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0];
                pose.position.y = state_val[1];
                pose.position.z = state_val[2];
                // Euler angles to quaternion
                Eigen::Quaterniond q;
                from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                if (!succ)
                {
                    ROS_INFO("IK failed");
                    return false;
                }
                else
                {
                    return mMoveitInterface->isStateValid(joint_state);
                }
            }
            return false;
        }

        bool isStateValid(const StateType &state_val,
                          const StateType &seed,
                          StateType &joint_state)
        {
            // check if the state is valid
            switch (mManipulationType->getSpaceType())
            {
            case manipulationType::spaceType::ConfigurationSpace:
                return mMoveitInterface->isStateValid(state_val);
            case manipulationType::spaceType::WorkSpace:
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0];
                pose.position.y = state_val[1];
                pose.position.z = state_val[2];
                // Euler angles to quaternion
                Eigen::Quaterniond q;
                from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                joint_state.resize(mMoveitInterface->num_joints);
                bool succ = mMoveitInterface->calculateIK(pose, seed, joint_state);
                normalizeAngles(joint_state);
                if (!succ)
                {
                    return false;
                }
                else
                {
                    return mMoveitInterface->isStateValid(joint_state);
                }
            }
            return false;
        }

        /// @brief Interpolate path between two states
        /// @param start The start state
        /// @param end The end state
        /// @param resolution The resolution of the path (default: 0.005 rad)
        /// @return The interpolated path
        static PathType interpolatePath(const StateType &start, const StateType &end,
                                        const double resolution = 0.02)
        {
            // TODO: Currently only works for configuration space
            assert(start.size() == end.size());
            PathType path;
            // get the maximum distance between the two states
            double max_distance{0.0};
            for (int i{0}; i < start.size(); i++)
            {
                double distance = std::abs(start[i] - end[i]);
                if (distance > max_distance)
                {
                    max_distance = distance;
                }
            }
            // calculate the number of steps
            int steps = std::ceil(max_distance / resolution);
            // interpolate the path
            for (int i{0}; i < steps; i++)
            {
                StateType state;
                for (int j{0}; j < start.size(); j++)
                {
                    state.push_back(start[j] + (end[j] - start[j]) * i / steps);
                }
                path.push_back(state);
            }
            return path;
        }

        bool isStateToStateValid(const StateType &start, const StateType &end)
        {
            PathType path = interpolatePath(start, end);
            return isPathValid(path);
        }

        bool isPathValid(const PathType &path) override
        {
            switch (mManipulationType->getSpaceType())
            {
            case manipulationType::spaceType::ConfigurationSpace:
                return mMoveitInterface->isPathValid(path);
            case manipulationType::spaceType::WorkSpace:
                PathType poses;
                for (auto &state : path)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = state[0];
                    pose.position.y = state[1];
                    pose.position.z = state[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state[5], state[4], state[3], q);
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();
                    StateType joint_state;
                    bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                    if (!succ)
                    {
                        return false;
                    }
                    else
                    {
                        poses.push_back(joint_state);
                    }
                }
                return mMoveitInterface->isPathValid(poses);
            }
            return false;
        }

        virtual bool getSuccessorsWs(int curr_state_ind,
                                     std::vector<int>& successors,
                                     std::vector<double> &costs)
        {
            // get the current state
            auto curr_state = this->getRobotState(curr_state_ind);
            auto curr_state_val = curr_state->state;
            // get the actions
            auto actions = mManipulationType->getActions();
            // convert to quaternion
            Eigen::Quaterniond q_curr;
            from_euler_zyx(curr_state_val[5], curr_state_val[4], curr_state_val[3], q_curr);
            // get the successors
            StateType new_state_val;
            for (auto action : actions)
            {
                new_state_val.clear();
                // create a new state in the length of the current state
                new_state_val.resize(curr_state_val.size());
                // increment the xyz coordinates
                for (int i{0}; i < 3; i++)
                {
                    new_state_val[i] = curr_state_val[i] + action[i];
                }

                Eigen::Quaterniond q_action{action[6], action[3], action[4], action[5]};
                auto q_new = q_curr * q_action;

                // convert the quaternion to euler angles
                get_euler_zyx(q_new, new_state_val[5], new_state_val[4], new_state_val[3]);
                normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
                // discretize
                roundStateToDiscretization(new_state_val, mManipulationType->state_discretization_);

                //            if (isStateToStateValid(curr_state_val, new_state_val)) {
                bool succ;
                StateType mapped_state;
                if (curr_state->state_mapped.empty())
                {
                    //                    ROS_INFO("No mapped state, using IK without seed");
                    succ = isStateValid(new_state_val,
                                        mapped_state);
                }
                else
                    succ = isStateValid(new_state_val,
                                        curr_state->state_mapped,
                                        mapped_state);
                if (succ)
                {
                    // create a new state
                    int next_state_ind = getOrCreateRobotState(new_state_val);
                    auto new_state = this->getRobotState(next_state_ind);
                    new_state->state_mapped = mapped_state;
                    // add the state to the successors
                    successors.push_back(next_state_ind);
                    // add the cost
                    double cost{0};
                    for (int i{0}; i < 3; i++)
                    {
                        cost += action[i] * action[i];
                    }
                    // add the cost of the rotation which is quaternion
                    double r, p, y;
                    get_euler_zyx(q_action, y, p, r);
                    cost += r * r + p * p + y * y;
                    costs.push_back(cost);
                }
            }
            return true;
        }

        /// @brief Visualize a state point in rviz for debugging
        /// @param state_id The state id
        /// @param type The type of state (greedy, attractor, etc)
        void VisualizePoint(double x, double y, double z) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = mMoveitInterface->mPlanningScene->getPlanningFrame();
            marker.header.stamp = ros::Time();
            marker.ns = "graph";
            marker.id = m_vis_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = z;
            marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.02; marker.scale.y = 0.02; marker.scale.z = 0.02;
            // green
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
            marker.color.a = 0.5;
            // visualize
            m_vis_pub.publish(marker);
            m_vis_id++;
        }

        virtual bool getSuccessorsCs(int curr_state_ind,
                                     std::vector<int>& successors,
                                     std::vector<double> &costs)
        {
            // get the current state
            auto curr_state = this->getRobotState(curr_state_ind);
            auto curr_state_val = curr_state->state;
            std::vector<Action> actions;
            if (bfs_heuristic_ == nullptr){
                 actions = mManipulationType->getActions();
            }
            // get the actions
            else {
                if (curr_state->state_mapped.empty()){
                    mMoveitInterface->calculateFK(curr_state_val, curr_state->state_mapped);
                    this->VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
                }
                auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                                       curr_state->state_mapped.at(1),
                                                                       curr_state->state_mapped.at(2));
                auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                         curr_state->state_mapped.at(1),
                                                                         curr_state->state_mapped.at(2));
                actions = mManipulationType->getAdaptiveActions(start_dist, goal_dist);
            }
            // get the successors
            for (auto &action : actions)
            {
                // create a new state in the length of the current state
                StateType new_state_val{};
                new_state_val.resize(curr_state_val.size());
                std::fill(new_state_val.begin(), new_state_val.end(), 0.0);

                // check if actions are all zero, if so, replace them with the goal state
                if (std::all_of(action.begin(), action.end(), [](double i){return i == 0;})){
                    new_state_val = bfs_heuristic_->mGoal;
                } else{
                    for (int i{0}; i < curr_state_val.size(); i++) {
                        new_state_val[i] = curr_state_val[i] + action[i];
                    }
                }
                // normalize the angles
                normalizeAngles(new_state_val, mJointLimits);
                // discretize the state
                roundStateToDiscretization(new_state_val, mManipulationType->state_discretization_);
                // check if the state went through discontinuity
                bool discontinuity{false};
                // check for maximum absolute action
                for (int i{0}; i < curr_state_val.size(); i++)
                {
                    if (fabs(new_state_val[i] - curr_state_val[i]) > 20.0 * mManipulationType->max_action_)
                    {
                        discontinuity = true;
                        break;
                    }
                }

                // if (isStateToStateValid(curr_state_val, new_state_val)) {
                if (isStateToStateValid(curr_state_val, new_state_val) && !discontinuity)
                {
                    // create a new state
                    int next_state_ind = getOrCreateRobotState(new_state_val);
                    // add the state to the successors
                    successors.push_back(next_state_ind);
                    // add the cost
                    // TODO: change this to the real cost
//                    double norm = 0;
//                    for (double i : action)
//                    {
//                        norm += i * i;
//                    }
//                    costs.push_back(sqrt(norm));
                    costs.push_back(1000);
                }
            }
            return true;
        }

        bool getSuccessors(int curr_state_ind,
                           std::vector<int> &successors,
                           std::vector<double> &costs) override
        {
            if (mManipulationType->getSpaceType() == manipulationType::spaceType::ConfigurationSpace)
            {
                return getSuccessorsCs(curr_state_ind, successors, costs);
            }
            else
            {
                return getSuccessorsWs(curr_state_ind, successors, costs);
            }
        }
    };
}

#endif // MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP
