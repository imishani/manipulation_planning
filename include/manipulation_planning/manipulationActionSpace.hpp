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
 * \file   actionSpace.hpp
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
#include <manipulation_planning/common/MoveitInterface.hpp>
#include <manipulation_planning/common/utils.hpp>

namespace ims {

    struct manipulationType : actionType {

        /// @brief Constructor
        manipulationType() : mActionType(ActionType::MOVE),
                             mSpaceType(spaceType::ConfigurationSpace),
                             mPrimFileName("../config/manip.mprim") {
            // make mActions to point to nullptr
            mActions = nullptr;
        };

        explicit manipulationType(std::string mprimFile) : mActionType(ActionType::MOVE),
                                                           mSpaceType(spaceType::ConfigurationSpace),
                                                           mPrimFileName(std::move(mprimFile)) {};

        /// @brief Destructor
        ~manipulationType() override = default;

        /// @brief The type of the action
        enum class ActionType {
            MOVE,
            GRASP,
            RELEASE
        };

        enum class spaceType {
            ConfigurationSpace,
            WorkSpace
        };
        /// @{ getters and setters
        /// @brief Get the action type
        /// @return The action type
        ActionType getActionType() const {
            return mActionType;
        }

        /// @brief Set the action type
        /// @param actionType The action type
        void setActionType(ActionType actionType) {
            mActionType = actionType;
        }

        /// @brief Get the space type
        /// @return The space type
        spaceType getSpaceType() const {
            return mSpaceType;
        }

        /// @brief Set the space type
        /// @param spaceType The space type
        void setSpaceType(spaceType spaceType) {
            mSpaceType = spaceType;
        }
        /// @}

        void Discretization(stateType& state_des) override {
            mStateDiscretization = state_des;
        }


        static std::vector<action> readMPfile(const std::string& file_name) {
            std::ifstream file(file_name);
            std::string line;
            std::vector<std::vector<double>> mprim;
            int i {0};
            while (std::getline(file, line)) {
                if (i == 0) {
                    i++;
                    continue;
                }
                std::istringstream iss(line);
                std::vector<double> line_;
                double num;
                while (iss >> num) {
                    line_.push_back(num);
                }
                mprim.push_back(line_);
            }
            return mprim;
        }

        /// @brief Get the possible actions
        /// @return A vector of all possible actions
        std::vector<action> getActions() override {
            if (mActions == nullptr) {
                mActions = std::make_shared<std::vector<action>>();
                switch (mActionType) {
                    case ActionType::MOVE:
                        switch (mSpaceType) {
                            case spaceType::ConfigurationSpace:{
                                auto mprim = readMPfile(mPrimFileName);
                                for (auto& action_ : mprim) {
                                    // convert from degrees to radians
                                    for (auto& num : action_) {
                                        num = num * M_PI / 180;
                                    }
                                    mActions->push_back(action_);
                                    // get the opposite action
                                    for (auto& num : action_) {
                                        num = -num;
                                    }
                                    mActions->push_back(action_);
                                }
                            }
                                break;
                            case spaceType::WorkSpace:{
                                auto mprim = readMPfile(mPrimFileName);
                                for (auto& action_ : mprim) {
                                    // make an inverted action
                                    action inverted_action(action_.size());
                                    inverted_action[0] = -action_[0]; inverted_action[1] = -action_[1]; inverted_action[2] = -action_[2];
                                    inverted_action[3] = -action_[3]; inverted_action[4] = -action_[4]; inverted_action[5] = -action_[5];
                                    // convert from euler angles to quaternions
                                    tf::Quaternion q;
                                    q.setRPY(action_[3]*M_PI / 180, action_[4]*M_PI / 180, action_[5]*M_PI / 180);
                                    // check from antipodal quaternions
                                    int sign = 1;
                                    if (q.w() < 0) {
                                        sign = -1;
                                    }
                                    action_.resize(7);
                                    action_[3] = sign*q.x();
                                    action_[4] = sign*q.y();
                                    action_[5] = sign*q.z();
                                    action_[6] = sign*q.w();
                                    mActions->push_back(action_);
                                    // get the opposite action
                                    q.setRPY(inverted_action[3]*M_PI / 180, inverted_action[4]*M_PI / 180, inverted_action[5]*M_PI / 180);
                                    // check from antipodal quaternions
                                    sign = 1;
                                    if (q.w() < 0) {
                                        sign = -1;
                                    }
                                    inverted_action.resize(7);
                                    inverted_action[3] = sign*q.x();
                                    inverted_action[4] = sign*q.y();
                                    inverted_action[5] = sign*q.z();
                                    inverted_action[6] = sign*q.w();
                                    mActions->push_back(inverted_action);
                                }
                            }
                                break;
                        }
                        break;
                    case ActionType::GRASP:
                        break;
                    case ActionType::RELEASE:
                        break;
                }
                return *mActions;
            }
            else {
                return *mActions;
            }
        }

        ActionType mActionType;
        spaceType mSpaceType;
        std::string mPrimFileName;
        std::shared_ptr<std::vector<action>> mActions;
    };


/// @class ManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
    class ManipulationActionSpace : public actionSpace {
    protected:
        /// @brief Manipulation type
        std::shared_ptr<manipulationType> mManipulationType;
        /// @brief Moveit interface
        std::shared_ptr<MoveitInterface> mMoveitInterface;
        /// @brief joint limits
        std::vector<std::pair<double, double>> mJointLimits;
        /// @brief Joint states seed
//    std::vector<double> mJointStatesSeed {0, 0, 0, 0, 0, 0};

    public:
        /// @brief Constructor
        /// @param moveitInterface The moveit interface
        /// @param manipulationType The manipulation type
        ManipulationActionSpace(const MoveitInterface& env,
                                const manipulationType& actions_ptr) : actionSpace(){
            mMoveitInterface = std::make_shared<MoveitInterface>(env);
            mManipulationType = std::make_shared<manipulationType>(actions_ptr);
            // get the joint limits
            mMoveitInterface->getJointLimits(mJointLimits);
        }

        /// @brief Get current joint states
        /// @param joint_states The joint states
        void getJointStates(stateType& joint_states) {
            auto joints = mMoveitInterface->mPlanningScene->getCurrentState();
            joints.copyJointGroupPositions(mMoveitInterface->mGroupName,
                                           joint_states);
        }

        /// @brief Get the workspace state
        /// @param ws_state The workspace state
        void getWorkspaceState(stateType& ws_state){
            // get the tip link name
            auto tip_link = mMoveitInterface->mPlanningScene->getRobotModel()->getJointModelGroup(mMoveitInterface->mGroupName)->getLinkModelNames().back();
            // get the end-effector pose
            auto ee_pose = mMoveitInterface->mPlanningScene->getCurrentState().getGlobalLinkTransform(tip_link);
            // get the euler angles
            ws_state.resize(6);
            ws_state[0] = ee_pose.translation().x(); ws_state[1] = ee_pose.translation().y(); ws_state[2] = ee_pose.translation().z();
            Eigen::Vector3d euler_angles = ee_pose.rotation().eulerAngles(2, 1, 0);
            ws_state[3] = euler_angles[2]; ws_state[4] = euler_angles[1]; ws_state[5] = euler_angles[0];
        }


        bool isStateValid(const stateType& state_val) override {
            // check if the state is valid
            switch (mManipulationType->getSpaceType()) {
                case manipulationType::spaceType::ConfigurationSpace:
                    return mMoveitInterface->isStateValid(state_val);
                case manipulationType::spaceType::WorkSpace:
                    geometry_msgs::Pose pose;
                    pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y();
                    pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    stateType joint_state;
                    bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                    if (!succ) {
                        return false;
                    }
                    else {
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
        static pathType interpolatePath(const stateType& start, const stateType& end,
                                        const double resolution=0.005) {
            // TODO: Currently only works for configuration space
            assert(start.size() == end.size());
            pathType path;
            // get the maximum distance between the two states
            double max_distance {0.0};
            for (int i {0} ; i < start.size() ; i++) {
                double distance = std::abs(start[i] - end[i]);
                if (distance > max_distance) {
                    max_distance = distance;
                }
            }
            // calculate the number of steps
            int steps = std::ceil(max_distance / resolution);
            // interpolate the path
            for (int i {0} ; i < steps ; i++) {
                stateType state;
                for (int j {0} ; j < start.size() ; j++) {
                    state.push_back(start[j] + (end[j] - start[j]) * i / steps);
                }
                path.push_back(state);
            }
            return path;
        }

        bool isStateToStateValid(const stateType& start, const stateType& end){
            pathType path = interpolatePath(start, end);
            return isPathValid(path);
        }

        bool isPathValid(const pathType& path) override {
            switch (mManipulationType->getSpaceType()) {
                case manipulationType::spaceType::ConfigurationSpace:
                    return mMoveitInterface->isPathValid(path);
                case manipulationType::spaceType::WorkSpace:
                    pathType poses;
                    for (auto& state : path) {
                        geometry_msgs::Pose pose;
                        pose.position.x = state[0]; pose.position.y = state[1]; pose.position.z = state[2];
                        // Euler angles to quaternion
                        Eigen::Quaterniond q;
                        from_euler_zyx(state[5], state[4], state[3], q);
                        pose.orientation.x = q.x(); pose.orientation.y = q.y();
                        pose.orientation.z = q.z(); pose.orientation.w = q.w();
                        stateType joint_state;
                        bool succ = mMoveitInterface->calculateIK(pose, joint_state);
                        if (!succ) {
                            return false;
                        }
                        else {
                            poses.push_back(joint_state);
                        }
                    }
                    return mMoveitInterface->isPathValid(poses);
            }
            return false;
        }

        bool getSuccessorsWs(int curr_state_ind,
                             std::vector<state*>& successors,
                             std::vector<double>& costs) {
            // get the current state
            auto curr_state = this->getState(curr_state_ind);
            auto curr_state_val = curr_state->getState();
            // get the actions
            auto actions = mManipulationType->getActions();
            // convert to quaternion
            Eigen::Quaterniond q_curr;
            from_euler_zyx(curr_state_val[5], curr_state_val[4], curr_state_val[3], q_curr);
            // get the successors
            stateType new_state_val;
            for (auto action : actions) {
                new_state_val.clear();
                // create a new state in the length of the current state
                new_state_val.resize(curr_state_val.size());
                // increment the xyz coordinates
                for (int i {0} ; i < 3 ; i++) {
                    new_state_val[i] = curr_state_val[i] + action[i];
                }

                Eigen::Quaterniond q_action {action[6], action[3], action[4], action[5]};
                auto q_new = q_curr * q_action;

                // convert the quaternion to euler angles
                get_euler_zyx(q_new, new_state_val[5], new_state_val[4], new_state_val[3]);
                normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
                // discretize
                roundStateToDiscretization(new_state_val, mManipulationType->mStateDiscretization);

//            if (isStateToStateValid(curr_state_val, new_state_val)) {
                if (isStateValid(new_state_val)) {
                    // create a new state
                    int next_state_ind = getOrCreateState(new_state_val);
                    auto new_state = this->getState(next_state_ind);
                    // add the state to the successors
                    successors.push_back(new_state);
                    // add the cost
                    double cost {0};
                    for (int i {0} ; i < 3 ; i++) {
                        cost += action[i]*action[i];
                    }
                    // add the cost of the rotation which is quaternion
                    double r, p, y;
                    get_euler_zyx(q_action, y, p, r);
                    cost += r*r + p*p + y*y;
                    costs.push_back(cost);
                }
            }
            return true;
        }

        bool getSuccessorsCs(int curr_state_ind,
                             std::vector<state*>& successors,
                             std::vector<double>& costs) {
            // get the current state
            auto curr_state = this->getState(curr_state_ind);
            auto curr_state_val = curr_state->getState();
            // get the actions
            auto actions = mManipulationType->getActions();
            // get the successors
            for (auto action : actions) {
                // create a new state in the length of the current state
                stateType new_state_val {};
                new_state_val.resize(curr_state_val.size());
                std::fill(new_state_val.begin(), new_state_val.end(), 0.0);

                for (int i {0} ; i < curr_state_val.size() ; i++) {
                    new_state_val[i] = curr_state_val[i] + action[i];
                }
                // normalize the angles
                normalizeAngles(new_state_val);
                // discretize the state
                roundStateToDiscretization(new_state_val, mManipulationType->mStateDiscretization);

                // if (isStateToStateValid(curr_state_val, new_state_val)) {
                if (isStateValid(new_state_val)) {
                    // create a new state
                    int next_state_ind = getOrCreateState(new_state_val);
                    auto new_state = this->getState(next_state_ind);
                    // add the state to the successors
                    successors.push_back(new_state);
                    // add the cost
                    // TODO: change this to the real cost
                    double norm = 0;
                    for (double i : action) {
                        norm += i * i;
                    }
                    costs.push_back(sqrt(norm));
                }
            }
            return true;
        }

        bool getSuccessors(int curr_state_ind,
                           std::vector<state*>& successors,
                           std::vector<double>& costs) override {
            if (mManipulationType->getSpaceType() == manipulationType::spaceType::ConfigurationSpace) {
                return getSuccessorsCs(curr_state_ind, successors, costs);
            } else {
                return getSuccessorsWs(curr_state_ind, successors, costs);
            }
        }
    };
}


#endif //MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP
