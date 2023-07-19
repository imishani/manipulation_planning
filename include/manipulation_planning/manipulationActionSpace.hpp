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

#include <memory>
#include <map>
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
        ros::NodeHandle m_pnh;
        ros::NodeHandle m_nh;
        ros::Publisher m_vis_pub;
        int m_vis_id = 0; // The next id to publish to
        collision_detection::AllowedCollisionMatrix internal_acm;

        // Create the move group pointer
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_group;


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
            m_pnh = ros::NodeHandle(env.mGroupName);
            m_vis_pub = m_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
            m_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(mMoveitInterface->mGroupName);
        }

        /// @brief Set the manipulation space type
        /// @param spaceType The manipulation type
        void setManipActionType(manipulationType::spaceType spaceType) {
            mManipulationType->setSpaceType(spaceType);
        }

        manipulationType::spaceType getManipActionType() {
            return mManipulationType->getSpaceType();
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
            normalize_euler_zyx(ws_state[5], ws_state[4], ws_state[3]);
            roundStateToDiscretization(ws_state, mManipulationType->mStateDiscretization);
        }


        bool isStateValid(const stateType& state_val) override {
            // check if the state is valid
            switch (mManipulationType->getSpaceType()) {
                case manipulationType::spaceType::ConfigurationSpace:
                    return mMoveitInterface->isStateValid(state_val);
                case manipulationType::spaceType::WorkSpace:
                    // check if state exists with IK solution already
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

        /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
        /// @param state_val The state to check
        /// @param joint_state The ik solution
        /// @return True if the state is valid, false otherwise
        bool isStateValid(const stateType& state_val,
                          stateType& joint_state) {
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
                    bool succ = mMoveitInterface->calculateIK(pose,joint_state);
                    if (!succ) {
                        ROS_INFO("IK failed");
                        return false;
                    }
                    else {
                        return mMoveitInterface->isStateValid(joint_state);
                    }
            }
            return false;
        }

        bool isStateValid(const stateType& state_val,
                          const stateType& seed,
                          stateType& joint_state){
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
                    joint_state.resize(mMoveitInterface->num_joints);
                    bool succ = mMoveitInterface->calculateIK(pose, seed,joint_state);
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

        virtual bool getSuccessorsWs(int curr_state_ind,
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
                bool succ; stateType mapped_state;
                if (curr_state->getMappedState().empty()){
//                    ROS_INFO("No mapped state, using IK without seed");
                    succ = isStateValid(new_state_val,
                                        mapped_state);
                }
                else
                    succ = isStateValid(new_state_val,
                                        curr_state->getMappedState(),
                                        mapped_state);
                if (succ) {
                    // create a new state
                    int next_state_ind = getOrCreateState(new_state_val);
                    auto new_state = this->getState(next_state_ind);
                    new_state->setMappedState(mapped_state);
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

        virtual bool getSuccessorsCs(int curr_state_ind,
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

        /// @brief Function that creates the acm to be used by the getMinDistanceAlongPath function. Needs to be updated when the name of the object changes or when new objects are added
        /// @param obj_name 
        void initACM (std::string obj_name) {

            // Format the allowed collision matrix
            collision_detection::AllowedCollisionMatrix acm = mMoveitInterface->mPlanningScene->getAllowedCollisionMatrixNonConst();
            std::vector<std::string> acm_names;
            acm.getAllEntryNames(acm_names);
            collision_detection::AllowedCollisionMatrix new_acm(acm_names, true); // Create a new acm with the names from the old one that allows all collisions
            
            // Update acm to allow collisions with all objects
            std::vector<std::string> world_obj_names = mMoveitInterface->mPlanningScene->getWorld()->getObjectIds();
            //std::vector<std::string> world_obj_names = mMoveitInterface->mPlanningSceneInterface->getKnownObjectNames();
            for (auto name : world_obj_names) {
                new_acm.setEntry(name, true);
            }
            
            // Set it so that only pairs with the object will be computed
            new_acm.setEntry(obj_name, false);
            
            // Modify the new acm so that collisions between the attached object and the manipulator aren't counted
            std::string prefix = m_group->getLinkNames()[0].substr(0,5); // Get prefix ex "arm_1"
            std::vector<std::string> arm_link_names;
            for (const auto& name : acm_names) {
                if (name.compare(0, prefix.size(), prefix) == 0) {
                    arm_link_names.push_back(name);
                }
            }
            new_acm.setEntry(obj_name, arm_link_names, true); // Set it so that pairs between the object and it's manipulator will not be computed

            internal_acm = new_acm;
        }

        /// @brief Get the minimum distance at each point along a trajectory between the attached object and all other collision objects in the space
        /// @param path 
        /// @param obj_name 
        /// @return minimum distance pair
        collision_detection::DistanceResultsData getMinDistanceAlongPath(pathType& path, std::string obj_name, bool visualize) {

            // Initialize the internal acm
            initACM(obj_name);
            
            // Update the planning state monitor
            mMoveitInterface->updatePlanningSceneMonitor();

            // Create a distance request and result for robot-world pairs
            collision_detection::DistanceRequest world_distance_request;
            //world_distance_request.type = collision_detection::DistanceRequestType::SINGLE; // Used to compute distances between all pairs
            world_distance_request.type = collision_detection::DistanceRequestType::GLOBAL;
            world_distance_request.enable_signed_distance = true;
            world_distance_request.enable_nearest_points = true;
            world_distance_request.distance_threshold = 1.0;
            //distance_request.group_name = m_group->getName(); // When this is included, no distance map is computed (guessing it has to do with the allowed collision matrix?)
            world_distance_request.acm = &internal_acm;
            world_distance_request.enableGroup(m_group->getRobotModel());
            

            // Create a distance request and result for robot-robot pairs
            collision_detection::DistanceRequest self_distance_request;
            //self_distance_request.type = collision_detection::DistanceRequestType::SINGLE; // Used to compute distances between all pairs
            self_distance_request.type = collision_detection::DistanceRequestType::GLOBAL;
            self_distance_request.enable_signed_distance = true;
            self_distance_request.enable_nearest_points = true;
            self_distance_request.distance_threshold = 1.0;
            self_distance_request.acm = &internal_acm;
            self_distance_request.enableGroup(m_group->getRobotModel());
            

            // Keep track of the minimum distance pair
            collision_detection::DistanceResultsData min_distance_pair;
            min_distance_pair.distance = std::numeric_limits<double>::infinity();

            // Loop through each possible joint value along the path
            for (const auto& joint_values : path) {

                // Create a robot state based on the joint values
                m_group->setStartStateToCurrentState();
                robot_state::RobotState path_state = mMoveitInterface->mPlanningScene->getCurrentStateNonConst();
                path_state.setJointGroupPositions(m_group->getName(), joint_values);
                path_state.update();

                // Create the results structures
                collision_detection::DistanceResult world_distance_result;
                collision_detection::DistanceResult self_distance_result;


                // Make a request to compute the distances for robot-world and robot-robot pairs
                mMoveitInterface->mPlanningScene->getCollisionEnv()->distanceRobot(world_distance_request, world_distance_result, path_state);
                mMoveitInterface->mPlanningScene->getCollisionEnv()->distanceSelf(self_distance_request, self_distance_result, path_state);

                // Get the distance data for the robot-world pairs and robot-robot pairs
                collision_detection::DistanceMap world_distance_map, self_distance_map;
                world_distance_map = world_distance_result.distances;
                self_distance_map = self_distance_result.distances;

                // Update the minimum distance pair if there is a closer pair
                if (world_distance_result.minimum_distance < min_distance_pair) min_distance_pair = world_distance_result.minimum_distance;
                if (self_distance_result.minimum_distance < min_distance_pair) min_distance_pair = self_distance_result.minimum_distance;
                
                if (visualize) {
                    // Visualize the lines between the two points in space
                    std_msgs::ColorRGBA line_color;
                    line_color.b = 1.0;
                    line_color.a = 0.2;
                    visualizeLine(world_distance_result.minimum_distance.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    visualizeLine(self_distance_result.minimum_distance.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);


                    // Vizualize the minimum distances
                    if (min_distance_pair.distance < 0) {
                        line_color.r = 1.0;
                        line_color.a = 0.5;
                    } else {
                        line_color.r = 0.5;
                        line_color.b = 0.5;
                        line_color.a = 0.2;
                    }
                    visualizeLine(min_distance_pair.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                }
            }
            
            return min_distance_pair;
        }

        void clearLines() {
            ims::clearLines(m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame());
        }


    };
}


#endif //MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP
