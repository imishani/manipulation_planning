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

#include <tf2_ros/transform_listener.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/attached_body.h>

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
        ros::Publisher robot_state_pub;
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
            m_vis_pub = m_nh.advertise<visualization_msgs::Marker>( "line_marker", 10 );
            robot_state_pub = m_nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);
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

        /// @brief Transform vector3d point from the world frame to the target frame
        /// @param point Point from the world frame that is updated to the target frame
        /// @param target_frame Frame that specifies the new orgin in space for which we want to find the point's coordinates with respect to
        /// @param robot_state The state that we want to find this transformation within
        /// @return True is the transformation is successful and false if it is not
        bool transformFromWorldFrame(Eigen::Vector3d& point, std::string target_frame, robot_state::RobotState& robot_state) {

            // Convert the 3d vector to a 4d vector
            Eigen::Vector4d world_vec = {point.x(), point.y(), point.z(), 1.0};

            // Check if we know the frame transform
            if (robot_state.knowsFrameTransform(target_frame)) {

                // Retrieve the frame transform
                Eigen::Isometry3d transform = robot_state.getFrameTransform(target_frame);

                // Compute the transformation of point to the end effector's frame and return
                Eigen::Vector4d end_effector_point = transform.inverse() * world_vec;

                // Update the values in the point
                point = end_effector_point.head<3>();
                return true;

            } else {
                ROS_ERROR("Frame transform unknown");
                return false;
            }
        }

        /// @brief Function that creates the acm to be used by the getMinDistanceAlongPath function. Needs to be updated when the name of the object changes or when new objects are added
        /// @param obj_name 
        collision_detection::AllowedCollisionMatrix getObjectACM (std::string obj_name) {

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
            
            //std::string prefix = m_group->getLinkNames()[0].substr(0,5); // Get prefix ex "arm_1"
            std::string prefix = "arm_1robotiq";
            std::vector<std::string> arm_link_names;
            for (const auto& name : acm_names) {
                if (name.compare(0, prefix.size(), prefix) == 0) {
                    arm_link_names.push_back(name);
                }
            }
            new_acm.setEntry(obj_name, arm_link_names, true); // Set it so that pairs between the object and it's manipulator will not be computed
            

            return new_acm;
        }

        void initACM (std::string obj_name) {
            internal_acm = getObjectACM(obj_name);
        }

        using distance_result = std::pair<collision_detection::DistanceResultsData, robot_state::RobotState>;

        /// @brief Get the minimum distance data result across the entire trajectory for each state and the closest collision object
        /// @param path The trajectory of the robot (in joint space)
        /// @param obj_name The name of the attached object that we are checking for distances with
        /// @param result_frame The frame with which we want the nearest points to be in
        /// @param visualize True if we want to generate markers displaying the closest collision at each point along the trajectory
        /// @return minimum distance result data object (contains info about the minimum distance and the nearest points w.r.t result frame)
        distance_result getMinDistanceAlongPath(pathType& path, std::string obj_name, std::string result_frame, collision_detection::CollisionResult::ContactMap& contact_map, bool visualize) {

            // Clear the existing lines
            if (visualize) clearLines();

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
            world_distance_request.max_contacts_per_body = 5;
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
            self_distance_request.max_contacts_per_body = 5;
            self_distance_request.acm = &internal_acm;
            self_distance_request.enableGroup(m_group->getRobotModel());

            // Keep track of the minimum distance pair
            collision_detection::DistanceResultsData min_distance_pair;
            min_distance_pair.distance = std::numeric_limits<double>::infinity();
            robot_state::RobotState min_distance_state = mMoveitInterface->mPlanningScene->getCurrentStateNonConst();

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

                // Update the minimum distance pair and state if there is a closer pair
                if (world_distance_result.minimum_distance < min_distance_pair) {
                    min_distance_pair = world_distance_result.minimum_distance;
                    min_distance_state = path_state;
                }
                if (self_distance_result.minimum_distance < min_distance_pair) {
                    min_distance_pair = self_distance_result.minimum_distance;
                    min_distance_state = path_state;
                }
                
                if (visualize) {

                    // Visualize the current state
                    publishState(path_state);

                    // Visualize the lines between the two points in space
                    std_msgs::ColorRGBA line_color;
                    line_color.b = 1.0;
                    line_color.a = 0.2;
                    visualizeLine(world_distance_result.minimum_distance.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    visualizeLine(self_distance_result.minimum_distance.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);

                    // Vizualize the minimum distances
                    line_color.r = 1.0;
                    line_color.a = 0.5;
                    visualizePoint(min_distance_pair.nearest_points[0], m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    visualizeLine(min_distance_pair.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    //visualizePoint(min_distance_pair.nearest_points[0], m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    //visualizePoint(min_distance_pair.nearest_points[1], m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                }
            }

            publishState(min_distance_state);

            // If there is a collision, compute the contact map
            if (min_distance_pair.distance < 0) {

                // Once we know the minimum state along the path, compute the contact map
                collision_detection::CollisionRequest collision_request;
                collision_request.contacts = true;
                collision_request.max_contacts = 2000;
                collision_request.max_contacts_per_pair = 400;
                collision_detection::CollisionResult collision_result;

                // Check for a collision
                mMoveitInterface->mPlanningScene->getCollisionEnv()->checkCollision(collision_request, collision_result, min_distance_state, internal_acm);

                // Update the contact map
                contact_map = collision_result.contacts;

                // Convert the values to the frame of the attached object
                for (auto& pair : contact_map)
                    for (auto& contact : pair.second)
                        transformFromWorldFrame(contact.pos, result_frame, min_distance_state);
            } else {

                // Clear the contact map is there is no collision
                contact_map.clear();
            }

            // Convert distance result points to the proper frame
            if (result_frame != mMoveitInterface->mPlanningScene->getPlanningFrame()) {
                transformFromWorldFrame(min_distance_pair.nearest_points[0], result_frame, min_distance_state);
                transformFromWorldFrame(min_distance_pair.nearest_points[1], result_frame, min_distance_state);
                transformFromWorldFrame(min_distance_pair.normal, result_frame, min_distance_state);
            }

            return std::make_pair(min_distance_pair, min_distance_state);
        }


        /// @brief Get the minimum distance data result across the entire trajectory for each state and the closest collision object
        /// @param path The trajectory of the robot (in joint space)
        /// @param obj_name The name of the attached object that we are checking for distances with
        /// @param result_frame The frame with which we want the nearest points to be in
        /// @param visualize True if we want to generate markers displaying the closest collision at each point along the trajectory
        /// @return minimum distance result data object (contains info about the minimum distance and the nearest points w.r.t result frame)
        distance_result getMinDistanceAlongPath(pathType& path, std::string obj_name, std::string result_frame, std::vector<Eigen::Vector3d>& min_distance_points, bool visualize) {

            // Clear the existing lines
            if (visualize) clearLines();

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
            world_distance_request.max_contacts_per_body = 5;
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
            self_distance_request.max_contacts_per_body = 5;
            self_distance_request.acm = &internal_acm;
            self_distance_request.enableGroup(m_group->getRobotModel());

            // Keep track of the minimum distance pair
            collision_detection::DistanceResultsData min_distance_pair;
            min_distance_pair.distance = std::numeric_limits<double>::infinity();
            robot_state::RobotState min_distance_state = mMoveitInterface->mPlanningScene->getCurrentStateNonConst();

            // Loop through each possible joint value along the path
            min_distance_points.clear();
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

                // Update the minimum distance pair and state if there is a closer pair
                if (world_distance_result.minimum_distance < min_distance_pair) {
                    min_distance_pair = world_distance_result.minimum_distance;
                    min_distance_state = path_state;
                }
                if (self_distance_result.minimum_distance < min_distance_pair) {
                    min_distance_pair = self_distance_result.minimum_distance;
                    min_distance_state = path_state;
                }
                
                if (visualize) {

                    // Visualize the current state
                    publishState(path_state);

                    // Visualize the lines between the two points in space
                    std_msgs::ColorRGBA line_color;
                    line_color.b = 1.0;
                    line_color.a = 0.2;
                    visualizeLine(world_distance_result.minimum_distance.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    visualizeLine(self_distance_result.minimum_distance.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);

                    // Vizualize the minimum distances
                    line_color.r = 1.0;
                    line_color.a = 0.5;
                    visualizePoint(min_distance_pair.nearest_points[0], m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    visualizeLine(min_distance_pair.nearest_points, m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    //visualizePoint(min_distance_pair.nearest_points[0], m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                    //visualizePoint(min_distance_pair.nearest_points[1], m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame(), line_color, m_vis_id++);
                }

                // Add the pairs to the list to check if they have a negative distance
                if (world_distance_result.minimum_distance.distance < 0) {
                    Eigen::Vector3d new_point = world_distance_result.minimum_distance.nearest_points[0];
                    transformFromWorldFrame(new_point, result_frame, path_state);
                    min_distance_points.push_back(new_point);
                }
                if (self_distance_result.minimum_distance.distance < 0) {
                    Eigen::Vector3d new_point = self_distance_result.minimum_distance.nearest_points[0];
                    transformFromWorldFrame(new_point, result_frame, path_state);
                    min_distance_points.push_back(new_point);
                }

            }

            publishState(min_distance_state);

            return std::make_pair(min_distance_pair, min_distance_state);
        }

        // Eigen::Vector3d getUnitVector (robot_state::RobotState& robot_state, std::string object, std::string obstacle) {

        //     Eigen::Vector3d obstacle_position;
        //     if (!robot_state.knowsFrameTransform(obstacle)) {
        //         geometry_msgs::Pose pose = mMoveitInterface->mPlanningSceneInterface->getObjects({obstacle})[0].primitive_poses[0];
        //         obstacle_position = {pose.position.x, pose.position.y, pose.position.z};
        //     } else {
        //         Eigen::Isometry3d transform_to_obstacle = robot_state.getFrameTransform(obstacle);
        //         obstacle_position = {transform_to_obstacle(0,3), transform_to_obstacle(1,3), transform_to_obstacle(2,3)};
        //     }

        //     // Retrieve the frame transforms
        //     Eigen::Isometry3d transform_to_object = robot_state.getFrameTransform(object);
        //     Eigen::Vector3d object_position(transform_to_object(0,3), transform_to_object(1,3), transform_to_object(2,3));

        //     //Eigen::Isometry3d object_to_obstacle = transform_to_obstacle - transform_to_object;
        //     Eigen::Vector3d result = obstacle_position - object_position;
        //     result.normalized();

        //     return result;
        // }

        void clearLines() {
            ims::clearLines(m_vis_pub, mMoveitInterface->mPlanningScene->getPlanningFrame());
        }

        void publishState(robot_state::RobotState& robot_state) {
            ims::publishState(robot_state_pub, robot_state);
        }
    };
}


#endif //MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP
