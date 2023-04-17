//
// Created by itamar on 4/3/23.
//

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
#include <MoveitInterface.hpp>
#include <utils.hpp>


struct manipulationType : ims::actionType {

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

   /// @brief In case of planning in the workspace, create a kdtree object for discretization
   /// @param KDTree The kdtree object
   /// @param psiVector The psi vector for discretization
    void DiscretizationWS(KDTree<Eigen::Vector3d>* KDTree,
                              std::vector<double>& psiVector) {
          mKDTree = KDTree;
          mPsiVector = psiVector;
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
    KDTree<Eigen::Vector3d>* mKDTree;
    std::vector<double> mPsiVector;
};


/// @class ManipulationActionSpace
/// @brief A class that implements the ActionSpace for Moveit
class ManipulationActionSpace : public ims::actionSpace {
private:
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
                            const manipulationType& actions_ptr) : ims::actionSpace(){
        mMoveitInterface = std::make_shared<MoveitInterface>(env);
        mManipulationType = std::make_shared<manipulationType>(actions_ptr);
        // get the joint limits
        mMoveitInterface->getJointLimits(mJointLimits);
    }

    bool isStateValid(const stateType& state_val) override {
        // check if the state is valid
        switch (mManipulationType->getSpaceType()) {
            case manipulationType::spaceType::ConfigurationSpace:
                return mMoveitInterface->isStateValid(state_val);
            case manipulationType::spaceType::WorkSpace:
                geometry_msgs::Pose pose;
                pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                // If using hopf coordinates as the state, convert to quaternions
                Eigen::Quaterniond q;
                Eigen::Vector3d Hopf(state_val[3], state_val[4], state_val[5]);
                hopfToQuaternion(Hopf, q);
                pose.orientation.x = q.x(); pose.orientation.y = q.y();
                pose.orientation.z = q.z(); pose.orientation.w = q.w();
                // If using quaternion coordinates as the state
//                pose.orientation.x = state_val[3]; pose.orientation.y = state_val[4];
//                pose.orientation.z = state_val[5]; pose.orientation.w = state_val[6];
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
                    // If using hopf coordinates as the state, convert to quaternions
                    Eigen::Quaterniond q;
                    Eigen::Vector3d Hopf(state[3], state[4], state[5]);
                    hopfToQuaternion(Hopf, q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y();
                    pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    // If using quaternion coordinates as the state
//                pose.orientation.x = state_val[3]; pose.orientation.y = state_val[4];
//                pose.orientation.z = state_val[5]; pose.orientation.w = state_val[6];

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

    /// \brief A function that dealing with the discontinuity of the joint angles
    /// \param angles The joint angles
    void NormalizeAngles(stateType& angles) {
        for (int i {0} ; i < angles.size() ; i++) {
            if (angles[i] > mJointLimits[i].second) {
                angles[i] = angles[i] - 2 * M_PI;
            }
            else if (angles[i] < mJointLimits[i].first) {
                angles[i] = angles[i] + 2 * M_PI;
            }
        }
    }


    bool getSuccessorsWs(int curr_state_ind,
                         std::vector<ims::state*>& successors,
                         std::vector<double>& costs) {
        // get the current state
        auto curr_state = this->getState(curr_state_ind);
        auto curr_state_val = curr_state->getState();
        std::cout << "curr_state_val: " << curr_state_val[0] << ", " << curr_state_val[1] << ", " << curr_state_val[2] << ", " << curr_state_val[3] << ", " << curr_state_val[4] << ", " << curr_state_val[5] << std::endl;
        // get the actions
        auto actions = mManipulationType->getActions();
        Eigen::Quaterniond q_curr;
        // If using the hopf coordinates as the state
        Eigen::Vector3d Hopf_curr = {curr_state_val[3], curr_state_val[4], curr_state_val[5]};
        hopfToQuaternion(Hopf_curr, q_curr);

        // If using the quaternion coordinates as the state
//        q_curr = {curr_state_val[6], curr_state_val[3], curr_state_val[4], curr_state_val[5]};
        // get the successors
        stateType new_state_val;
        for (auto action : actions) {
            // create a new state in the length of the current state
            new_state_val.resize(3);
            // increment the xyz coordinates
            for (int i {0} ; i < 3 ; i++) {
                new_state_val[i] = curr_state_val[i] + action[i];
            }
            // round the xyz coordinates
            roundStateToDiscretization(new_state_val, mManipulationType->mStateDiscretization);

            Eigen::Quaterniond q_action {action[6], action[3], action[4], action[5]};
            Eigen::Quaterniond q_new_total = q_curr * q_action;
            q_new_total.normalize();
            // make sure the quaternion is antipodal
            if (q_new_total.w() < 0) {
                q_new_total.x() = -q_new_total.x();
                q_new_total.y() = -q_new_total.y();
                q_new_total.z() = -q_new_total.z();
                q_new_total.w() = -q_new_total.w();
            }

            // If using hopf coordinates as the state
            Eigen::Vector3d Hopf_new;
            quaternionToHopf(q_new_total, Hopf_new);
            getClosestHopf(Hopf_new, mManipulationType->mKDTree, mManipulationType->mPsiVector);
            new_state_val.resize(6);
            new_state_val[3] = Hopf_new[0]; new_state_val[4] = Hopf_new[1]; new_state_val[5] = Hopf_new[2];
            // If using quaternion coordinates as the state
//            getClosestQuaternion(quat_proj, mManipulationType->mKDTree, mManipulationType->mPsiVector);
//            // normalize the quaternion
//            std::vector<double> quat_proj {q_curr.x(), q_curr.y(), q_curr.z(), q_curr.w()};
//            new_state_val[3] = quat_proj[0]; new_state_val[4] = quat_proj[1];
//            new_state_val[5] = quat_proj[2]; new_state_val[6] = quat_proj[3];


            // check if the state is valid by linear interpolation
//            if (isStateToStateValid(curr_state_val, new_state_val)) {
            if (isStateValid(new_state_val)) {
                // create a new state
                int next_state_ind = getOrCreateState(new_state_val);
                auto new_state = this->getState(next_state_ind);
                // add the state to the successors
                successors.push_back(new_state);
                // add the cost
                double cost {0}; int ind {0};
                for (double i : action) {
                    // the first three elements are the xyz coordinates and the last four are the quaternion coordinates
                    if (ind < 3) {
                        cost += i * i;
                    } else {
                        cost += i * i * 0.1; // TODO: change this to the real cost
                    }
                    ind++;
                }
                costs.push_back(cost);
            }
        }
        return true;
    }

    bool getSuccessorsCs(int curr_state_ind,
                         std::vector<ims::state*>& successors,
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
            NormalizeAngles(new_state_val);
            // discretize the state
            roundStateToDiscretization(new_state_val, mManipulationType->mStateDiscretization);

            // check if the state is valid by linear interpolation
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
                       std::vector<ims::state*>& successors,
                       std::vector<double>& costs) override {
        if (mManipulationType->getSpaceType() == manipulationType::spaceType::ConfigurationSpace) {
            return getSuccessorsCs(curr_state_ind, successors, costs);
        } else {
            return getSuccessorsWs(curr_state_ind, successors, costs);
        }
    }
};

#endif //MANIPULATION_PLANNING_MANIPULATIONACTIONSPACE_HPP
