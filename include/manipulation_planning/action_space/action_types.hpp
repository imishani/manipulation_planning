//
// Created by itamar on 10/6/24.
//

#ifndef ACTION_TYPES_HPP
#define ACTION_TYPES_HPP

// project includes
#include <search/action_space/action_space.hpp>

#include <manipulation_planning/common/utils.hpp>

namespace ims {

struct AdaptiveActionTypeMixin {

    virtual ~AdaptiveActionTypeMixin() = default;

    virtual ActionSequence getAdaptivePrimActions(const double &start_dist, const double &goal_dist) = 0;
};

struct ActionTypeMixed : virtual ActionType, AdaptiveActionTypeMixin {};


struct ManipulationType : ActionTypeMixed {

    /// @brief Constructor
    ManipulationType() : action_type_(ActionType::MOVE),
                         space_type_(SpaceType::ConfigurationSpace),
                         max_action_(0.0) {
        mprim_file_name_ = ros::package::getPath("manipulation_planning") + "/config/manip_6dof.mprim";
    };

    /// @brief Constructor with motion primitives file given
    /// @param[in] mprim_file The path to the motion primitives file
    explicit ManipulationType(std::string mprim_file) : action_type_(ActionType::MOVE),
                                                        space_type_(SpaceType::ConfigurationSpace),
                                                        mprim_file_name_(std::move(mprim_file)),
                                                        max_action_(0.0){
                                                            //            readMPfile();
                                                        };

    /// @brief Constructor with adaptive motion primitives given
    /// @brief Destructor
    ~ManipulationType() override = default;

    /// @brief The type of the action
    enum class ActionType {
        MOVE,
        GRASP,
        RELEASE
    };

    enum class SpaceType {
        ConfigurationSpace,
        WorkSpace
    };
    /// @{ getters and setters
    /// @brief Get the action type
    /// @return The action type
    ActionType getActionType() const {
        return action_type_;
    }

    /// @brief Set the action type
    /// @param ActionType The action type
    void setActionType(ActionType ActionType) {
        action_type_ = ActionType;
    }

    /// @brief Get the space type
    /// @return The space type
    SpaceType getSpaceType() const {
        return space_type_;
    }

    /// @brief Set the space type
    /// @param SpaceType The space type
    void setSpaceType(SpaceType SpaceType) {
        space_type_ = SpaceType;
    }
    /// @}

    void Discretization(StateType &state_des) override {
        state_discretization_ = state_des;
    }

    void readMPfile() {
        std::ifstream file(mprim_file_name_);
        std::string line;
        switch (space_type_) {
            case SpaceType::ConfigurationSpace: {
                int tot_prim{0}, dof{0}, num_short_prim{0};
                int i{0};
                while (std::getline(file, line)) {
                    if (i == 0) {
                        // First line being with: "Motion_Primitives(degrees): " and then three numbers. Make sure the line begins with the string and then get the numbers
                        std::string first_line = "Motion_Primitives(degrees): ";
                        // Check if the line begins with the string
                        if (line.find(first_line) != 0) {
                            ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                        }
                        // Get the numbers
                        std::istringstream iss(line.substr(first_line.size()));
                        if (!(iss >> tot_prim >> dof >> num_short_prim)) {
                            ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                        }
                        i++;
                        continue;
                    }
                    std::istringstream iss(line);
                    std::vector<double> line_;
                    double num;
                    while (iss >> num) {
                        line_.push_back(num);
                        if (abs(num * M_PI / 180.0) > max_action_) {
                            max_action_ = abs(num * M_PI / 180.0);
                        }
                    }
                    // Check if short or long primitive (the last num_short_prim lines are short)
                    if (i > tot_prim - num_short_prim) {
                        short_mprim_.push_back(line_);
                    }
                    else {
                        long_mprim_.push_back(line_);
                    }
                    i++;
                }
            } break;
            case SpaceType::WorkSpace: {
                int tot_ptim{0}, positions_prims{0}, orientations_prims{0};
                int i{0};
                while (std::getline(file, line)) {
                    if (i == 0) {
                        // First line being with: "Motion_Primitives(degrees): " and then three numbers. Make sure the line begins with the string and then get the numbers
                        std::string first_line = "Motion_Primitives(meters/degrees): ";
                        // Check if the line begins with the string
                        if (line.find(first_line) != 0) {
                            ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                        }
                        // Get the numbers
                        std::istringstream iss(line.substr(first_line.size()));
                        if (!(iss >> tot_ptim >> positions_prims >> orientations_prims)) {
                            ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                        }
                        i++;
                        continue;
                    }
                    std::istringstream iss(line);
                    std::vector<double> line_;
                    double num;
                    while (iss >> num) {
                        line_.push_back(num);
                        if (abs(num) > max_action_) {
                            max_action_ = abs(num);
                        }
                    }
                    // TODO: Currently I am using short_mprim_ to store the work space motion primitives. This is not correct.
                    short_mprim_.push_back(line_);
                    i++;
                }
            }
        }
    }

    /// @brief Get the possible actions
    /// @return A vector of all possible actions
    std::vector<Action> getPrimActions() override {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        if (actions_.empty()) {
            switch (action_type_) {
                case ActionType::MOVE:
                    switch (space_type_) {
                        case SpaceType::ConfigurationSpace: {
                            // TODO: Add snap option
                            std::vector<std::vector<double>> mprim;
                            mprim.insert(mprim.end(), long_mprim_.begin(), long_mprim_.end());
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
                        } break;
                        case SpaceType::WorkSpace: {
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
                        } break;
                    }
                    break;
                case ActionType::GRASP:
                    break;
                case ActionType::RELEASE:
                    break;
            }
            return actions_;
        }
        else {
            return actions_;
        }
    }

    ActionSequence getAdaptivePrimActions(const double &start_dist, const double &goal_dist) override {
        return getAdaptiveActions(start_dist, goal_dist);
    }

    /// @brief Get adaptive motion primitives
    /// @param start_dist The distance from the start
    /// @param goal_dist The distance from the goal
    /// @return A vector of actions
    ActionSequence getAdaptiveActions(const double &start_dist, const double &goal_dist) {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        actions_.clear();
        if (mprim_active_type_.long_dist.first)  // insert long distance primitive and convert to radians
            for (auto &action_ : long_mprim_) {
                std::vector<double> action, action_rev;
                for (const auto &num : action_) {
                    action.push_back(num * M_PI / 180.0);
                    action_rev.push_back(-num * M_PI / 180.0);
                }
                actions_.push_back(action);
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.short_dist.first && (start_dist < mprim_active_type_.short_dist.second || goal_dist < mprim_active_type_.short_dist.second))
            for (auto &action_ : short_mprim_) {
                std::vector<double> action, action_rev;
                for (const auto &num : action_) {
                    action.push_back(num * M_PI / 180.0);
                    action_rev.push_back(-num * M_PI / 180.0);
                }
                actions_.push_back(action);
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.snap_xyz.first && goal_dist < mprim_active_type_.snap_xyz.second)
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
        if (mprim_active_type_.snap_rpy.first && goal_dist < mprim_active_type_.snap_rpy.second)
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
        if (mprim_active_type_.snap_xyzrpy.first && goal_dist < mprim_active_type_.snap_xyzrpy.second) {
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE,
                                INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
            ROS_DEBUG_NAMED("adaptive_mprim", "snap xyzrpy");
            ROS_DEBUG_STREAM("goal_dist: " << goal_dist);
        }
        return actions_;
    }

    /// @brief Set values in the motion primitive active type.
    /// @param short_dist The short distance threshold
    /// @param long_dist The long distance threshold
    /// @param snap_xyz The snap xyz threshold
    /// @param snap_rpy The snap rpy threshold
    /// @param snap_xyzrpy The snap xyzrpy threshold
    void setMprimActiveType(const std::pair<bool, double> &short_dist,
                            const std::pair<bool, double> &long_dist,
                            const std::pair<bool, double> &snap_xyz,
                            const std::pair<bool, double> &snap_rpy,
                            const std::pair<bool, double> &snap_xyzrpy) {
        mprim_active_type_.short_dist = short_dist;
        mprim_active_type_.long_dist = long_dist;
        mprim_active_type_.snap_xyz = snap_xyz;
        mprim_active_type_.snap_rpy = snap_rpy;
        mprim_active_type_.snap_xyzrpy = snap_xyzrpy;
    }

    /// @brief Motion primitive active type: Used for adaptive motion primitives, given a few motion primitives,
    /// which one is active at a given time and it's threshold
    struct MotionPrimitiveActiveType {
        std::pair<bool, double> short_dist = std::make_pair(true, 0.2);
        std::pair<bool, double> long_dist = std::make_pair(true, 0.4);
        std::pair<bool, double> snap_xyz = std::make_pair(false, 0.2);
        std::pair<bool, double> snap_rpy = std::make_pair(false, 0.2);
        std::pair<bool, double> snap_xyzrpy = std::make_pair(true, 0.5);
    };

    ActionType action_type_;
    SpaceType space_type_;
    std::string mprim_file_name_;
    MotionPrimitiveActiveType mprim_active_type_;

    std::vector<Action> actions_;
    std::vector<Action> short_mprim_;
    std::vector<Action> long_mprim_;

    std::vector<bool> mprim_enabled_;
    std::vector<double> mprim_thresh_;
    double max_action_;

};

struct OmniBaseActionType : ActionTypeMixed {

    /// @brief Constructor
    OmniBaseActionType() {
        mprim_file_name_ = ros::package::getPath("manipulation_planning") + "/config/omni_base.mprim";
    }

    void Discretization(StateType &state_des) override {
        state_discretization_ = state_des;
    }

    void readMPfile() {
        std::ifstream file(mprim_file_name_);
        std::string line;
        int tot_prim{0}, dof{0}, num_short_prim{0};
        int i{0};
        while (std::getline(file, line)) {
            if (i == 0) {
                // First line being with: "Motion_Primitives(meters/radians): " and then three numbers. Make sure the line begins with the string and then get the numbers
                std::string first_line = "Motion_Primitives(meters/radians): ";
                // Check if the line begins with the string
                if (line.find(first_line) != 0) {
                    ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                }
                // Get the numbers
                std::istringstream iss(line.substr(first_line.size()));
                if (!(iss >> tot_prim >> dof >> num_short_prim)) {
                    ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                }
                i++;
                continue;
            }
            std::istringstream iss(line);
            std::vector<double> line_;
            double num;
            while (iss >> num) {
                line_.push_back(num);
            }
            // Check if short or long primitive (the last num_short_prim lines are short)
            if (i > tot_prim - num_short_prim) {
                short_mprim_.push_back(line_);
            }
            else {
                long_mprim_.push_back(line_);
            }
            i++;
        }
    }

    ActionSequence getPrimActions() override {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        if (actions_.empty()) {
            std::vector<std::vector<double>> mprim;
            mprim.insert(mprim.end(), long_mprim_.begin(), long_mprim_.end());
            mprim.insert(mprim.end(), short_mprim_.begin(), short_mprim_.end());
            for (auto &action_ : mprim) {
                actions_.push_back(action_);
                // get the opposite action
                for (auto &num : action_) {
                    num = -num;
                }
                actions_.push_back(action_);
            }
            return actions_;
        }
        else {
            return actions_;
        }
    }

    ActionSequence getAdaptivePrimActions(const double &start_dist, const double &goal_dist) override {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        actions_.clear();
        if (mprim_active_type_.long_dist.first)  // insert long distance primitive
            for (auto &action_ : long_mprim_) {
                actions_.push_back(action_);
                // negate
                Action action_rev(action_.size());
                for (int i = 0; i < action_.size(); i++) {
                    action_rev[i] = -action_[i];
                }
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.short_dist.first && (start_dist < mprim_active_type_.short_dist.second || goal_dist < mprim_active_type_.short_dist.second))
        // if (mprim_active_type_.short_dist.first && goal_dist < mprim_active_type_.short_dist.second)
            for (auto &action_ : short_mprim_) {
                actions_.push_back(action_);
                // negate
                Action action_rev(action_.size());
                for (int i = 0; i < action_.size(); i++) {
                    action_rev[i] = -action_[i];
                }
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.snap_xy.first && goal_dist < mprim_active_type_.snap_xy.second)
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
        if (mprim_active_type_.snap_yaw.first && goal_dist < mprim_active_type_.snap_yaw.second)
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
        if (mprim_active_type_.snap_xyyaw.first && goal_dist < mprim_active_type_.snap_xyyaw.second) {
            actions_.push_back({INF_DOUBLE, INF_DOUBLE, INF_DOUBLE});  // TODO: Fix this to make it better designed
            ROS_DEBUG_NAMED("adaptive_mprim", "snap xyyaw");
            ROS_DEBUG_STREAM("goal_dist: " << goal_dist);
        }
        return actions_;
    }

    /// @brief Set values in the motion primitive active type.
    /// @param short_dist The short distance threshold
    /// @param long_dist The long distance threshold
    /// @param snap_xy The snap xy threshold
    /// @param snap_yaw The snap yaw threshold
    /// @param snap_xyyaw The snap xy yaw threshold
    void setMprimActiveType(const std::pair<bool, double> &short_dist,
                            const std::pair<bool, double> &long_dist,
                            const std::pair<bool, double> &snap_xy,
                            const std::pair<bool, double> &snap_yaw,
                            const std::pair<bool, double> &snap_xyyaw) {
        mprim_active_type_.short_dist = short_dist;
        mprim_active_type_.long_dist = long_dist;
        mprim_active_type_.snap_xy = snap_xy;
        mprim_active_type_.snap_yaw = snap_yaw;
        mprim_active_type_.snap_xyyaw = snap_xyyaw;
    }

    /// @brief Motion primitive active type: Used for adaptive motion primitives, given a few motion primitives,
    /// which one is active at a given time and it's threshold
    struct MotionPrimitiveActiveType {
        std::pair<bool, double> short_dist = std::make_pair(true, 1.0);
        std::pair<bool, double> long_dist = std::make_pair(true, 1.0);
        std::pair<bool, double> snap_xy = std::make_pair(false, 2.0);
        std::pair<bool, double> snap_yaw = std::make_pair(false, 0.4);
        std::pair<bool, double> snap_xyyaw = std::make_pair(true, 2.0);
    };

    std::string mprim_file_name_;

    MotionPrimitiveActiveType mprim_active_type_;

    std::vector<Action> actions_;
    std::vector<Action> short_mprim_;
    std::vector<Action> long_mprim_;

    std::vector<bool> mprim_enabled_;
    std::vector<double> mprim_thresh_;

};

struct TorsoActionType : ActionTypeMixed {
    /// @brief Constructor
    TorsoActionType() {
        mprim_file_name_ = ros::package::getPath("manipulation_planning") + "/config/torso.mprim";
    }

    void Discretization(StateType &state_des) override {
        state_discretization_ = state_des;
    }

    void readMPfile() {
        std::ifstream file(mprim_file_name_);
        std::string line;
        int tot_prim{0}, dof{0}, num_short_prim{0};
        int i{0};
        while (std::getline(file, line)) {
            if (i == 0) {
                // First line being with: "Motion_Primitives(meters/radians): " and then three numbers. Make sure the line begins with the string and then get the numbers
                std::string first_line = "Motion_Primitives(meters/radians): ";
                // Check if the line begins with the string
                if (line.find(first_line) != 0) {
                    ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                }
                // Get the numbers
                std::istringstream iss(line.substr(first_line.size()));
                if (!(iss >> tot_prim >> dof >> num_short_prim)) {
                    ROS_ERROR_STREAM("The first line of the motion primitives file should begin with: " << first_line);
                }
                i++;
                continue;
            }
            std::istringstream iss(line);
            std::vector<double> line_;
            double num;
            while (iss >> num) {
                line_.push_back(num);
            }
            // Check if short or long primitive (the last num_short_prim lines are short)
            if (i > tot_prim - num_short_prim) {
                short_mprim_.push_back(line_);
            }
            else {
                long_mprim_.push_back(line_);
            }
            i++;
        }
    }

    ActionSequence getPrimActions() override {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        if (actions_.empty()) {
            std::vector<std::vector<double>> mprim;
            mprim.insert(mprim.end(), long_mprim_.begin(), long_mprim_.end());
            mprim.insert(mprim.end(), short_mprim_.begin(), short_mprim_.end());
            for (auto &action_ : mprim) {
                actions_.push_back(action_);
                // get the opposite action
                for (auto &num : action_) {
                    num = -num;
                }
                actions_.push_back(action_);
            }
            return actions_;
        }
        else {
            return actions_;
        }
    }

    ActionSequence getAdaptivePrimActions(const double &start_dist, const double &goal_dist) override {
        if (short_mprim_.empty() && long_mprim_.empty()) {
            readMPfile();
        }
        actions_.clear();
        if (mprim_active_type_.long_dist.first)  // insert long distance primitive
            for (auto &action_ : long_mprim_) {
                actions_.push_back(action_);
                // negate
                Action action_rev(action_.size());
                for (int i = 0; i < action_.size(); i++) {
                    action_rev[i] = -action_[i];
                }
                actions_.push_back(action_rev);
            }
        // if (mprim_active_type_.short_dist.first && (start_dist < mprim_active_type_.short_dist.second || goal_dist < mprim_active_type_.short_dist.second))
        if (mprim_active_type_.short_dist.first && goal_dist < mprim_active_type_.short_dist.second)
            for (auto &action_ : short_mprim_) {
                actions_.push_back(action_);
                // negate
                Action action_rev(action_.size());
                for (int i = 0; i < action_.size(); i++) {
                    action_rev[i] = -action_[i];
                }
                actions_.push_back(action_rev);
            }
        if (mprim_active_type_.snap_z.first && goal_dist < mprim_active_type_.snap_z.second)
            actions_.push_back({INF_DOUBLE});  // TODO: Fix this to make it better designed
        return actions_;
    }

    /// @brief Set values in the motion primitive active type.
    /// @param short_dist The short distance threshold
    /// @param long_dist The long distance threshold
    /// @param snap_z The snap z threshold
    void setMprimActiveType(const std::pair<bool, double> &short_dist,
                            const std::pair<bool, double> &long_dist,
                            const std::pair<bool, double> &snap_z) {
        mprim_active_type_.short_dist = short_dist;
        mprim_active_type_.long_dist = long_dist;
        mprim_active_type_.snap_z = snap_z;
    }

    struct MotionPrimitiveActiveType {
        std::pair<bool, double> short_dist = std::make_pair(true, 0.1);
        std::pair<bool, double> long_dist = std::make_pair(true, 0.2);
        std::pair<bool, double> snap_z = std::make_pair(true, 0.1);
    };

    std::string mprim_file_name_;

    MotionPrimitiveActiveType mprim_active_type_;

    std::vector<Action> actions_;
    std::vector<Action> short_mprim_;
    std::vector<Action> long_mprim_;

    std::vector<bool> mprim_enabled_;
    std::vector<double> mprim_thresh_;
};

struct MobileManipulationType : ActionTypeMixed {

    enum group_type {
        OMNI_BASE,
        // DIFF_DRIVE_BASE,
        ARM7DOF,
        ARM6DOF,
        TORSO,
    };


    std::vector<std::pair<std::string, std::shared_ptr<ActionTypeMixed>>> sub_groups_action_types_;

    void init(std::vector<std::pair<std::string, group_type>> &sub_groups) {

        StateType OMNI_BASE_DISCRETIZATION {0.1, 0.1, 1.0 * M_PI / 180.0};
        // StateType DIFF_DRIVE_BASE_DISCRETIZATION {0.1, 0.1, 1.0 * M_PI / 180.0},
        StateType ARM6DOF_DISCRETIZATION(6, 1.0);
        StateType ARM7DOF_DISCRETIZATION(7, 1.0);
        deg2rad(ARM6DOF_DISCRETIZATION);
        deg2rad(ARM7DOF_DISCRETIZATION);
        StateType TORSO_DISCRETIZATION {0.1};

        for (auto &sub_group : sub_groups) {
            switch (sub_group.second) {
                case OMNI_BASE: {
                    auto omni_base_action_type = std::make_shared<OmniBaseActionType>();
                    omni_base_action_type->Discretization(OMNI_BASE_DISCRETIZATION);
                    state_discretization_.insert(state_discretization_.end(), OMNI_BASE_DISCRETIZATION.begin(), OMNI_BASE_DISCRETIZATION.end());
                    sub_groups_action_types_.emplace_back(sub_group.first, omni_base_action_type);
                } break;
                // case DIFF_DRIVE_BASE: {
                //     auto diff_drive_base_action_type = std::make_shared<DiffDriveBaseActionType>();
                //     sub_groups_action_types_.emplace_back(sub_group.first, diff_drive_base_action_type);
                // } break;
                case ARM7DOF: {
                    auto arm7dof_action_type = std::make_shared<ManipulationType>("../config/manip_7dof.mprim");
                    arm7dof_action_type->Discretization(ARM7DOF_DISCRETIZATION);
                    state_discretization_.insert(state_discretization_.end(), ARM7DOF_DISCRETIZATION.begin(), ARM7DOF_DISCRETIZATION.end());
                    sub_groups_action_types_.emplace_back(sub_group.first, arm7dof_action_type);
                } break;
                case ARM6DOF: {
                    auto arm6dof_action_type = std::make_shared<ManipulationType>("../config/manip_6dof.mprim");
                    arm6dof_action_type->Discretization(ARM6DOF_DISCRETIZATION);
                    state_discretization_.insert(state_discretization_.end(), ARM6DOF_DISCRETIZATION.begin(), ARM6DOF_DISCRETIZATION.end());
                    sub_groups_action_types_.emplace_back(sub_group.first, arm6dof_action_type);
                } break;
                case TORSO: {
                    auto torso_action_type = std::make_shared<TorsoActionType>();
                    torso_action_type->Discretization(TORSO_DISCRETIZATION);
                    state_discretization_.insert(state_discretization_.end(), TORSO_DISCRETIZATION.begin(), TORSO_DISCRETIZATION.end());
                    sub_groups_action_types_.emplace_back(sub_group.first, torso_action_type);
                } break;
                default: {
                    ROS_ERROR_STREAM("The group type: " << sub_group.second << " is not supported.");
                }
            }
        }
    }

    ActionSequence getPrimActions() override {
        // get the actions for each sub group and combine them
        ActionSequence actions;
        for (auto &sub_group : sub_groups_action_types_) {
            auto sub_group_actions = sub_group.second->getPrimActions();
            actions.insert(actions.end(), sub_group_actions.begin(), sub_group_actions.end());
        }
        return actions;
    }

    ActionSequence getAdaptivePrimActions(const double &start_dist,
                                  const double &goal_dist) override {
        // get the actions for each sub group and combine them
        ActionSequence actions;
        int i = 0;
        for (auto &sub_group : sub_groups_action_types_) {
            auto sub_group_actions = sub_group.second->getAdaptivePrimActions(start_dist, goal_dist);
            for (auto &action : sub_group_actions) {
                Action action_type(state_discretization_.size(), 0);
                for (int j = 0; j < action.size(); j++) {
                    action_type[i + j] = action[j];
                }
                actions.push_back(action_type);
            }
            i += sub_group.second->state_discretization_.size();
        }
        return actions;
    }

    void Discretization(StateType &state_des) override {
        state_discretization_ = state_des;
    }

};


}  // namespace ims

#endif //ACTIONE_TYPES_HPP
