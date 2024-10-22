//
// Created by itamar on 4/13/23.
//

#ifndef MANIPULATION_PLANNING_MANIPHEURISTICS_HPP
#define MANIPULATION_PLANNING_MANIPHEURISTICS_HPP

// standard includes
#include <memory>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>

// moveit and ros includes
#include <moveit/distance_field/propagation_distance_field.h>

// search includes
#include <manipulation_planning/common/smpl_grid/grid.h>
#include <search/common/intrusive_heap.h>
#include <search/planners/bfs3d.h>
#include <search/action_space/action_space.hpp>

#include <search/action_space/egraph_action_space.hpp>
#include <search/common/experience_graph.hpp>
#include <search/common/scene_interface.hpp>
#include <search/common/types.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/dijkstra.hpp>

// project includes
#include <manipulation_planning/common/utils.hpp>

namespace ims {

// Currently not compliant with void getPrimActions(std::vector<ActionSequence>& action_seqs, std::vector<std::vector<double>> & action_transition_costs ) = 0; Commented out for now.
//class scene3Dpoint : public SceneInterface {
//public:
//    explicit scene3Dpoint(std::shared_ptr<distance_field::PropagationDistanceField> &map_) : SceneInterface() {
//        this->df_map = map_;
//    }
//
//    std::shared_ptr<distance_field::PropagationDistanceField> df_map;
//};
//
//struct actionType3Dpoint : public ActionType {
//
//    actionType3Dpoint() : ActionType() {
//        this->num_actions = 26;
//        this->action_costs = std::vector<double>(num_actions, 1);
//        this->action_deltas = std::vector<std::vector<double>>(num_actions, std::vector<double>(3, 0));
//        int i{0};
//        for (int x{-1}; x <= 1; x++) {
//            for (int y{-1}; y <= 1; y++) {
//                for (int z{-1}; z <= 1; z++) {
//                    if (x == 0 && y == 0 && z == 0) {
//                        continue;
//                    }
//                    action_deltas[i][0] = x;
//                    action_deltas[i][1] = y;
//                    action_deltas[i][2] = z;
//
//                    action_costs[i] = std::sqrt(x * x + y * y + z * z);
//                    i++;
//                }
//            }
//        }
//
//    }
//
//    std::vector<Action> getPrimActions() override {
//        return this->action_deltas;
//    }
//
//    void Discretization(StateType &state_des) override {
//        state_discretization_ = state_des;
//    }
//
//    int num_actions;
//    std::vector<double> action_costs;
//    std::vector<std::vector<double>> action_deltas;
//
//};
//
//class actionSpace3Dpoint : public ActionSpace {
//
//private:
//    std::shared_ptr<scene3Dpoint> m_env;
//    std::shared_ptr<actionType3Dpoint> m_actions;
//
//public:
//    actionSpace3Dpoint(const scene3Dpoint &env,
//                        const actionType3Dpoint &actions_ptr) : ActionSpace() {
//        this->m_env = std::make_shared<scene3Dpoint>(env);
//        this->m_actions = std::make_shared<actionType3Dpoint>(actions_ptr);
//    }
//
//    void getActions(int state_id,
//                    std::vector<ActionSequence> &actions_seq,
//                    bool check_validity) override {
//        auto actions = m_actions->getPrimActions();
//        for (int i {0} ; i < m_actions->num_actions ; i++){
//            auto action = actions[i];
//            if (check_validity){
//                auto curr_state = this->getRobotState(state_id);
//                auto next_state_val = StateType(curr_state->state.size());
//                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
//                if (!isStateValid(next_state_val)){
//                    continue;
//                }
//            }
//            ActionSequence action_seq;
//            action_seq.push_back(action);
//            actions_seq.push_back(action_seq);
//        }
//    }
//
//    bool isStateValid(const StateType &state_val) override {
//        return m_env->df_map->getCell((int)state_val[0], (int)state_val[1], (int)state_val[2]).distance_square_ > 0;
//    }
//
//    bool isPathValid(const PathType &path) override {
//        return std::all_of(path.begin(), path.end(),
//                            [this](const StateType &state_val) { return isStateValid(state_val); });
//    }
//
//    bool getSuccessors(int curr_state_ind,
//                        std::vector<int> &successors,
//                        std::vector<double> &costs) override {
//        auto curr_state = this->getRobotState(curr_state_ind);
//        auto curr_state_val = curr_state->state;
//        std::vector<ActionSequence> actions;
//        getActions(curr_state_ind, actions, false);
//        for (int i {0} ; i < actions.size() ; i++){
//            auto action = actions[i][0];
//            auto next_state_val = StateType(curr_state_val.size());
//            std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(),
//                            std::plus<>());
//            // Check if state is outside the map
//            if (next_state_val[0] < 0 || next_state_val[0] >= m_env->df_map->getXNumCells() ||
//                next_state_val[1] < 0 || next_state_val[1] >= m_env->df_map->getYNumCells() ||
//                next_state_val[2] < 0 || next_state_val[2] >= m_env->df_map->getZNumCells()) {
//                continue;
//            }
//            if (isStateValid(next_state_val)) {
//                int next_state_ind = getOrCreateRobotState(next_state_val);
//                successors.push_back(next_state_ind);
//                costs.push_back(m_actions->action_costs[i]);
//            }
//        }
//        return true;
//    }
//
//    /// \brief Get state by value (if not exist, DO NOT create it)
//    /// \param state_val The state value
//    /// \param state_ind The state index to be returned
//    /// \return True if the state exists, false otherwise
//    bool getStateByValue(const StateType& state_val, size_t& state_ind) {
//        // check if the state exists
//        auto curr_state = new RobotState; curr_state->state = state_val;
//        auto it = state_to_id_.find(curr_state);
//        if(it == state_to_id_.end()){
//            delete curr_state;
//            return false;
//        }
//        state_ind = it->second;
//        delete curr_state;
//        return true;
//    }
//};


/// @brief SE(3) distance heuristic using hopf coordinates
struct SE3HeuristicHopf : public BaseHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            // get the position of the states
            Eigen::Vector3d pos1{s1[0], s1[1], s1[2]};
            Eigen::Vector3d pos2{s2[0], s2[1], s2[2]};
            // get the orientation of the states
            Eigen::Quaterniond quat1;
            Eigen::Quaterniond quat2;
            hopfToQuaternion(Eigen::Vector3d{s1[3], s1[4], s1[5]}, quat1);
            hopfToQuaternion(Eigen::Vector3d{s2[3], s2[4], s2[5]}, quat2);
            // get the distance between the positions
            dist = (pos1 - pos2).norm();
            // get the distance between the orientations
            dist += 2 * std::acos(std::min(1.0, std::abs(quat1.dot(quat2))));
            return true;
        }
    }
};

class BFSHeuristic : public BaseHeuristic {
protected:
    // The full pose of the goal EE. Orientation and translation.
    Eigen::Isometry3d ee_goal_state_;

public:
    BFSHeuristic() : BFSHeuristic(getDistanceFieldMoveIt()) {
    }

    explicit BFSHeuristic(std::shared_ptr<distance_field::PropagationDistanceField> distance_field,
                          const std::string& group_name = "manipulator_1") {
        distance_field_ = std::move(distance_field);
        // setup robot move group and planning scene
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
        // robot model
        robot_model = move_group->getRobotModel();
        // joint model group
        joint_model_group = robot_model->getJointModelGroup(group_name);

        kinematic_state = std::make_shared<moveit::core::RobotState>(robot_model);
        auto names = joint_model_group->getLinkModelNames();
        // get the planning group tip link
        tip_link = joint_model_group->getLinkModelNames().back();

        syncGridAndBfs();

        // Flag for whether the goal is set.
        is_goal_set = false;
    }

    void setInflationRadius(double radius) {
        inflation_radius_ = radius;
    }

    void setCostPerCell(int cost_per_cell) {
        cost_per_cell_ = cost_per_cell;
    }

    void setGoal(const StateType& goal) override {
        // The goal is specified in configuration space.
        goal_ = goal;
        
        // Compute the goal position in world space.
        kinematic_state->setJointGroupPositions(joint_model_group, goal);
        ee_goal_state_ = kinematic_state->getGlobalLinkTransform(tip_link);

        auto goal_position = ee_goal_state_.translation();
        int x, y, z;
        distance_field_->worldToGrid(goal_position.x(), goal_position.y(), goal_position.z(),
                                     x, y, z);
        if (!bfs_->inBounds(x, y, z))
            throw std::runtime_error("goal is out of bounds");

        m_goal_cells.emplace_back(x, y, z);

        bfs_->run(x, y, z); // TODO(yoraish): does this take other _robots_ into account too? Or only static obstacles.

        // Flag for whether the goal is set.
        is_goal_set = true;
    }

    void setStart(const StateType& start) override {
        start_ = start;
        kinematic_state->setJointGroupPositions(joint_model_group, start);
        auto ee_start_state = kinematic_state->getGlobalLinkTransform(tip_link);

        auto start_position = ee_start_state.translation();
        distance_field_->worldToGrid(start_position.x(), start_position.y(), start_position.z(),
                                     start_cells[0], start_cells[1], start_cells[2]);
        //            if (!bfs_->inBounds(x, y, z))
        //                throw std::runtime_error("start is out of bounds");
    }

    /// @brief Get the metric distance to the goal state
    /// @param x The x position of the state
    /// @param y The y position of the state
    /// @param z The z position of the state
    /// @return The distance to the goal state
    double getMetricGoalDistance(double x, double y, double z) const {
        int gx, gy, gz;
        distance_field_->worldToGrid(x, y, z, gx, gy, gz);
        if (!bfs_->inBounds(gx, gy, gz))
            return (double)::smpl::BFS_3D::WALL * distance_field_->getResolution();
        else
            return (double)bfs_->getDistance(gx, gy, gz) * distance_field_->getResolution();
    }

    /// @brief Get the metric distance to the start state
    /// @param x The x position of the state
    /// @param y The y position of the state
    /// @param z The z position of the state
    /// @return The distance to the start state
    double getMetricStartDistance(double x, double y, double z) {
        int sx, sy, sz;
        distance_field_->worldToGrid(x, y, z, sx, sy, sz);
        // manhattan distance
        return (std::abs(sx - start_cells[0]) + std::abs(sy - start_cells[1]) + std::abs(sz - start_cells[2])) * distance_field_->getResolution();
    }

    bool getHeuristic(const StateType& s1, const StateType& s2, double& dist) override {
        // check if s2 is a goal state
        kinematic_state->setJointGroupPositions(joint_model_group, s2);
        auto ee_check_state = kinematic_state->getGlobalLinkTransform(tip_link);

        auto check_position = ee_check_state.translation();
        int x, y, z;
        distance_field_->worldToGrid(check_position.x(), check_position.y(), check_position.z(),
                                     x, y, z);
        // check if s2 is a goal state
        for (const auto& goal_cell : m_goal_cells) {
            if (goal_cell.x == x && goal_cell.y == y && goal_cell.z == z) {
                return getHeuristic(s1, dist);
            }
        }

        std::cout << "getHeuristic between two random states with BFS is not supported!" << std::endl;
        return false;
    }

    bool getHeuristic(const StateType& s, double& dist) override {
        if (m_goal_cells.empty() || !is_goal_set) {
            std::cout << "Goal is not set in BFS heuristic! It must be set prior to calling getHeuristic" << std::endl;
            return false;
        }

        kinematic_state->setJointGroupPositions(joint_model_group, s);
        auto pose_ee = kinematic_state->getGlobalLinkTransform(tip_link);

        auto s_position = pose_ee.translation();
        int x, y, z;
        distance_field_->worldToGrid(s_position.x(), s_position.y(), s_position.z(),
                                     x, y, z);

        dist = getBfsCostToGoal(*bfs_, x, y, z);

        return true;
    }

    Eigen::Isometry3d getGoalPoseEE() const {
        return ee_goal_state_;
    }

    StateType getGoalPoseEExyzrpy() const{
        // Get the goal pose in a vector containing the xyz (meters) and rpy (radians) of the goal pose.
        StateType ee_goal_state_xyzrpy(6);
        ee_goal_state_xyzrpy[0] = ee_goal_state_.translation().x();
        ee_goal_state_xyzrpy[1] = ee_goal_state_.translation().y();
        ee_goal_state_xyzrpy[2] = ee_goal_state_.translation().z();
        // The stored Isometry3d uses zyx euler angles.
        ee_goal_state_xyzrpy[3] = ee_goal_state_.rotation().eulerAngles(2, 1, 0).z();
        ee_goal_state_xyzrpy[4] = ee_goal_state_.rotation().eulerAngles(2, 1, 0).y();
        ee_goal_state_xyzrpy[5] = ee_goal_state_.rotation().eulerAngles(2, 1, 0).x();
        
        return ee_goal_state_xyzrpy;
    }

private:
    std::shared_ptr<distance_field::PropagationDistanceField> distance_field_;
    std::unique_ptr<::smpl::BFS_3D> bfs_;
    double inflation_radius_ = 0.02;
    int cost_per_cell_ = 100;
    int start_cells[3]{};

    // Flag for whether the goal is set.
    bool is_goal_set = false;

    struct CellCoord {
        int x, y, z;
        CellCoord() = default;
        CellCoord(int x, int y, int z) : x(x), y(y), z(z) {}
    };
    std::vector<CellCoord> m_goal_cells;

    void syncGridAndBfs() {
        const int xc = distance_field_->getXNumCells();
        const int yc = distance_field_->getYNumCells();
        const int zc = distance_field_->getZNumCells();
        bfs_ = std::make_unique<::smpl::BFS_3D>(xc, yc, zc);
        const int cell_count = xc * yc * zc;
        int wall_count = 0;
        for (int x = 0; x < xc; ++x) {
            for (int y = 0; y < yc; ++y) {
                for (int z = 0; z < zc; ++z) {
                    const double radius = inflation_radius_;
                    if (distance_field_->getDistance(x, y, z) <= radius) {
                        bfs_->setWall(x, y, z);
                        ++wall_count;
                    }
                }
            }
        }

        std::cout << wall_count << " " << cell_count << " " << 100.0 * (double)wall_count / cell_count << " walls in the bfs heuristic" << std::endl;
    }

    int getBfsCostToGoal(const ::smpl::BFS_3D& bfs, int x, int y, int z) const {
        if (!bfs.inBounds(x, y, z) || bfs.getDistance(x, y, z) == ::smpl::BFS_3D::WALL)
            return INF_INT;
        else {
            return cost_per_cell_ * bfs.getDistance(x, y, z);
        }
    }

    std::string tip_link;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::core::RobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    moveit::core::RobotStatePtr kinematic_state;
};

/// @brief Same as the BFS Heuristic, but with states that include time in thier last element of the state vector. The methods here remove the time component and call the BFS heuristic methods as normal.
class BFSRemoveTimeHeuristic : public BFSHeuristic {
public:
    BFSRemoveTimeHeuristic() : BFSHeuristic() {
    }

    explicit BFSRemoveTimeHeuristic(std::shared_ptr<distance_field::PropagationDistanceField> distance_field,
                                    const std::string& group_name = "panda0_arm") : BFSHeuristic(distance_field, group_name) {
    }

    void setGoal(const StateType& goal) override {
        StateType goal_wo_time = goal;
        goal_wo_time.pop_back();
        BFSHeuristic::setGoal(goal_wo_time);
    }

    void setStart(const StateType& start) override {
        StateType start_wo_time = start;
        start_wo_time.pop_back();
        BFSHeuristic::setStart(start_wo_time);
    }

    bool getHeuristic(const StateType& s1, const StateType& s2, double& dist) override {
        StateType s1_wo_time = s1;
        s1_wo_time.pop_back();
        StateType s2_wo_time = s2;
        s2_wo_time.pop_back();
        return BFSHeuristic::getHeuristic(s1_wo_time, s2_wo_time, dist);
    }

    bool getHeuristic(const StateType& s, double& dist) override {
        StateType s_wo_time = s;
        s_wo_time.pop_back();
        return BFSHeuristic::getHeuristic(s_wo_time, dist);
    }
};

class BFSHeuristicEgraph : public EGraphHeuristicBase {
protected:
    // The full pose of the goal EE. Orientation and translation.
    Eigen::Isometry3d ee_goal_state_;

public:
    void init(std::shared_ptr<EGraphActionSpace> action_space,
              std::shared_ptr<distance_field::PropagationDistanceField> distance_field,
              const std::string& group_name = "manipulator_1") {
        action_space_ = std::move(action_space);

        distance_field_ = std::move(distance_field);
        // setup robot move group and planning scene
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
        // robot model
        robot_model_ = move_group_->getRobotModel();
        // joint model group
        joint_model_group_ = robot_model_->getJointModelGroup(group_name);

        kinematic_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        auto names = joint_model_group_->getLinkModelNames();
        // get the planning group tip link
        tip_link_ = joint_model_group_->getLinkModelNames().back();

        size_t num_cells_x = distance_field_->getXNumCells();
        size_t num_cells_y = distance_field_->getYNumCells();
        size_t num_cells_z = distance_field_->getZNumCells();
        double resolution = distance_field_->getResolution();

        dist_grid_.assign(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

        syncGridAndDijkstra();

        auto add_wall = [&](int x, int y, int z) {
            dist_grid_(x, y, z).dist = Wall;
        };

        // pad distance grid borders with walls
        for (int y = 0; y < num_cells_y; ++y) {
            for (int z = 0; z < num_cells_z; ++z) {
                add_wall(0, y, z);
                add_wall((int)num_cells_x - 1, y, z);
            }
        }
        for (int x = 1; x < num_cells_x - 1; ++x) {
            for (int z = 0; z < num_cells_z; ++z) {
                add_wall(x, 0, z);
                add_wall(x, (int)num_cells_y - 1, z);
            }
        }
        for (int x = 1; x < num_cells_x - 1; ++x) {
            for (int y = 1; y < (int)num_cells_y - 1; ++y) {
                add_wall(x, y, 0);
                add_wall(x, y, (int)num_cells_z - 1);
            }
        }
    }

    double weightEGraph() const {
        return weight_egraph_;
    }

    void setWeightEGraph(double weight) {
        weight_egraph_ = weight;
    }

    void setInflationRadius(double radius) {
        inflation_radius_ = radius;
    }

    void setCostPerCell(int cost_per_cell) {
        cost_per_cell_ = cost_per_cell;
    }

    void getEquivalentStates(int s_id, std::vector<int>& state_ids) override {
        // get the state
        auto state = action_space_->getRobotState(s_id);
        // get the position of the state
        // **WARNING**: Assuming the mapped state is task space
        Eigen::Vector3d pos{state->state_mapped[0], state->state_mapped[1], state->state_mapped[2]};
        // get the grid values of the position
        int x, y, z;
        distance_field_->worldToGrid(pos.x(), pos.y(), pos.z(), x, y, z);
        // check if in bounds
        if (((x < 0) || (y < 0) || (z < 0) ||
             (x >= (distance_field_->getXNumCells() - 2)) ||
             (y >= (distance_field_->getYNumCells() - 2)) ||
             (z >= (distance_field_->getZNumCells() - 2)))) {
            return;
        }
        ++x;
        ++y;
        ++z;

        auto hit = heur_nodes_.find({x, y, z});
        if (hit == heur_nodes_.end()) {
            return;
        }
        for (smpl::ExperienceGraph::node_id n : hit->second.up_nodes) {
            int id = action_space_->getStateID(n);
            if (id != s_id) {
                state_ids.push_back(id);
            }
        }
    }

    void getShortcutSuccessors(int s_id, std::vector<int>& state_ids) override {
        std::vector<smpl::ExperienceGraph::node_id> egraph_nodes;
        action_space_->getEGraphNodes(s_id, egraph_nodes);
        for (auto node : egraph_nodes) {
            auto comp_id = component_ids_[node];
            for (auto shortcut_node : shortcut_nodes_[comp_id]) {
                int id = action_space_->getStateID(shortcut_node);
                if (id != s_id) {
                    state_ids.push_back(id);
                }
            }
        }
    }

    /// @brief Get the metric distance to the goal state
    /// @param x The x position of the state
    /// @param y The y position of the state
    /// @param z The z position of the state
    /// @return The distance to the goal state
    double getMetricGoalDistance(double x, double y, double z) const {
        int gx, gy, gz;
        distance_field_->worldToGrid(x, y, z, gx, gy, gz);
        // take goal and check its grid value
        int goal_x, goal_y, goal_z;
        distance_field_->worldToGrid(goal_ws_[0], goal_ws_[1], goal_ws_[2], goal_x, goal_y, goal_z);
        int dx = gx - goal_x;
        int dy = gy - goal_y;
        int dz = gz - goal_z;

        return distance_field_->getResolution() * (abs(dx) + abs(dy) + abs(dz));
    }

    /// @brief Get the metric distance to the start state
    /// @param x The x position of the state
    /// @param y The y position of the state
    /// @param z The z position of the state
    /// @return The distance to the start state
    double getMetricStartDistance(double x, double y, double z) {
        return 0;
    }

    void setGoal(const StateType& goal) override{
        //////////* The assumption here is that the goal is in configuration spae //////////*/
        //////////* TODO: Fix this assumption in the future ////////////////////////////////*/
        kinematic_state_->setJointGroupPositions(joint_model_group_, goal);
        ee_goal_state_ = kinematic_state_->getGlobalLinkTransform(tip_link_);

        auto goal_position = ee_goal_state_.translation();
        goal_ = goal;
        goal_ws_ = {goal_position.x(), goal_position.y(), goal_position.z()};
        int x_goal, y_goal, z_goal;
        distance_field_->worldToGrid(goal_position.x(), goal_position.y(), goal_position.z(),
                                        x_goal, y_goal, z_goal);

        projectExperienceGraph();

        // precompute shortcuts
        assert(component_ids_.size() == action_space_->getExperienceGraph()->num_nodes());
        auto egraph = action_space_->getExperienceGraph();
        auto nodes = egraph->nodes();
        for (auto nit {nodes.first} ; nit != nodes.second; ++nit) {
            auto comp_id = component_ids_[*nit];
            if (shortcut_nodes_[comp_id].empty()) {
                shortcut_nodes_[comp_id].push_back(*nit);
                continue;
            }

            Eigen::Vector3d p;
            auto state = action_space_->getRobotState(*nit);
            p.x() = state->state_mapped[0]; p.y() = state->state_mapped[1]; p.z() = state->state_mapped[2];
            auto dist = (goal_position - p).squaredNorm();
            state = action_space_->getRobotState(shortcut_nodes_[comp_id].front());
            Eigen::Vector3d lp;
            lp.x() = state->state_mapped[0]; lp.y() = state->state_mapped[1]; lp.z() = state->state_mapped[2];
            auto curr_dist = (goal_position - lp).squaredNorm();

            if (dist < curr_dist) {
                shortcut_nodes_[comp_id].clear();
                shortcut_nodes_[comp_id].push_back(*nit);
            } else if (dist == curr_dist) {
                shortcut_nodes_[comp_id].push_back(*nit);
            }
        }
        // check if goal in bounds
        if ((x_goal < 0) || (y_goal < 0) || (z_goal < 0) ||
            (x_goal >= (distance_field_->getXNumCells() - 2)) ||
            (y_goal >= (distance_field_->getYNumCells() - 2)) ||
            (z_goal >= (distance_field_->getZNumCells() - 2))) {
            std::cout << RED << "Cell (" << x_goal << ", " << y_goal << ", " << z_goal << ") is out of bounds" << RESET << std::endl;
            return;
        }

        x_goal++; y_goal++; z_goal++;
        open_heur_.clear();
        auto* c = &dist_grid_(x_goal, y_goal, z_goal);
        c->dist = 0;
        open_heur_.push(c);
    }


    void setStart(const StateType& start) override {
        start_ = start;
    }

    bool getHeuristic(const StateType& s1, const StateType& s2, double& dist) override {
        std::cout << RED << "[ERROR] getHeuristic between two random states with ExperienceDijkstraHeuristic is not supported!" << std::endl;
        return false;
    }

    bool getHeuristic(const StateType& s, double& dist) override {
        // Assuming s is in configuration space
        // get the position of the state
        kinematic_state_->setJointGroupPositions(joint_model_group_, s);
        auto ee_state = kinematic_state_->getGlobalLinkTransform(tip_link_);

        auto s_position = ee_state.translation();
        int x, y, z;
        distance_field_->worldToGrid(s_position.x(), s_position.y(), s_position.z(),
                                     x, y, z);
        // check if in bounds
        if (x < 0 || y < 0 || z < 0 ||
            (x >= (distance_field_->getXNumCells() - 2)) ||
            (y >= (distance_field_->getYNumCells() - 2)) ||
            (z >= (distance_field_->getZNumCells() - 2))) {
            std::cout << RED << "Cell (" << x << ", " << y << ", " << z << ") is out of bounds" << RESET << std::endl;
            dist = INF_DOUBLE;
            return false;
        }
        ++x;
        ++y;
        ++z;

        Cell* cell = &dist_grid_(x, y, z);
        if (cell->dist == Wall) {
            dist = INF_DOUBLE;
            return true;
        }

        static int last_expand_count{0};
        int expand_count{0};
        static int repeat_count{1};
        while (cell->dist == Unknown && !open_heur_.empty()) {
            ++expand_count;
            Cell* curr_cell = open_heur_.min();
            open_heur_.pop();

            int cidx = (int)std::distance(dist_grid_.data(), curr_cell);
            size_t cx, cy, cz;
            dist_grid_.index_to_coord(cidx, cx, cy, cz);

            // relax experience graph adjacency edges
            auto it = heur_nodes_.find(Eigen::Vector3i(cx, cy, cz));
            if (it != end(heur_nodes_)) {
                auto& hnode = it->second;
                for (auto& adj : hnode.edges) {
                    auto dx = adj.x() - cx;
                    auto dy = adj.y() - cy;
                    auto dz = adj.z() - cz;
                    auto* ncell = &dist_grid_(adj.x(), adj.y(), adj.z());

                    auto cost = (int)(1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
                    auto new_cost = curr_cell->dist + cost;
                    if (new_cost < ncell->dist) {
                        ncell->dist = new_cost;
                        if (open_heur_.contains(ncell)) {
                            open_heur_.decrease(ncell);
                        }
                        else {
                            open_heur_.push(ncell);
                        }
                    }
                }
            }
            // relax neighboring edges
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 & dy == 0 & dz == 0) {
                            continue;
                        }

                        auto sx = cx + dx;
                        auto sy = cy + dy;
                        auto sz = cz + dz;

                        auto* ncell = &dist_grid_(sx, sy, sz);

                        // bounds and obstacle check
                        if (ncell->dist == Wall) {
                            continue;
                        }

                        auto cost = (int)(weight_egraph_ * 1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
                        auto new_cost = curr_cell->dist + cost;

                        if (new_cost < ncell->dist) {
                            ncell->dist = new_cost;
                            if (open_heur_.contains(ncell)) {
                                open_heur_.decrease(ncell);
                            }
                            else {
                                open_heur_.push(ncell);
                            }
                        }
                    }
                }
            }
        }
        if (last_expand_count != expand_count) {
            last_expand_count = expand_count;
            repeat_count = 1;
        }
        else {
            ++repeat_count;
        }

        if (cell->dist > INF_DOUBLE) {
            dist = INF_DOUBLE;
            return true;
        }
        dist = cell->dist;
        return true;
    }

private:
    static const int Unknown = std::numeric_limits<int>::max() >> 1;
    static const int Wall = std::numeric_limits<int>::max();
    //        static const int Infinity = Unknown;

    StateType goal_ws_;
    std::string tip_link_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;
    moveit::core::RobotStatePtr kinematic_state_;

    std::shared_ptr<EGraphActionSpace> action_space_ = nullptr;

    std::shared_ptr<distance_field::PropagationDistanceField> distance_field_;

    struct Cell : public ::smpl::HeapElement {
        int dist{};

        Cell() = default;
        explicit Cell(int d) : ::smpl::HeapElement(), dist(d) {}
    };

    ::smpl::Grid3<Cell> dist_grid_;

    struct CellCompare {
        bool operator()(const Cell& a, const Cell& b) const {
            return a.dist < b.dist;
        }
    };

    double inflation_radius_ = 0.02;
    int cost_per_cell_ = 100;
    double weight_egraph_ = 1.0;

    ::smpl::IntrusiveHeap<Cell, CellCompare> open_heur_;

    // map down-projected state cells to adjacent down-projected state cells
    struct Vector3iHash {
        typedef Eigen::Vector3i argument_type;
        typedef std::size_t result_type;

        result_type operator()(const argument_type& s) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, std::hash<int>()(s.x()));
            boost::hash_combine(seed, std::hash<int>()(s.y()));
            boost::hash_combine(seed, std::hash<int>()(s.z()));
            return seed;
        }
    };

    // map from experience graph nodes to their heuristic cell coordinates
    std::vector<Eigen::Vector3i> projected_nodes_;

    // map from experience graph nodes to their component ids
    std::vector<int> component_ids_;
    std::vector<std::vector<smpl::ExperienceGraph::node_id>> shortcut_nodes_;

    struct HeuristicNode {
        std::vector<smpl::ExperienceGraph::node_id> up_nodes;

        std::vector<Eigen::Vector3i> edges;
    };

    hash_map<Eigen::Vector3i, HeuristicNode, Vector3iHash> heur_nodes_;

    /// @brief Project experience graph states down to their 3D projections. Note that this
    /// should be called once the goal is set in the environment as the projection to
    /// 3D may be based off of the goal condition (for instance, the desired planning
    /// frame is determined according to the planning link and a fixed offset)
    void projectExperienceGraph() {
        heur_nodes_.clear();
        // project experience graph into 3d space (projections of adjacent nodes in
        // the experience graph impose additional edges in 3d, cost equal to the
        // cheapest transition):
        //
        // (1) lookup transitions on-demand when a grid cell is expanded, loop
        // through all experience graph states and their neighbors (method used by
        // origin experience graph code)
        //
        // (2) embed an adjacency list in the dense grid structure as a
        // precomputation
        //
        // (3) maintain an external adjacency list mapping cells with projections
        // from experience graph states to adjacent cells (method used here)

        // get the experience graph
        auto egraph = action_space_->getExperienceGraph();
        if (!egraph) {
            throw std::runtime_error("Experience graph is not initialized");
        }

        projected_nodes_.resize(egraph->num_nodes());

        int proj_node_count{0};
        int proj_edge_count{0};
        auto nodes = egraph->nodes();
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            int first_id = action_space_->getStateID(*nit);
            auto first_state = action_space_->getRobotState(first_id);
            // get the position of the state
            // **WARNING**: Assuming the mapped state is task space
            Eigen::Vector3d pos{first_state->state_mapped[0],
                                first_state->state_mapped[1],
                                first_state->state_mapped[2]};
            // get the grid values of the position
            int x, y, z;
            distance_field_->worldToGrid(pos.x(), pos.y(), pos.z(), x, y, z);

            ++x;
            ++y;
            ++z;
            projected_nodes_[*nit] = Eigen::Vector3i{x, y, z};

            // insert node into down-projected experience graph
            HeuristicNode empty;
            auto ent = heur_nodes_.insert(std::make_pair(Eigen::Vector3i{x, y, z}, empty));
            if (ent.second) {
                ++proj_node_count;
            }

            auto eit = ent.first;
            auto& hnode = eit->second;
            hnode.up_nodes.push_back(*nit);

            auto adj = egraph->adjacent_nodes(*nit);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                int second_id = action_space_->getStateID(*ait);
                auto second_state = action_space_->getRobotState(second_id);
                // get the position of the state
                // **WARNING**: Assuming the mapped state is task space
                Eigen::Vector3d pos_2{second_state->state_mapped[0], second_state->state_mapped[1], second_state->state_mapped[2]};
                // get the grid values of the position
                int x_2, y_2, z_2;
                distance_field_->worldToGrid(pos.x(), pos.y(), pos.z(), x_2, y_2, z_2);
                // check if the second state is in bounds
                if ((x_2 < 0 || y_2 < 0 || z_2 < 0 ||
                     (x_2 >= (distance_field_->getXNumCells() - 2)) ||
                     (y_2 >= (distance_field_->getYNumCells() - 2)) ||
                     (z_2 >= (distance_field_->getZNumCells() - 2)))) {
                    continue;
                }
                ++x_2;
                ++y_2;
                ++z_2;

                // insert adjacent node
                if (std::find(hnode.edges.begin(), hnode.edges.end(), Eigen::Vector3i{x_2, y_2, z_2}) == hnode.edges.end()) {
                    hnode.edges.emplace_back(x_2, y_2, z_2);
                    ++proj_edge_count;
                }
            }
        }
        std::cout << YELLOW << "Projected " << proj_node_count << " nodes and " << proj_edge_count << " edges" << RESET << std::endl;

        auto comp_count{0};
        component_ids_.assign(egraph->num_nodes(), -1);
        for (auto nit{nodes.first}; nit != nodes.second; ++nit) {
            if (component_ids_[*nit] != -1)
                continue;

            std::vector<smpl::ExperienceGraph::node_id> frontier;
            frontier.push_back(*nit);
            while (!frontier.empty()) {
                auto n = frontier.back();
                frontier.pop_back();
                component_ids_[n] = comp_count;
                auto adj = egraph->adjacent_nodes(n);
                for (auto ait = adj.first; ait != adj.second; ++ait) {
                    if (component_ids_[*ait] == -1) {
                        frontier.push_back(*ait);
                    }
                }
            }
            ++comp_count;
        }
        // pre-allocate shortcuts array here, fill in updateGoal()
        shortcut_nodes_.assign(comp_count, std::vector<smpl::ExperienceGraph::node_id>());
        std::cout << YELLOW << "Experience graph has " << comp_count << " components" << RESET << std::endl;
    }

    void syncGridAndDijkstra() {
        const int xc = distance_field_->getXNumCells();
        const int yc = distance_field_->getYNumCells();
        const int zc = distance_field_->getZNumCells();

        auto cell_count = xc * yc * zc;
        int wall_count = 0;

        for (int x = 0; x < xc; ++x) {
            for (int y = 0; y < yc; ++y) {
                for (int z = 0; z < zc; ++z) {
                    auto radius = inflation_radius_;
                    if (distance_field_->getDistance(x, y, z) <= radius) {
                        dist_grid_(x + 1, y + 1, z + 1).dist = Wall;
                        ++wall_count;
                    }
                }
            }
        }
        std::cout << wall_count << " " << cell_count << " " << 100.0 * (double)wall_count / cell_count << " walls in the bfs heuristic" << std::endl;
    }
};

/// TODO: Fix this?
//    class DijkstraHeuristic : public BaseHeuristic{
//    public:
//        DijkstraHeuristic() : DijkstraHeuristic(getDistanceFieldMoveIt()){
//        }
//
//        explicit DijkstraHeuristic(std::shared_ptr<distance_field::PropagationDistanceField> distance_field_,
//                     const std::string& group_name = "manipulator_1"){
//            distance_field_ = std::move(distance_field_);
//            // setup robot move group and planning scene
//            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
//            // robot model
//            robot_model_ = move_group_->getRobotModel();
//            // joint model group
//            joint_model_group_ = robot_model_->getJointModelGroup(group_name);
//
//            kinematic_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
//
//            // get the planning group tip link
//            tip_link_ = joint_model_group_->getLinkModelNames().back();
//        }
//
//        void setGoal(StateType& goal) override{
//            mGoal = goal;
//
//            kinematic_state_->setJointGroupPositions(joint_model_group_, goal);
//
//            auto end_effector_state = kinematic_state_->getGlobalLinkTransform(tip_link_);
//
//            auto goal_pose = end_effector_state.translation();
//            goal_position_ind.resize(3);
//            int x, y, z;
//            distance_field_->worldToGrid(goal_pose.x(), goal_pose.y(), goal_pose.z(),
//                                         x, y, z);
//            goal_position_ind[0] = x; goal_position_ind[1] = y; goal_position_ind[2] = z;
//
//            run_planner();
//        }
//
//        bool run_planner(){
//            auto* zero_heuristic = new ZeroHeuristic();
//            dijkstraParams params (zero_heuristic);
//            scene3Dpoint scene(distance_field_);
//            actionType3Dpoint actions;
//            action_space = std::make_shared<actionSpace3Dpoint>(scene, actions);
//            dijkstra_planner = new dijkstra(params);
//
//            try {
//                dijkstra_planner->initializePlanner(action_space, goal_position_ind, goal_position_ind);
//            }
//            catch (std::exception& e){
//                std::cout << e.what() << std::endl;
//                throw std::runtime_error("failed to initialize planner");
//            }
//
//            if (!dijkstra_planner->exhaustPlan())
//                throw std::runtime_error("failed to look in the entire space");
//        }
//
//        bool getHeuristic(StateType& s1, StateType& s2, double& dist) override{
//                // Do forward kinematics to get the goal pose
//                kinematic_state_->setJointGroupPositions(joint_model_group_, s1);
//                auto ee_s1_state = kinematic_state_->getGlobalLinkTransform(tip_link_);
//
//                kinematic_state_->setJointGroupPositions(joint_model_group_, s2);
//                auto ee_s2_state = kinematic_state_->getGlobalLinkTransform(tip_link_);
//
//                auto s1_position = ee_s1_state.translation();
//                auto s2_position = ee_s2_state.translation();
//
//                int x1, y1, z1, x2, y2, z2;
//                distance_field_->worldToGrid(s1_position.x(), s1_position.y(), s1_position.z(),
//                                             x1, y1, z1);
//                distance_field_->worldToGrid(s2_position.x(), s2_position.y(), s2_position.z(),
//                                                x2, y2, z2);
//
//                StateType s1_pos {(double)x1, (double)y1, (double)z1};
//                StateType s2_pos {(double)x2, (double)y2, (double)z2};
//
//                size_t s1_ind_int, s2_ind_int;
//                if (!action_space->getStateByValue(s1_pos, s1_ind_int))
//                    return false;
//                if (!action_space->getStateByValue(s2_pos, s2_ind_int))
//                    return false;
//                else{
//                    dist = action_space->getState(s1_ind_int)->f - action_space->getState(s2_ind_int)->f;
//                    return true;
//                }
//            }
//
//        bool getHeuristic(State* s, double& dist) override{
//            if (goal_position_ind.empty())
//                return false;
//            else {
//                const auto& s_state = s->getState();
//                kinematic_state_->setJointGroupPositions(joint_model_group_, s_state);
//                auto ee_state = kinematic_state_->getGlobalLinkTransform(tip_link_);
//
//                auto s_position = ee_state.translation();
//                int x, y, z;
//                distance_field_->worldToGrid(s_position.x(), s_position.y(), s_position.z(),
//                                             x, y, z);
//
//                StateType s_pos {(double)x, (double)y, (double)z};
//                size_t s_ind_int;
//                if (!action_space->getStateByValue(s_pos, s_ind_int))
//                    return false;
//                else {
//                    dist = action_space->getState(s_ind_int)->f;
//                    return true;
//                }
//
//
//            }
//        }
//
//        std::shared_ptr<distance_field::PropagationDistanceField> distance_field_;
//        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//
//        std::shared_ptr<actionSpace3Dpoint> action_space;
//
//        moveit::core::RobotModelConstPtr robot_model_;
//        const moveit::core::JointModelGroup* joint_model_group_;
//        moveit::core::RobotStatePtr kinematic_state_;
//
//        StateType goal_position_ind;
//
//        dijkstra* dijkstra_planner = nullptr;
//
//        std::string tip_link_;
//    };
}  // namespace ims

#endif  // MANIPULATION_PLANNING_MANIPHEURISTICS_HPP
