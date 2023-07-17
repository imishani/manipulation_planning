//
// Created by itamar on 4/13/23.
//

#ifndef MANIPULATION_PLANNING_MANIPHEURISTICS_HPP
#define MANIPULATION_PLANNING_MANIPHEURISTICS_HPP

// standard includes
#include <memory>
#include <utility>
#include <vector>

// moveit and ros includes
#include <moveit/distance_field/propagation_distance_field.h>

// search includes
#include <search/common/scene_interface.hpp>
#include <search/common/action_space.hpp>
#include <search/common/types.hpp>
#include <search/planners/dijkstra.hpp>
#include <search/planners/bfs3d.h>

// project includes
#include <manipulation_planning/common/utils.hpp>

namespace ims {

    class scene3Dpoint : public SceneInterface {
    public:
        explicit scene3Dpoint(std::shared_ptr<distance_field::PropagationDistanceField> &map_) : SceneInterface() {
            this->df_map = map_;
        }

        std::shared_ptr<distance_field::PropagationDistanceField> df_map;
    };

    struct actionType3Dpoint : public ActionType {

        actionType3Dpoint() : ActionType() {
            this->num_actions = 26;
            this->action_costs = std::vector<double>(num_actions, 1);
            this->action_deltas = std::vector<std::vector<double>>(num_actions, std::vector<double>(3, 0));
            int i{0};
            for (int x{-1}; x <= 1; x++) {
                for (int y{-1}; y <= 1; y++) {
                    for (int z{-1}; z <= 1; z++) {
                        if (x == 0 && y == 0 && z == 0) {
                            continue;
                        }
                        action_deltas[i][0] = x;
                        action_deltas[i][1] = y;
                        action_deltas[i][2] = z;

                        action_costs[i] = std::sqrt(x * x + y * y + z * z);
                        i++;
                    }
                }
            }

        }

        std::vector<Action> getActions() override {
            return this->action_deltas;
        }

        void Discretization(StateType &state_des) override {
            state_discretization_ = state_des;
        }

        int num_actions;
        std::vector<double> action_costs;
        std::vector<std::vector<double>> action_deltas;

    };

    class actionSpace3Dpoint : public ActionSpace {

    private:
        std::shared_ptr<scene3Dpoint> m_env;
        std::shared_ptr<actionType3Dpoint> m_actions;

    public:
        actionSpace3Dpoint(const scene3Dpoint &env,
                           const actionType3Dpoint &actions_ptr) : ActionSpace() {
            this->m_env = std::make_shared<scene3Dpoint>(env);
            this->m_actions = std::make_shared<actionType3Dpoint>(actions_ptr);
        }

        bool isStateValid(const StateType &state_val) override {
            return m_env->df_map->getCell(state_val[0], state_val[1], state_val[2]).distance_square_ > 0;
        }

        bool isPathValid(const PathType &path) override {
            return std::all_of(path.begin(), path.end(),
                               [this](const StateType &state_val) { return isStateValid(state_val); });
        }

        bool getSuccessors(int curr_state_ind,
                           std::vector<int> &successors,
                           std::vector<double> &costs) override {
            auto curr_state = this->getRobotState(curr_state_ind);
            auto curr_state_val = curr_state->state;
            auto actions = m_actions->getActions();
            for (int i{0}; i < m_actions->num_actions; i++) {
                auto action = actions[i];
                auto next_state_val = StateType(curr_state_val.size());
                std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(),
                               std::plus<>());
                // Check if state is outside the map
                if (next_state_val[0] < 0 || next_state_val[0] >= m_env->df_map->getXNumCells() ||
                    next_state_val[1] < 0 || next_state_val[1] >= m_env->df_map->getYNumCells() ||
                    next_state_val[2] < 0 || next_state_val[2] >= m_env->df_map->getZNumCells()) {
                    continue;
                }
                if (isStateValid(next_state_val)) {
                    int next_state_ind = getOrCreateRobotState(next_state_val);
                    successors.push_back(next_state_ind);
                    costs.push_back(m_actions->action_costs[i]);
                }
            }
            return true;
        }

        /// \brief Get state by value (if not exist, DO NOT create it)
        /// \param state_val The state value
        /// \param state_ind The state index to be returned
        /// \return True if the state exists, false otherwise
        bool getStateByValue(const StateType& state_val, size_t& state_ind) {
            // check if the state exists
            auto curr_state = new RobotState; curr_state->state = state_val;
            auto it = state_to_id_.find(curr_state);
            if(it == state_to_id_.end()){
                delete curr_state;
                return false;
            }
            state_ind = it->second;
            delete curr_state;
            return true;
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
//            m_distanceField = std::move(distance_field_);
//            // setup robot move group and planning scene
//            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
//            // robot model
//            robot_model = move_group->getRobotModel();
//            // joint model group
//            joint_model_group = robot_model->getJointModelGroup(group_name);
//
//            kinematic_state = std::make_shared<moveit::core::RobotState>(robot_model);
//
//            // get the planning group tip link
//            tip_link = joint_model_group->getLinkModelNames().back();
//        }
//
//        void setGoal(StateType& goal) override{
//            mGoal = goal;
//
//            kinematic_state->setJointGroupPositions(joint_model_group, goal);
//
//            auto end_effector_state = kinematic_state->getGlobalLinkTransform(tip_link);
//
//            auto goal_pose = end_effector_state.translation();
//            goal_position_ind.resize(3);
//            int x, y, z;
//            m_distanceField->worldToGrid(goal_pose.x(), goal_pose.y(), goal_pose.z(),
//                                         x, y, z);
//            goal_position_ind[0] = x; goal_position_ind[1] = y; goal_position_ind[2] = z;
//
//            run_planner();
//        }
//
//        bool run_planner(){
//            auto* zero_heuristic = new ZeroHeuristic();
//            dijkstraParams params (zero_heuristic);
//            scene3Dpoint scene(m_distanceField);
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
//                kinematic_state->setJointGroupPositions(joint_model_group, s1);
//                auto ee_s1_state = kinematic_state->getGlobalLinkTransform(tip_link);
//
//                kinematic_state->setJointGroupPositions(joint_model_group, s2);
//                auto ee_s2_state = kinematic_state->getGlobalLinkTransform(tip_link);
//
//                auto s1_position = ee_s1_state.translation();
//                auto s2_position = ee_s2_state.translation();
//
//                int x1, y1, z1, x2, y2, z2;
//                m_distanceField->worldToGrid(s1_position.x(), s1_position.y(), s1_position.z(),
//                                             x1, y1, z1);
//                m_distanceField->worldToGrid(s2_position.x(), s2_position.y(), s2_position.z(),
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
//                kinematic_state->setJointGroupPositions(joint_model_group, s_state);
//                auto ee_state = kinematic_state->getGlobalLinkTransform(tip_link);
//
//                auto s_position = ee_state.translation();
//                int x, y, z;
//                m_distanceField->worldToGrid(s_position.x(), s_position.y(), s_position.z(),
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
//        std::shared_ptr<distance_field::PropagationDistanceField> m_distanceField;
//        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
//
//        std::shared_ptr<actionSpace3Dpoint> action_space;
//
//        moveit::core::RobotModelConstPtr robot_model;
//        const moveit::core::JointModelGroup* joint_model_group;
//        moveit::core::RobotStatePtr kinematic_state;
//
//        StateType goal_position_ind;
//
//        dijkstra* dijkstra_planner = nullptr;
//
//        std::string tip_link;
//    };


    class BFSHeuristic : public BaseHeuristic{
    public:
        BFSHeuristic() : BFSHeuristic(getDistanceFieldMoveIt()){
        }

        explicit BFSHeuristic(std::shared_ptr<distance_field::PropagationDistanceField> distance_field_,
                              const std::string& group_name = "manipulator_1"){
            m_distanceField = std::move(distance_field_);
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
//            tip_link = names.at(5);

            syncGridAndBfs();
        }

        void setInflationRadius(double radius)
        {
            m_inflation_radius = radius;
        }

        void setCostPerCell(int cost_per_cell)
        {
            m_cost_per_cell = cost_per_cell;
        }

        void setGoal(StateType& goal) override{
            mGoal = goal;
            kinematic_state->setJointGroupPositions(joint_model_group, goal);
            auto ee_goal_state = kinematic_state->getGlobalLinkTransform(tip_link);

            auto goal_position = ee_goal_state.translation();
            int x, y, z;
            m_distanceField->worldToGrid(goal_position.x(), goal_position.y(), goal_position.z(),
                                         x, y, z);
            if (!m_bfs->inBounds(x, y, z))
                throw std::runtime_error("goal is out of bounds");

            m_goal_cells.emplace_back(x, y, z);

            m_bfs->run(x, y, z);
        }

        void setStart(StateType& start) override{
            mStart = start;
            kinematic_state->setJointGroupPositions(joint_model_group, start);
            auto ee_start_state = kinematic_state->getGlobalLinkTransform(tip_link);

            auto start_position = ee_start_state.translation();
            m_distanceField->worldToGrid(start_position.x(), start_position.y(), start_position.z(),
                                         start_cells[0], start_cells[1], start_cells[2]);
//            if (!m_bfs->inBounds(x, y, z))
//                throw std::runtime_error("start is out of bounds");
        }


        /// @brief Get the metric distance to the goal state
        /// @param x The x position of the state
        /// @param y The y position of the state
        /// @param z The z position of the state
        /// @return The distance to the goal state
        double getMetricGoalDistance(double x, double y, double z) const
        {
            int gx, gy, gz;
            m_distanceField->worldToGrid(x, y, z, gx, gy, gz);
            if (!m_bfs->inBounds(gx, gy, gz))
                return (double)smpl::BFS_3D::WALL * m_distanceField->getResolution();
            else
                return (double)m_bfs->getDistance(gx, gy, gz) * m_distanceField->getResolution();
        }

        /// @brief Get the metric distance to the start state
        /// @param x The x position of the state
        /// @param y The y position of the state
        /// @param z The z position of the state
        /// @return The distance to the start state
        double getMetricStartDistance(double x, double y, double z)
        {
            int sx, sy, sz;
            m_distanceField->worldToGrid(x, y, z, sx, sy, sz);
            // manhattan distance
            return (std::abs(sx - start_cells[0]) + std::abs(sy - start_cells[1]) + std::abs(sz - start_cells[2])) * m_distanceField->getResolution();
        }


        bool getHeuristic(StateType& s1, StateType& s2, double& dist) override{
            // check if s2 is a goal state
            kinematic_state->setJointGroupPositions(joint_model_group, s2);
            auto ee_check_state = kinematic_state->getGlobalLinkTransform(tip_link);

            auto check_position = ee_check_state.translation();
            int x, y, z;
            m_distanceField->worldToGrid(check_position.x(), check_position.y(), check_position.z(),
                                         x, y, z);
            // check if s2 is a goal state
            for (const auto& goal_cell : m_goal_cells)
            {
                if (goal_cell.x == x && goal_cell.y == y && goal_cell.z == z)
                {
                    return getHeuristic(s1, dist);
                }
            }

            std::cout << "getHeuristic between two random states with BFS is not supported!" << std::endl;
            return false;
        }

        bool getHeuristic(StateType& s, double& dist) override{
            if (m_goal_cells.empty())
                return false;

            kinematic_state->setJointGroupPositions(joint_model_group, s);
            auto ee_state = kinematic_state->getGlobalLinkTransform(tip_link);

            auto s_position = ee_state.translation();
            int x, y, z;
            m_distanceField->worldToGrid(s_position.x(), s_position.y(), s_position.z(),
                                         x, y, z);

            dist = getBfsCostToGoal(*m_bfs, x, y, z);
            return true;
        }


    private:
        std::shared_ptr<distance_field::PropagationDistanceField> m_distanceField;
        std::unique_ptr<smpl::BFS_3D> m_bfs;
        double m_inflation_radius = 0.02;
        int m_cost_per_cell = 100;
        int start_cells[3]{};

        struct CellCoord
        {
            int x, y, z;
            CellCoord() = default;
            CellCoord(int x, int y, int z) : x(x), y(y), z(z) { }
        };
        std::vector<CellCoord> m_goal_cells;

        void syncGridAndBfs(){
            const int xc = m_distanceField->getXNumCells();
            const int yc = m_distanceField->getYNumCells();
            const int zc = m_distanceField->getZNumCells();
            m_bfs = std::make_unique<smpl::BFS_3D>(xc, yc, zc);
            const int cell_count = xc * yc * zc;
            int wall_count = 0;
            for (int x = 0; x < xc; ++x) {
                for (int y = 0; y < yc; ++y) {
                    for (int z = 0; z < zc; ++z) {
                        const double radius = m_inflation_radius;
                        if (m_distanceField->getDistance(x, y, z) <= radius) {
                            m_bfs->setWall(x, y, z);
                            ++wall_count;
                        }
                    }
                }
            }

            std::cout << wall_count << " " << cell_count << " " << 100.0 * (double)wall_count / cell_count << " walls in the bfs heuristic" << std::endl;

        }

        int getBfsCostToGoal(const smpl::BFS_3D& bfs, int x, int y, int z) const{
            if (!bfs.inBounds(x, y, z) || bfs.getDistance(x, y, z) == smpl::BFS_3D::WALL)
                return Infinity;
            else {
                return m_cost_per_cell * bfs.getDistance(x, y, z);
            }
        }

        std::string tip_link;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        moveit::core::RobotModelConstPtr robot_model;
        const moveit::core::JointModelGroup* joint_model_group;
        moveit::core::RobotStatePtr kinematic_state;

    };

    /// @brief SE(3) distance heuristic using hopf coordinates
    struct SE3HeuristicHopf : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                // get the position of the states
                Eigen::Vector3d pos1 {s1[0], s1[1], s1[2]};
                Eigen::Vector3d pos2 {s2[0], s2[1], s2[2]};
                // get the orientation of the states
                Eigen::Quaterniond quat1; Eigen::Quaterniond quat2;
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
}


#endif //MANIPULATION_PLANNING_MANIPHEURISTICS_HPP
