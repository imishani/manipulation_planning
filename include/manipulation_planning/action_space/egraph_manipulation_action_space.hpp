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
 * \file   egraph_manipulation_action_space.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/11/23
 */

#ifndef MANIPULATION_PLANNING_EGRAPH_MANIPULATION_ACTION_SPACE_HPP
#define MANIPULATION_PLANNING_EGRAPH_MANIPULATION_ACTION_SPACE_HPP

#include "manipulation_action_space.hpp"

namespace ims {

    /// @class ManipulationActionSpace
    /// @brief A class that implements the ActionSpace for Moveit
    class EgraphManipulationActionSpace : public EGraphActionSpace {

    protected:
        /// @brief Manipulation type
        std::shared_ptr<ManipulationType> manipulation_type_;
        /// @brief Moveit interface
        std::shared_ptr<MoveitInterface> moveit_interface_;
        /// @brief joint limits
        std::vector<std::pair<double, double>> joint_limits_;
        /// @brief Joint states seed
        //    std::vector<double> mJointStatesSeed {0, 0, 0, 0, 0, 0};
        /// @brief The BFS heuristic
        BFSHeuristicEgraph* bfs_heuristic_;

        // TODO: delete: temp
        int vis_id_ = 0;
        ros::NodeHandle nh_;
        ros::Publisher vis_pub_;


    public:

        /// @brief Constructor
        /// @param moveitInterface The moveit interface
        /// @param ManipulationType The manipulation type
        EgraphManipulationActionSpace(const MoveitInterface &env,
                                      const ManipulationType &actions_ptr,
                                      BFSHeuristicEgraph* bfs_heuristic = nullptr) : EGraphActionSpace(),
                                                                                     bfs_heuristic_(bfs_heuristic) {
            moveit_interface_ = std::make_shared<MoveitInterface>(env);
            manipulation_type_ = std::make_shared<ManipulationType>(actions_ptr);
            // get the joint limits
            moveit_interface_->getJointLimits(joint_limits_);
            vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

        }

        void getActions(int state_id,
                        std::vector<ActionSequence> &actions_seq,
                        bool check_validity) override {
            auto curr_state = this->getRobotState(state_id);
            auto curr_state_val = curr_state->state;
            if (bfs_heuristic_ == nullptr){
                auto actions = manipulation_type_->getPrimActions();
                for (int i {0} ; i < actions.size() ; i++){
                    auto action = actions[i];
                    ActionSequence action_seq {curr_state_val};
                    // push back the new state after the action
                    StateType next_state_val(curr_state_val.size());
                    std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(), std::plus<>());
                    actions_seq.push_back(action_seq);
                }
            } else {
                if (curr_state->state_mapped.empty()){
                    moveit_interface_->calculateFK(curr_state_val, curr_state->state_mapped);
                }
//                std::vector<double> ws_state_check(6);
//                moveit_interface_->calculateFK(curr_state_val, ws_state_check);
                VisualizePoint(curr_state->state_mapped.at(0), curr_state->state_mapped.at(1), curr_state->state_mapped.at(2));
//                std::cout << BOLDCYAN << "Visualized state: " << state_id << RESET << std::endl;
                auto goal_dist = bfs_heuristic_->getMetricGoalDistance(curr_state->state_mapped.at(0),
                                                                       curr_state->state_mapped.at(1),
                                                                       curr_state->state_mapped.at(2));
                auto start_dist = bfs_heuristic_->getMetricStartDistance(curr_state->state_mapped.at(0),
                                                                         curr_state->state_mapped.at(1),
                                                                         curr_state->state_mapped.at(2));
                auto actions = manipulation_type_->getAdaptiveActions(start_dist, goal_dist);

                for (int i {0} ; i < actions.size() ; i++){
                    auto action = actions[i];
                    ActionSequence action_seq {curr_state_val};
                    // if the action is snap, then the next state is the goal state
                    // TODO: Add the option to have a goal state defined in ws even if planning in conf space
                    if (action[0] == INF_DOUBLE){
                        action_seq.push_back(bfs_heuristic_->goal_); // TODO: It is wierd that I am using the heuristic here
                    }
                    else {
                        // push back the new state after the action
                        StateType next_state_val(curr_state_val.size());
                        std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(), std::plus<>());
                        action_seq.push_back(next_state_val);
                    }
                    actions_seq.push_back(action_seq);
                }
            }
        }

        /// @brief Set the manipulation space type
        /// @param SpaceType The manipulation type
        void setManipActionType(ManipulationType::SpaceType SpaceType)
        {
            manipulation_type_->setSpaceType(SpaceType);
        }

        ManipulationType::SpaceType getManipActionType()
        {
            return manipulation_type_->getSpaceType();
        }

        /// @brief Get current joint states
        /// @param joint_states The joint states
        void getCurrJointStates(StateType &joint_states)
        {
            auto joints = moveit_interface_->planning_scene_->getCurrentState();
            joints.copyJointGroupPositions(moveit_interface_->group_name_,
                                           joint_states);
        }

        /// @brief Get the workspace state
        /// @param ws_state The workspace state
        void getCurrWorkspaceState(StateType &ws_state)
        {
            // get the tip link name
            auto tip_link = moveit_interface_->planning_scene_->getRobotModel()->getJointModelGroup(moveit_interface_->group_name_)->getLinkModelNames().back();
            // get the end-effector pose
            auto ee_pose = moveit_interface_->planning_scene_->getCurrentState().getGlobalLinkTransform(tip_link);
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
            roundStateToDiscretization(ws_state, manipulation_type_->state_discretization_);
        }

        bool isStateValid(const StateType &state_val) override
        {
            // check if the state is valid
            switch (manipulation_type_->getSpaceType())
            {
                case ManipulationType::SpaceType::ConfigurationSpace:
                    return moveit_interface_->isStateValid(state_val);
                case ManipulationType::SpaceType::WorkSpace:
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
                    bool succ = moveit_interface_->calculateIK(pose, joint_state);
                    if (!succ)
                    {
                        return false;
                    }
                    else
                    {
                        return moveit_interface_->isStateValid(joint_state);
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
            switch (manipulation_type_->getSpaceType())
            {
                case ManipulationType::SpaceType::ConfigurationSpace:
                    return moveit_interface_->isStateValid(state_val);
                case ManipulationType::SpaceType::WorkSpace:
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
                    bool succ = moveit_interface_->calculateIK(pose, joint_state);
                    if (!succ)
                    {
                        ROS_INFO("IK failed");
                        return false;
                    }
                    else
                    {
                        return moveit_interface_->isStateValid(joint_state);
                    }
            }
            return false;
        }

        bool isStateValid(const StateType &state_val,
                          const StateType &seed,
                          StateType &joint_state)
        {
            // check if the state is valid
            switch (manipulation_type_->getSpaceType())
            {
                case ManipulationType::SpaceType::ConfigurationSpace:
                    return moveit_interface_->isStateValid(state_val);
                case ManipulationType::SpaceType::WorkSpace:
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
                    joint_state.resize(moveit_interface_->num_joints_);
                    bool succ = moveit_interface_->calculateIK(pose, seed, joint_state);
                    normalizeAngles(joint_state);
                    if (!succ)
                    {
                        return false;
                    }
                    else
                    {
                        return moveit_interface_->isStateValid(joint_state);
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
                                        const double resolution = 0.2)
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
            switch (manipulation_type_->getSpaceType())
            {
                case ManipulationType::SpaceType::ConfigurationSpace:
                    return moveit_interface_->isPathValid(path);
                case ManipulationType::SpaceType::WorkSpace:
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
                        bool succ = moveit_interface_->calculateIK(pose, joint_state);
                        if (!succ)
                        {
                            return false;
                        }
                        else
                        {
                            poses.push_back(joint_state);
                        }
                    }
                    return moveit_interface_->isPathValid(poses);
            }
            return false;
        }

        virtual bool getSuccessorSequencesWs(int curr_state_ind,
                                    std::vector<std::vector<int>>& seqs_state_ids,
                                    std::vector<std::vector<double>> & seqs_transition_costs) {
            seqs_state_ids.clear();
            seqs_transition_costs.clear();
            // REMOVE.
            std::vector<int> successors;
            std::vector<double> costs;
            // END REMOVE.

            // get the current state
            auto curr_state = this->getRobotState(curr_state_ind);
            auto curr_state_val = curr_state->state;
            // get the actions
            auto actions = manipulation_type_->getPrimActions();
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
                roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);

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

            // REMOVE.
            for (int i{0}; i < successors.size(); i++) {
                seqs_state_ids.push_back({curr_state_ind, successors[i]});
                seqs_transition_costs.push_back({costs[i], 0});
            }
            // END REMOVE.

            return true;
        }

            virtual bool getSuccessorSequencesCs(int curr_state_ind,
                                   std::vector<std::vector<int>>& seqs_state_ids,
                                   std::vector<std::vector<double>> & seqs_transition_costs) {
            seqs_state_ids.clear();
            seqs_transition_costs.clear();
            // REMOVE.
            std::vector<int> successors;
            std::vector<double> costs;
            // END REMOVE.

            std::vector<ActionSequence> actions;
            getActions(curr_state_ind, actions, false);
            // get the successors
            for (auto &action : actions)
            {
                // the first state is the current state and the last state is the successor
                auto curr_state_val = action.front();
                auto new_state_val = action.back();
                // normalize the angles
                normalizeAngles(new_state_val, joint_limits_);
                // discretize the state
                roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
                // check if the state went through discontinuity
                bool discontinuity{false};
                // check for maximum absolute action
                for (int i{0}; i < curr_state_val.size(); i++)
                {
                    if (new_state_val[i] < joint_limits_[i].first || new_state_val[i] > joint_limits_[i].second)
                    {
                        discontinuity = true;
                        break;
                    }
                }

                if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val))
                {
                    // create a new state
                    int next_state_ind = getOrCreateRobotState(new_state_val);
                    // add the FK of the state
                    auto new_state = getRobotState(next_state_ind);
                    moveit_interface_->calculateFK(new_state->state, new_state->state_mapped);
                    // add the state to the successors
                    successors.push_back(next_state_ind);
                    // add the cost
                    costs.push_back(1000);
                }
            }

            // REMOVE.
            for (int i{0}; i < successors.size(); i++) {
                seqs_state_ids.push_back({curr_state_ind, successors[i]});
                seqs_transition_costs.push_back({costs[i], 0});
            }
            // END REMOVE.

            return true;
        }

        bool getSuccessorSequences(int curr_state_ind,
                                   std::vector<std::vector<int>>& seqs_state_ids,
                                   std::vector<std::vector<double>> & seqs_transition_costs) override
        {
            if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace)
            {
                return getSuccessorSequencesCs(curr_state_ind, seqs_state_ids, seqs_transition_costs);
            }
            else
            {
                return getSuccessorSequencesWs(curr_state_ind, seqs_state_ids, seqs_transition_costs);
            }
        }

        /// @brief Visualize a state point in rviz for debugging
        /// @param state_id The state id
        /// @param type The type of state (greedy, attractor, etc)
        void VisualizePoint(double x, double y, double z) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
            marker.header.stamp = ros::Time();
            marker.ns = "graph";
            marker.id = vis_id_;
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
            vis_pub_.publish(marker);
            vis_id_++;
        }


        /* ##################################### */
        /* ####### E-GRAPH MIXIN METHODS ####### */
        /* ##################################### */

        /// @note It is important to make sure the search initiate the same states! (see getOrCreateRobotState())
        bool loadEGraph(const std::string& path) override {
            // The path needs to be a directory containing the experience files
            // check if path is a directory
            boost::filesystem::path p(path);
            if (!boost::filesystem::is_directory(p)) {
                std::cout << RED << "[ERROR]: Path in loadEGraph is not a directory" << RESET << std::endl;
                return false;
            }

            // loop through all files in the directory and parse them
            for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {})) {
                if (entry.path().extension() == ".csv") {
                    std::vector<StateType> egraph_states;
                    if (!parseEGraphFile(entry.path().string(), egraph_states) || egraph_states.empty()) {
                        continue;
                    }
                    auto& prev_state = egraph_states.front();
                    roundStateToDiscretization(prev_state, manipulation_type_->state_discretization_);
                    auto pid = egraph_.insert_node(prev_state);
                    state_to_egraph_nodes_[prev_state].push_back(pid);

                    int entry_s_id = getOrCreateRobotState(prev_state);

                    // map the state id to the node id in the experience graph
                    egraph_state_ids_.resize(pid + 1, -1);
                    egraph_state_ids_[pid] = entry_s_id;
                    states_to_nodes_[entry_s_id] = pid;

                    std::vector<StateType> edge_data;
                    for (size_t i = 1; i < egraph_states.size(); ++i) {
                        auto& curr_state = egraph_states[i];
                        StateType cs = curr_state;
                        if (curr_state != prev_state) { // TODO: check if its fine
                            auto cid = egraph_.insert_node(curr_state);
                            state_to_egraph_nodes_[curr_state].push_back(cid);

                            int curr_s_id = getOrCreateRobotState(curr_state);

                            // map the state id to the node id in the experience graph
                            egraph_state_ids_.resize(cid + 1, -1);
                            egraph_state_ids_[cid] = curr_s_id;
                            states_to_nodes_[curr_s_id] = cid;

                            // add edge
                            egraph_.insert_edge(pid, cid, edge_data);
                            pid = cid;
                            prev_state = cs;
                        } else {
                            edge_data.push_back(curr_state);
                        }
                    }
                }
            }
            // make sure all states have FK
            for (auto &state : states_) {
                if (state->state_mapped.empty()) {
                    moveit_interface_->calculateFK(state->state, state->state_mapped);
                }
            }
            return true;
        }

        void getEGraphNodes(int state_id,
                            std::vector<ims::smpl::ExperienceGraph::node_id> &nodes) override {
            auto it = states_to_nodes_.find(state_id);
            if (it != states_to_nodes_.end()) {
                nodes.push_back(it->second);
            }
        }

        bool shortcut(int first_id, int second_id, int& cost) override {
            auto* state_1 = getRobotHashEntry(first_id);
            auto* state_2 = getRobotHashEntry(second_id);

            if (state_1 == nullptr || state_2 == nullptr) {
                return false;
            }
            cost = 1000;
            return true;
        }

        bool checkShortcutTransition(int first_id,
                                     int second_id,
                                     PathType& trans_path) override {
            auto prev_nit = std::find(egraph_state_ids_.begin(), egraph_state_ids_.end(), first_id);
            auto curr_nit = std::find(egraph_state_ids_.begin(), egraph_state_ids_.end(), second_id);
            if (prev_nit != egraph_state_ids_.end() &&
                curr_nit != egraph_state_ids_.end()) {
                ims::smpl::ExperienceGraph::node_id prev_nid = std::distance(egraph_state_ids_.begin(), prev_nit);
                ims::smpl::ExperienceGraph::node_id curr_nid = std::distance(egraph_state_ids_.begin(), curr_nit);
                std::vector<ims::smpl::ExperienceGraph::node_id> node_path;
                bool found = findShortestExperienceGraphPath(prev_nid, curr_nid, node_path);
                if (found){
                    for (ims::smpl::ExperienceGraph::node_id n : node_path){
                        int s_id = egraph_state_ids_[n];
                        auto* entry = getRobotHashEntry(s_id);
                        assert(entry);
                        trans_path.push_back(entry->state);
                    }
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }

        bool snap(int first_id, int second_id, int& cost) override {
            auto* state_1 = getRobotHashEntry(first_id);
            auto* state_2 = getRobotHashEntry(second_id);

            if (state_1 == nullptr || state_2 == nullptr) {
                return false;
            }

            if (isStateToStateValid(state_1->state, state_2->state)) {
                cost = 1000;
                return true;
            } else {
                return false;
            }
        }

        bool checkSnapTransition(int first_id,
                                 int second_id,
                                 PathType& trans_path) override {
            int cost;
            if (snap(first_id, second_id, cost)){
                auto* entry = getRobotHashEntry(second_id);
                assert(entry);
                trans_path.push_back(entry->state);
                return true;
            } else {
                return false;
            }
        }

        const std::shared_ptr<ims::smpl::ExperienceGraph> getExperienceGraph() const override {
            std::shared_ptr<ims::smpl::ExperienceGraph> egraph_ptr = std::make_shared<ims::smpl::ExperienceGraph>(egraph_);
            return egraph_ptr;
        }

        std::shared_ptr<ims::smpl::ExperienceGraph> getExperienceGraph() override {
            std::shared_ptr<ims::smpl::ExperienceGraph> egraph_ptr = std::make_shared<ims::smpl::ExperienceGraph>(egraph_);
            return egraph_ptr;
        }

        int getStateID(ims::smpl::ExperienceGraph::node_id n) const override {
            if (n < egraph_state_ids_.size()) {
                return egraph_state_ids_[n];
            } else {
                return -1;
            }
        }

    private:
        /// @brief The E-Graph attributes
        hash_map<int, smpl::ExperienceGraph::node_id > states_to_nodes_;
        smpl::ExperienceGraph egraph_;
        std::vector<int> egraph_state_ids_;

        typedef hash_map<
            StateType ,
            std::vector<smpl::ExperienceGraph::node_id>,
            StateTypeHash> StateToEGraphNodesMap;

        StateToEGraphNodesMap state_to_egraph_nodes_;

        /// @brief Parsing function of experience graph file
        /// @param filepath - path to experience graph file
        /// @param egraph_states - vector of states in the experience graph
        /// @return true if parsing was successful, false otherwise
        static bool parseEGraphFile(const std::string& filepath,
                                    PathType& egraph_states) {
            std::ifstream egraph_file(filepath);
            if (!egraph_file.is_open()) {
                std::cout << RED << "[ERROR]: Failed to open experience graph file: " << filepath << RESET << std::endl;
                return false;
            }

            /* parse CSV
                the format of the CSV file is as follows:
                    1. header line: "Experience, N (number of states), dim (dimension of state)"
                    2. state lines: "state_1, state_2, ..., state_n"
            */

            std::string line;
            std::vector<std::string> tokens;
            std::vector<std::string> state_tokens;
            StateType state_values;
            int num_states = 0;
            int dim = 0;
            while (std::getline(egraph_file, line)) {
                // read line and tokenize on commas ',' and make sure no empty tokens are created
                boost::split(tokens, line, boost::is_any_of(","));
                tokens.erase(std::remove_if(tokens.begin(), tokens.end(), [](const std::string& s) { return s.empty(); }), tokens.end());

                if (tokens[0] == "Experience") {
                    num_states = std::stoi(tokens[1]);
                    dim = std::stoi(tokens[2]);
                    egraph_states.reserve(num_states);
                } else {
                    state_tokens.clear();
                    state_tokens = tokens;
                    state_values.clear();
                    state_values.reserve(dim);
                    for (const auto& token : state_tokens) {
                        state_values.push_back(std::stod(token));
                    }
                    egraph_states.emplace_back(state_values);
                }
            }
            return true;
        }

        /// @brief Find the shortest experience path between two states
        /// @param start_node - start node id
        /// @param goal_node - goal node id
        /// @param path - vector of states in the shortest experience path
        /// @return true if path was found, false otherwise
        bool findShortestExperienceGraphPath(
            ims::smpl::ExperienceGraph::node_id start_node,
            ims::smpl::ExperienceGraph::node_id goal_node,
            std::vector<ims::smpl::ExperienceGraph::node_id>& path)
        {
            struct ExperienceGraphSearchNode : ::smpl::HeapElement
            {
                int g;
                bool closed;
                ExperienceGraphSearchNode* bp;
                ExperienceGraphSearchNode() :
                    g(std::numeric_limits<int>::max()),
                    closed(false),
                    bp(nullptr)
                { }
            };

            struct NodeCompare
            {
                bool operator()(
                    const ExperienceGraphSearchNode& a,
                    const ExperienceGraphSearchNode& b)
                {
                    return a.g < b.g;
                }
            };

            typedef ::smpl::IntrusiveHeap<ExperienceGraphSearchNode, NodeCompare> heap_type;

            std::vector<ExperienceGraphSearchNode> search_nodes(egraph_.num_nodes());

            heap_type open;

            search_nodes[start_node].g = 0;
            open.push(&search_nodes[start_node]);
            int exp_count = 0;
            while (!open.empty()) {
                ++exp_count;
                ExperienceGraphSearchNode* min = open.min();
                open.pop();
                min->closed = true;

                if (min == &search_nodes[goal_node]) {
                    std::cout << RED << "[ERROR]: Found shortest experience graph path" << RESET << std::endl;
                    ExperienceGraphSearchNode* ps = nullptr;
                    for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                         s; s = s->bp)
                    {
                        if (s != ps) {
                            path.push_back(std::distance(search_nodes.data(), s));
                            ps = s;
                        } else {
                            std::cout << RED << "[ERROR]: Cycle detected!" << RESET << std::endl;
                        }
                    }
                    std::reverse(path.begin(), path.end());
                    return true;
                }

                ims::smpl::ExperienceGraph::node_id n = std::distance(search_nodes.data(), min);
                auto adj = egraph_.adjacent_nodes(n);
                for (auto ait = adj.first; ait != adj.second; ++ait) {
                    ExperienceGraphSearchNode& succ = search_nodes[*ait];
                    if (succ.closed) {
                        continue;
                    }
                    int new_cost = min->g + 1;
                    if (new_cost < succ.g) {
                        succ.g = new_cost;
                        succ.bp = min;
                        if (open.contains(&succ)) {
                            open.decrease(&succ);
                        } else {
                            open.push(&succ);
                        }
                    }
                }
            }

            std::cout << "Expanded " << exp_count << " nodes looking for shortcut" << std::endl;
            return false;
        }
    };

} // namespace ims

#endif //MANIPULATION_PLANNING_EGRAPH_MANIPULATION_ACTION_SPACE_HPP
