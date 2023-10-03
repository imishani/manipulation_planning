/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   trajectory_utils.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sept 19 2023
*/


#ifndef MANIPULATION_PLANNING_TRAJECTORY_UTILS_HPP
#define MANIPULATION_PLANNING_TRAJECTORY_UTILS_HPP

#include <vector>

#include <ros/ros.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <search/common/types.hpp>
#include <search/common/collisions.hpp>
#include <search/common/conflicts.hpp>
#include<manipulation_planning/action_space/manipulation_constrained_action_space.hpp>

namespace ims {

// TODO(yoraish): let's move trajectory profiling code to here.


void iterativeMultiAgentPathShortcutting(const std::shared_ptr<MultiAgentPaths>& paths,
                                         const std::vector<std::string>& move_group_names,
                                         const std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_spaces,
                                         int num_iters,
                                         std::shared_ptr<MultiAgentPaths>& shortcutted_paths
                                            )
{
    // Assign one of the action spaces to be the anchor action space. Collisions between all agents will be checked there.
    std::shared_ptr<ConstrainedActionSpace> anchor_action_space = action_spaces[0];

    // Start by checking that there are no collisions in the paths waypoints. This is a sanity check.
    std::vector<std::shared_ptr<Conflict>> conflicts;
    std::vector<ConflictType> conflict_types{ConflictType::POINT3D};

    anchor_action_space->getPathsConflicts(paths, conflicts, conflict_types, 10, move_group_names);

    std::cout << "Found " << conflicts.size() << " conflicts in the paths before shortcutting." << std::endl;

    // Shortcut the paths. Iterate over timesteps and attempt to move a point closer to the two nearest points to it. So we go between time [1,T-1]. At each t, we attempt to assign q[t] = 0.95*q[t] + 0.025 * (q[t-1] + q[t+1]). The weights are arbitrary.
    shortcutted_paths = std::make_shared<MultiAgentPaths>(*paths);
    std::shared_ptr<MultiAgentPaths> temp_shortcutted_paths = std::make_shared<MultiAgentPaths>(*paths);
    int max_path_length = 0;
    for (int a = 0; a < move_group_names.size(); ++a)
    {
        if (paths->at(a).size() > max_path_length)
        {
            max_path_length = paths->at(a).size();
        }
    }
    for(int iter{0}; iter < num_iters; iter++){

        
        for (int t = 1; t < max_path_length - 1; ++t)
        {
            // Iterate over all agents
            for (int a = 0; a < move_group_names.size(); ++a)
            {
                // Get the times for the current agent. If the requested time is larger than the maximum time in this path, then skip.
                if (t+1 >= paths->at(a).size()){
                    continue;
                }

                // Get the current point
                std::vector<double> current_point = paths->at(a)[t];

                // Get the previous point
                std::vector<double> previous_point = paths->at(a)[t-1];

                // Get the next point
                std::vector<double> next_point = paths->at(a)[t+1];

                // Compute the new point
                std::vector<double> new_point;
                for (int i = 0; i < current_point.size(); ++i){
                    new_point.push_back(0.6*current_point[i] + 0.2*(previous_point[i] + next_point[i]));
                }

                // Check if the point is valid.
                bool new_point_valid = action_spaces[a]->isStateValid(new_point);
                if (!new_point_valid){
                    // The point is not valid. Skip.
                    continue;
                }

                // Set the new point
                temp_shortcutted_paths->at(a)[t] = new_point;

                // Check for collisions.
                conflicts.clear();
                anchor_action_space->getPathsConflicts(temp_shortcutted_paths, conflicts, conflict_types, 10, move_group_names, t-1, t+1);
                // std::cout << "At t=" << t << ", found " << conflicts.size() << " conflicts in the paths after shortcutting." << std::endl;
                if (conflicts.size() == 0){
                    // No conflicts. Copy the temp paths to the shortcutted paths.
                    shortcutted_paths = std::make_shared<MultiAgentPaths>(*temp_shortcutted_paths);
                }
                else{
                    // There are conflicts. Copy the original paths to the shortcutted paths.
                    temp_shortcutted_paths = std::make_shared<MultiAgentPaths>(*paths);
                }
            } 
        }

    }


}


} // namespace ims

#endif //MANIPULATION_PLANNING_TRAJECTORY_UTILS_HPP
