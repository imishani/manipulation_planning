//
// Created by itamar on 10/30/23.
//

#ifndef MANIPULATION_PLANNING_COMMON_FACTORIES_HPP
#define MANIPULATION_PLANNING_COMMON_FACTORIES_HPP

#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <search/planners/arastar.hpp>
#include "manipulation_planning/common/moveit_scene_interface.hpp"

namespace ims {

/////////////////////////
// Heuristic Factories //
/////////////////////////
auto MakeBFSHeuristic(const planning_interface::PlannerConfigurationSettings& configuration_settings,
                      const std::shared_ptr<MoveitInterface>& scene_interface) -> std::unique_ptr<BaseHeuristic> {

    std::cout << "Getting distance field" << std::endl;
    std::shared_ptr<distance_field::PropagationDistanceField> df = scene_interface->getDistanceFieldMoveItInterface();
    std::cout << "Got distance field" << std::endl;
    // try to get the group name from the planner configuration, otherwise throw an error
    std::string group_name = configuration_settings.group;
    if (group_name.empty()) {
        ROS_ERROR_STREAM("No group name specified for BFS heuristic");
        throw std::runtime_error("No group name specified for BFS heuristic");
    }
    return std::make_unique<BFSHeuristic>(df, group_name);
}

auto MakeJointAnglesHeuristic(const planning_interface::PlannerConfigurationSettings& configuration_settings,
                              const std::shared_ptr<MoveitInterface>& scene_interface) -> std::unique_ptr<BaseHeuristic>{
    return std::make_unique<JointAnglesHeuristic>();
}

//////////////////////
// Search Factories //
//////////////////////
auto MakeAStar(BaseHeuristic* heuristic,
               const planning_interface::PlannerConfigurationSettings& configuration_settings) -> std::unique_ptr<BestFirstSearch>{
    ims::AStarParams params(heuristic);
    auto it = configuration_settings.config.find("allowed_planning_time");
    if (it != configuration_settings.config.end()) {
        params.time_limit_ = std::stod(it->second);
    } else {
        ROS_WARN_STREAM("No time limit specified for AStar, using default: " << 10.0);
        params.time_limit_ = 10.0;
    }
    return std::make_unique<AStar>(params);
}

auto MakeWAStar(BaseHeuristic* heuristic,
                const planning_interface::PlannerConfigurationSettings& configuration_settings) -> std::unique_ptr<BestFirstSearch> {
    double weight;
    auto weight_itr = configuration_settings.config.find("weight");
    if (weight_itr == configuration_settings.config.end()) {
        ROS_WARN_STREAM("No weight specified for WAstar, using default: " << 10.0);
        weight = 10.0;
    } else {
        weight = std::stod(weight_itr->second);
    }

    ims::wAStarParams params(heuristic, weight);
    auto it = configuration_settings.config.find("allowed_planning_time");
    if (it != configuration_settings.config.end()) {
        params.time_limit_ = std::stod(it->second);
        ROS_INFO_STREAM("wAstar time limit: " << params.time_limit_);
    } else {
        ROS_WARN_STREAM("No time limit specified for AStar, using default: " << 10.0);
        params.time_limit_ = 10.0;
    }
    return std::make_unique<ims::wAStar>(params);
}

auto MakeARAstar(BaseHeuristic* heuristic,
                 const planning_interface::PlannerConfigurationSettings& configuration_settings) -> std::unique_ptr<BestFirstSearch>{
    double initial_weight;
    double weight_delta;
    double final_weight;
    auto initial_weight_itr = configuration_settings.config.find("initial_weight");
    auto weight_delta_itr = configuration_settings.config.find("weight_delta");
    auto final_weight_itr = configuration_settings.config.find("final_weight");
    if (initial_weight_itr == configuration_settings.config.end()) {
        ROS_WARN_STREAM("No initial weight specified for ARAstar, using default: " << 100.0);
        initial_weight = 100.0;
    } else {
        initial_weight = std::stod(initial_weight_itr->second);
    }
    if (weight_delta_itr == configuration_settings.config.end()) {
        ROS_WARN_STREAM("No weight delta specified for ARAstar, using default: " << 10.0);
        weight_delta = 10.0;
    } else {
        weight_delta = std::stod(weight_delta_itr->second);
    }
    if (final_weight_itr == configuration_settings.config.end()) {
        ROS_WARN_STREAM("No final weight specified for ARAstar, using default: " << 1.0);
        final_weight = 1.0;
    } else {
        final_weight = std::stod(final_weight_itr->second);
    }
    ims::ARAStarParams params(heuristic, initial_weight, weight_delta, final_weight);
    auto it = configuration_settings.config.find("allowed_planning_time");
    if (it != configuration_settings.config.end()) {
        params.time_limit_ = std::stod(it->second);
        params.ara_time_limit = params.time_limit_;
        ROS_INFO_STREAM("ARAstar time limit: " << params.time_limit_);
    } else {
        ROS_WARN_STREAM("No time limit specified for AStar, using default: " << 10.0);
        params.time_limit_ = 10.0;
    }
    return std::make_unique<ARAStar>(params);
}

} // namespace ims

#endif
