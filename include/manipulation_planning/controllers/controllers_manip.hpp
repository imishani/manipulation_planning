/*
* Copyright (C) 2024, Itamar Mishani
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
 * \file   controllers_manip.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Aug 31 2024
 */

#pragma once

// include standard libraries
#include <iostream>
#include <unordered_map>

// include search libraries
#include <search/controllers/base_controller.hpp>
#include <manipulation_planning/action_space/manipulation_action_space_mgs.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <manipulation_planning/common/utils.hpp>

namespace ims {
struct PointSamplerController : Controller {
    struct PointSamplerUserData {
        int num_samples{10};
        StateType start_state;
        StateType goal_state;
        double dist;
    };

    PointSamplerController() {
        type = ControllerType::GENERATOR;
    }

    ~PointSamplerController() {
        delete static_cast<PointSamplerUserData *>(user_data);
    }

    void init(const std::shared_ptr<ManipulationMGSActionSpace> &manip_action_space,
              const StateType &start_state,
              const StateType &goal_state,
              double dist,
              int num_samples = 100) {
        auto *user_data = new PointSamplerUserData();
        user_data->start_state = start_state;
        user_data->goal_state = goal_state;
        user_data->dist = dist;
        user_data->num_samples = num_samples;
        this->user_data = user_data;
        this->as_ptr = manip_action_space;
    }
};

inline std::vector<ActionSequence> PointSamplerControllerFn(void *user,
                                                            const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    auto *user_data = static_cast<PointSamplerController::PointSamplerUserData *>(user);
    auto *manip_action_space = dynamic_cast<ManipulationMGSActionSpace *>(action_space_ptr.get());

    std::vector<ActionSequence> action_sequences;
    for (int i = 0; i < user_data->num_samples; i++) {
        auto start_state = user_data->start_state;
        auto goal_state = user_data->goal_state;
        auto dist = user_data->dist;
        StateType random_state;
        do {
            manip_action_space->generateRandomState(start_state, goal_state, dist, random_state);
        } while (!manip_action_space->isStateValid(random_state));
        action_sequences.push_back({random_state});
    }
    return action_sequences;
}

struct PointsIKAroundObjectsController : Controller {
    struct PointsIKObjectsUserData {
        std::shared_ptr<MoveitInterface> moveit_interface;
        std::vector<std::string> object_names;
        std::unordered_map<std::string, collision_detection::World::ObjectConstPtr> object_poses;
        // the AABB of the object
        std::unordered_map<std::string, std::pair<std::vector<Eigen::Vector3d>,
                                                  std::vector<double> > > shapes_bounding_spheres;
    };

    PointsIKAroundObjectsController() {
        type = ControllerType::GENERATOR;
    }

    ~PointsIKAroundObjectsController() {
        delete static_cast<PointsIKObjectsUserData *>(user_data);
    }

    void init(const std::shared_ptr<ManipulationMGSActionSpace> &manip_action_space,
              const std::shared_ptr<MoveitInterface>& moveit_interface) {
        auto *user_data = new PointsIKObjectsUserData();
        user_data->moveit_interface = moveit_interface;
        this->user_data = user_data;
        this->as_ptr = manip_action_space;
        // get the object names
        user_data->object_names = moveit_interface->getPlanningSceneMoveit()->getWorld()->getObjectIds();
        // get the object poses
        for (const auto &object_name : user_data->object_names) {
            user_data->object_poses[object_name] = moveit_interface->getPlanningSceneMoveit()->getWorld()->getObject(
                object_name);
            // get the bounding sphere
            std::vector<double> radius_tot;
            std::vector<Eigen::Vector3d> center_tot;
            for (const auto &shape : user_data->object_poses[object_name]->shapes_) {
                double radius;
                Eigen::Vector3d center;
                computeShapeBoundingSphere(shape.get(), center, radius);
                radius_tot.push_back(radius + 0.1);
                center_tot.push_back(center);
            }
            // get the sphere that bounds the object
            user_data->shapes_bounding_spheres[object_name] = {center_tot, radius_tot};
        }
    }
};

inline std::vector<ActionSequence> PointsIKAroundObjectsControllerFn(void *user,
                                                                     const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    auto *user_data = static_cast<PointsIKAroundObjectsController::PointsIKObjectsUserData *>(user);
    auto *manip_action_space = dynamic_cast<ManipulationMGSActionSpace *>(action_space_ptr.get());
    // for each shape, get the bounding sphere and sample points around it, then do IK
    std::vector<ActionSequence> action_sequences;
    for (const auto &object_name : user_data->object_names) {
        auto object_pose = user_data->object_poses[object_name];
        auto bounding_spheres = user_data->shapes_bounding_spheres[object_name];
        // sample points around the bounding spheres
        for (int i = 0; i < bounding_spheres.first.size(); i++) {
            auto center = bounding_spheres.first[i];
            auto radius = bounding_spheres.second[i];
            // sample points around the bounding sphere
            std::vector<Eigen::Vector3d> points;
            // get random points around the bounding sphere
            for (int j = 0; j < 5; j++) {
                Eigen::Vector3d point;
                // get random firection (unit vector)
                Eigen::Vector3d direction = Eigen::Vector3d::Random();
                direction.normalize();
                point = center + radius * direction;
                geometry_msgs::Pose pose;
                // set the position from the point and take random orientation
                pose.position.x = point.x(); pose.position.y = point.y(); pose.position.z = point.z();
                auto ori = Eigen::Quaterniond::UnitRandom().toRotationMatrix();
                Eigen::Quaterniond q(ori);
                // pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
                pose.orientation.x = 0.0; pose.orientation.y = 0.0;
                pose.orientation.z = 0.0; pose.orientation.w = 1.0;
                StateType state_IK;
                user_data->moveit_interface->calculateIK(pose, state_IK, 0.5);
                if (manip_action_space->isStateValid(state_IK)) {
                    action_sequences.push_back({state_IK});
                }
            }
        }
    }
    return action_sequences;
}



////////////////// Mobile Manipulation Controllers //////////////////////

struct PointSamplerControllerMobile : Controller {
    struct PointSamplerUserData {
        int num_samples{10};
        StateType start_state;
        StateType goal_state;
        double dist;
    };

    PointSamplerControllerMobile() {
        type = ControllerType::GENERATOR;
    }

    ~PointSamplerControllerMobile() {
        delete static_cast<PointSamplerUserData *>(user_data);
    }

    void init(const std::shared_ptr<MobileManipulationMGSActionSpace> &manip_action_space,
              const StateType &start_state,
              const StateType &goal_state,
              double dist,
              int num_samples = 100) {
        auto *user_data = new PointSamplerUserData();
        user_data->start_state = start_state;
        user_data->goal_state = goal_state;
        user_data->dist = dist;
        user_data->num_samples = num_samples;
        this->user_data = user_data;
        this->as_ptr = manip_action_space;
    }
};

inline std::vector<ActionSequence> MobilePointSamplerControllerFn(void *user,
                                                            const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    auto *user_data = static_cast<PointSamplerControllerMobile::PointSamplerUserData *>(user);
    auto *manip_action_space = dynamic_cast<MobileManipulationMGSActionSpace *>(action_space_ptr.get());

    std::vector<ActionSequence> action_sequences;
    for (int i = 0; i < user_data->num_samples; i++) {
        auto start_state = user_data->start_state;
        auto goal_state = user_data->goal_state;
        auto dist = user_data->dist;
        StateType random_state;
        do {
            manip_action_space->generateRandomState(start_state, goal_state, dist, random_state);
        } while (!manip_action_space->isStateValid(random_state));
        action_sequences.push_back({random_state});
    }
    return action_sequences;
}

struct PointsIKAroundObjectsControllerMobile : Controller {
    struct PointsIKObjectsUserData {
        std::shared_ptr<MoveitInterface> moveit_interface;
        std::vector<std::string> object_names;
        std::unordered_map<std::string, collision_detection::World::ObjectConstPtr> object_poses;
        // the AABB of the object
        std::unordered_map<std::string, std::pair<std::vector<Eigen::Vector3d>,
                                                  std::vector<double> > > shapes_bounding_spheres;
    };

    PointsIKAroundObjectsControllerMobile() {
        type = ControllerType::GENERATOR;
    }

    ~PointsIKAroundObjectsControllerMobile() {
        delete static_cast<PointsIKObjectsUserData *>(user_data);
    }

    void init(const std::shared_ptr<MobileManipulationMGSActionSpace> &manip_action_space,
              const std::shared_ptr<MoveitInterface>& moveit_interface) {
        auto *user_data = new PointsIKObjectsUserData();
        user_data->moveit_interface = moveit_interface;
        this->user_data = user_data;
        this->as_ptr = manip_action_space;
        // get the object names
        user_data->object_names = moveit_interface->getPlanningSceneMoveit()->getWorld()->getObjectIds();
        // get the object poses
        for (const auto &object_name : user_data->object_names) {
            user_data->object_poses[object_name] = moveit_interface->getPlanningSceneMoveit()->getWorld()->getObject(
                object_name);
            // get the bounding sphere
            std::vector<double> radius_tot;
            std::vector<Eigen::Vector3d> center_tot;
            for (const auto &shape : user_data->object_poses[object_name]->shapes_) {
                double radius;
                Eigen::Vector3d center;
                computeShapeBoundingSphere(shape.get(), center, radius);
                radius_tot.push_back(radius + 0.1);
                center_tot.push_back(center);
            }
            // get the sphere that bounds the object
            user_data->shapes_bounding_spheres[object_name] = {center_tot, radius_tot};
        }
    }
};

inline std::vector<ActionSequence> MobilePointsIKAroundObjectsControllerFn(void *user,
                                                                     const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    auto *user_data = static_cast<PointsIKAroundObjectsControllerMobile::PointsIKObjectsUserData *>(user);
    auto *manip_action_space = dynamic_cast<MobileManipulationMGSActionSpace *>(action_space_ptr.get());
    // for each shape, get the bounding sphere and sample points around it, then do IK
    std::vector<ActionSequence> action_sequences;
    for (const auto &object_name : user_data->object_names) {
        auto object_pose = user_data->object_poses[object_name];
        auto bounding_spheres = user_data->shapes_bounding_spheres[object_name];
        // sample points around the bounding spheres
        for (int i = 0; i < bounding_spheres.first.size(); i++) {
            auto center = bounding_spheres.first[i];
            auto radius = bounding_spheres.second[i];
            // sample points around the bounding sphere
            std::vector<Eigen::Vector3d> points;
            // get random points around the bounding sphere
            for (int j = 0; j < 5; j++) {
                Eigen::Vector3d point;
                // get random firection (unit vector)
                Eigen::Vector3d direction = Eigen::Vector3d::Random();
                direction.normalize();
                point = center + radius * direction;
                geometry_msgs::Pose pose;
                // set the position from the point and take random orientation
                pose.position.x = point.x(); pose.position.y = point.y(); pose.position.z = point.z();
                auto ori = Eigen::Quaterniond::UnitRandom().toRotationMatrix();
                Eigen::Quaterniond q(ori);
                // pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
                pose.orientation.x = 0.0; pose.orientation.y = 0.0;
                pose.orientation.z = 0.0; pose.orientation.w = 1.0;
                StateType state_IK;
                user_data->moveit_interface->calculateIK(pose, state_IK, 0.5);
                if (manip_action_space->isStateValid(state_IK)) {
                    action_sequences.push_back({state_IK});
                }
            }
        }
    }
    return action_sequences;
}



}
