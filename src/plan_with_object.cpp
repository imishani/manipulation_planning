// Hayden Feddock
// 6/14/2023

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometry_msgs/Pose.h>

/*
#include <manipulation_planning/common/MoveitInterface.hpp>
#include <ctmp/ctmpActionSpace.hpp>
#include <manipulation_planning/heuristics/manipHeuristics.hpp>
#include <planners/BestFirstSearch.hpp>
#include <manipulation_planning/common/utils.hpp>
#include <ctmp/zero_time_planner.hpp>
*/


double rad2deg(double);
std::vector<double> rad2deg(std::vector<double>);
double deg2rad(double);
std::vector<double> deg2rad(std::vector<double>);


//////////////////// MAIN FUNCTION ////////////////////

int main (int argc, char** argv) {

    // Initialize the ros node and create the node handle
    ros::init(argc, argv, "move_with_object_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");

    // Get the current state of the robot
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    /*

    // Create the Moveit interface
    ims::MoveitInterface scene_interface("manipulator_1");

    // Create the action type
    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
    std::string path_mprim = full_path.string() + "/config/manip.mprim";
    ims::ctmpActionType action_type (path_mprim);

    // Create a CTMP action space
    ims::ctmpActionSpace(scene_interface, action_type);
    std::shared_ptr<ims::ctmpActionSpace> action_space_ptr = std::make_shared<ims::ctmpActionSpace>(scene_interface, action_type);

    // Define the planning parameters
    auto df = ims::getDistanceFieldMoveIt();
    auto* heuristic = new ims::BFSHeuristic(df, "manipulator_1");
    ims::BestFirstSearchParams params(heuristic);

    // Define the start (current) and goal states in radians
    stateType start_state {0, 0, 0, 0, 0, 0};
    auto joint_names = move_group.getVariableNames();
    for (int i = 0; i < 6; i++) {
        start_state[i] = current_state->getVariablePosition(joint_names[i]);
    }
    stateType goal_state({1, 55, 95, 1, -40, 90});
    ims::deg2rad(goal_state);

    // Normalize the angles of the start and goal states
    ims::normalizeAngles(start_state);
    ims::normalizeAngles(goal_state);

    // Discretize the start and goal states
    stateType discretization({1, 1, 1, 1, 1, 1});
    ims::deg2rad(discretization);
    ims::roundStateToDiscretization(start_state, discretization);
    ims::roundStateToDiscretization(goal_state, discretization);

    // Create the ctmp zero time planner
    ZeroTimePlanner planner;
    
    bool query_mode = false;
    std::string planner_type = "RRTConnect";

    // Initialize the ctmp zero time planner
    try {
        planner.initializePlanner(action_space_ptr, params, start_state, goal_state, query_mode, planner_type);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }


    */




    




    ros::shutdown();
    return 0;


    /*
    // Create an interface for the manipulator arm and the gripper
    moveit::planning_interface::MoveGroupInterface move_group_interface("manipulator_1");
    //moveit::planning_interface::MoveGroupInterface gripper_group_interface("gripper_1");

    // Create an interface for the planning scene to catch collisions
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Reset the scene
    planning_scene_interface.removeCollisionObjects({"cube", "box1"});

    /////////////////// PLAN TO A CARTESIAN-SPACE GOAL ////////////////////

    // Create a target position for the desired cartesian coordinates of the end effector
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 0.0;
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.6;
    target_pose1.position.z = 0.76;
    move_group_interface.setPoseTarget(target_pose1);

    // Plan the path from the current arm position the target position
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // If the planning is successful, then execute the plan
    if (success) {
        ROS_INFO("Planning successful!");
        move_group_interface.execute(plan1);
    } else {
        ROS_INFO("Planning unsuccessful");
        return 0;
    }

    /////////////////// CREATE A COLLISION OBJECT TO AVOID ////////////////////

    // Define a collision object ROS message for the object to avoid
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // Give the object it's ID
    collision_object.id = "box1";

    // Define the shape of the box object
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define a position for the box object (relative to frame id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.00;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Create a vector to hold all of the collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Add the collision object to the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    ////////////// PLAN TO A CARTESIAN-SPACE GOAL AVOIDING COLLISION OBJECT //////////////

    // Create a target position for the desired cartesian coordinates of the end effector
    geometry_msgs::Pose target_pose2;
    target_pose1.orientation.w = 0.0;
    target_pose1.position.x = 0.5;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 1.25;
    move_group_interface.setPoseTarget(target_pose2);

    // Plan the path from the current arm position the target position
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = (move_group_interface.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // If the planning is successful, then execute the plan
    if (success) {
        ROS_INFO("Planning successful!");
        move_group_interface.execute(plan2);
    } else {
        ROS_INFO("Planning unsuccessful");
        return 0;
    }
    */

    /*
    // Initialize the ros node and create the node handle
    ros::init(argc, argv, "move_to_specific_point_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    // Create an interface for the manipulator arm and the gripper
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_1");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper_1");

    // Create an interface for the planning scene to catch collisions
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Reset the scene
    planning_scene_interface.removeCollisionObjects({"cube"});

    //////////////////////////// CREATE WALLS ////////////////////////////

    // Define a collision object for the left and right walls
    moveit_msgs::CollisionObject left_wall;
    moveit_msgs::CollisionObject right_wall;
    left_wall.header.frame_id = move_group.getPlanningFrame();
    right_wall.header.frame_id = move_group.getPlanningFrame();
    left_wall.id = "left_wall";
    right_wall.id = "right_wall";

    // Set the shape and size of the wall
    shape_msgs::SolidPrimitive wall_primitive;
    wall_primitive.type = shape_msgs::SolidPrimitive::BOX;
    wall_primitive.dimensions.resize(3);
    wall_primitive.dimensions[0] = 0.02;
    wall_primitive.dimensions[1] = std::sqrt(2);
    wall_primitive.dimensions[2] = 2.0;

    // Set the position of the walls
    geometry_msgs::Pose left_wall_pose;
    left_wall_pose.orientation.w = 0.9238795;
    left_wall_pose.orientation.x = 0.0;
    left_wall_pose.orientation.y = 0.0;
    left_wall_pose.orientation.z = 0.3826834;
    left_wall_pose.position.x = 0.0;
    left_wall_pose.position.y = 1.0;
    left_wall_pose.position.z = 1.81;

    geometry_msgs::Pose right_wall_pose;
    right_wall_pose.orientation.w = 0.9238795;
    right_wall_pose.orientation.x = 0.0;
    right_wall_pose.orientation.y = 0.0;
    right_wall_pose.orientation.z = -0.3826834;
    right_wall_pose.position.x = 0.0;
    right_wall_pose.position.y = -1.0;
    right_wall_pose.position.z = 1.81;

    // Set the color of the wall
    std_msgs::ColorRGBA wall_color; // TODO: wall color does not work right now
    wall_color.r = 1.0;
    wall_color.g = 0.0;
    wall_color.b = 0.0;
    wall_color.a = 0.2;

    // Give the properties of the object and it's position to the walls
    left_wall.primitives.push_back(wall_primitive);
    left_wall.primitive_poses.push_back(left_wall_pose);
    left_wall.operation = left_wall.ADD;

    right_wall.primitives.push_back(wall_primitive);
    right_wall.primitive_poses.push_back(right_wall_pose);
    right_wall.operation = right_wall.ADD;


    //////////////////////////// CREATE OBJECT ////////////////////////////

    // Define a collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "cube";

    // Set the shape and size of the collision object
    shape_msgs::SolidPrimitive object_primitive;
    object_primitive.type = shape_msgs::SolidPrimitive::BOX;
    object_primitive.dimensions.resize(3);
    object_primitive.dimensions[0] = 0.05;
    object_primitive.dimensions[1] = 0.05;
    object_primitive.dimensions[2] = 0.05;

    // Set the position of the collision object
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = 0.0;
    object_pose.position.x = 0.0;
    object_pose.position.y = 0.6;
    object_pose.position.z = 0.76;

    // Give the properties of the object and it's position to the collision object
    collision_object.primitives.push_back(object_primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_interface.applyPlanningScene(planning_scene);

    //////////////////////////// PRE-GRASP POSE ////////////////////////////

    // Set the goal configuration to the pre-grasp pose
    //std::vector<double> goal_joint_values_deg = {0, 0, 0, 0, 0, 0};
    std::vector<double> goal_joint_values_deg = {75, 34, 56, 1, -21, -165}; // TODO: Implement inverse kinematics to determine location of goal in joint space
    std::vector<double> goal_joint_values = deg2rad(goal_joint_values_deg);

    // Set the target state as the goal state
    move_group.setJointValueTarget(goal_joint_values);

    // Create a plan to approach the pre-grasp position
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    bool success = (move_group.plan(pre_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning was successful, execute the planned motion
    if (success) {
        ROS_INFO("Pre-grasp planning successful!");
        move_group.execute(pre_grasp_plan);
    } else {
        ROS_INFO("Pre-grasp planning unsuccessful");
        return 0;
    }

    //////////////////////////// GRASPING POSE ////////////////////////////

    // Set the goal configuration to the grasping pose
    goal_joint_values_deg = {74, 43, 55, 1, -12, -165};
    goal_joint_values = deg2rad(goal_joint_values_deg);

    // Set the target state as the goal state
    move_group.setJointValueTarget(goal_joint_values);

    // Create a plan to approach the grasping position
    moveit::planning_interface::MoveGroupInterface::Plan grasping_plan;
    success = (move_group.plan(grasping_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning was successful, execute the planned motion
    if (success) {
        ROS_INFO("Grasp planning successful!");
        move_group.execute(grasping_plan);
    } else {
        ROS_INFO("Grasp planning unsuccessful");
        return 0;
    }

    //////////////////////////// GRIPPER POSE ////////////////////////////

    // Set the goal configuration for the gripper
    goal_joint_values_deg = {19};
    goal_joint_values = deg2rad(goal_joint_values_deg);

    // Set the target state as the goal state
    gripper_group.setJointValueTarget(goal_joint_values);

    // Create a plan to approach the grasping position
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    success = (gripper_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning was successful, execute the planned motion
    if (success) {
        ROS_INFO("Gripper planning successful!");
        move_group.execute(gripper_plan);
    } else {
        ROS_INFO("Gripper planning unsuccessful");
        return 0;
    }
    
    // Attach the object to the robot
    move_group.attachObject("cube");
    ros::Duration(1.0).sleep(); // Allow time for attachment to be processed

    //////////////////////////// POST-GRASP POSE ////////////////////////////

    // Set the goal configuration to the post-grasping pose
    goal_joint_values_deg = {75, 34, 56, 1, -21, -165};
    goal_joint_values = deg2rad(goal_joint_values_deg);

    // Set the target state as the goal state
    move_group.setJointValueTarget(goal_joint_values);

    // Create a plan to approach the post-grasping position
    moveit::planning_interface::MoveGroupInterface::Plan post_grasp_plan;
    success = (move_group.plan(post_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning was successful, execute the planned motion
    if (success) {
        ROS_INFO("Post-grasp planning successful!");
        move_group.execute(post_grasp_plan);
    } else {
        ROS_INFO("Post-grasp planning unsuccessful");
        return 0;
    }

    //////////////////////////// PLACE OBJECT ////////////////////////////

    // Set the goal configuration to the placing pose
    goal_joint_values_deg = {-10, 57, 103, -1, -45, -78};
    goal_joint_values = deg2rad(goal_joint_values_deg);

    // Set the target state as the goal state
    move_group.setJointValueTarget(goal_joint_values);

    // Create a plan to approach the placing position
    moveit::planning_interface::MoveGroupInterface::Plan placing_plan;
    success = (move_group.plan(placing_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning was successful, execute the planned motion
    if (success) {
        ROS_INFO("Place planning successful!");
        move_group.execute(placing_plan);
    } else {
        ROS_INFO("Place planning unsuccessful");
        return 0;
    }

    // Release the object
    move_group.detachObject("cube");
    ros::Duration(1.0).sleep();

    //////////////////////////// RESET POSE ////////////////////////////

    // Set the goal configuration to the reset pose
    goal_joint_values_deg = {0, 0, 0, 0, 0, 0};
    goal_joint_values = deg2rad(goal_joint_values_deg);

    // Set the target state as the goal state
    move_group.setJointValueTarget(goal_joint_values);

    // Create a plan to approach the reset position
    moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
    success = (move_group.plan(reset_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning was successful, execute the planned motion
    if (success) {
        ROS_INFO("Reset planning successful!");
        move_group.execute(reset_plan);
    } else {
        ROS_INFO("Reset planning unsuccessful");
        return 0;
    }

    return 0;

    */
}

/*

//////////////////// EXTRA FUNCTIONS ////////////////////

// Convert radians to degrees
double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

// Convert degrees to radians
double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// Convert a vector of radians to degrees
std::vector<double> rad2deg(std::vector<double> rads) {
    std::vector<double> degs(rads.size());
    for (int i=0; i<rads.size(); i++) {
        degs[i] = rad2deg(rads[i]);
    }
    return degs;
}

// Convert a vector of degrees to radians
std::vector<double> deg2rad(std::vector<double> degs) {
    std::vector<double> rads(degs.size());
    for (int i=0; i<degs.size(); i++) {
        rads[i] = deg2rad(degs[i]);
    }
    return rads;
}

*/