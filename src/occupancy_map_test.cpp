//
// Created by itamar on 5/23/23.
//

// standard includes
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <visualization_msgs/Marker.h>


// project includes
#include <manipulation_planning/common/utils.hpp>


int main(int argc, char** argv){

    // initialize ROS
    ros::init(argc, argv, "occupancy_map_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // get the move group interface
    moveit::planning_interface::MoveGroupInterfaceUniquePtr group = std::make_unique<moveit::planning_interface::MoveGroupInterface>("manipulator_1");
    auto df = ims::getDistanceFieldMoveIt();
    // show the bounding box of the distance field
    ros::Publisher bb_pub = nh.advertise<visualization_msgs::Marker>("bb_marker", 10);
    // get the planning frame
    ims::visualizeBoundingBox(df, bb_pub, group->getPlanningFrame());

    // get the current robot state
    moveit::core::RobotState current_state = *group->getCurrentState();

    std::vector<std::vector<int>> occupancy_grid;
    ims::getRobotOccupancy(df, current_state, group, occupancy_grid);

    ims::countOccupiedCells(*df);
    // add markers to RVIZ to visualize the occupancy grid
    ros::Publisher one_marker_pub = nh.advertise<visualization_msgs::Marker>("occ_marker", 0);
    ros::Duration(1).sleep();
    // visualize all occupied cells

    ims::visualizeOccupancy(df, one_marker_pub, group->getPlanningFrame(), 1);
    ros::Duration(1).sleep();


    return 0;


}
