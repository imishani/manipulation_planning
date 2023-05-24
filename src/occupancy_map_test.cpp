//
// Created by itamar on 5/23/23.
//

// standard includes
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/robot_state/robot_state.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// project includes
#include <manipulation_planning/common/utils.hpp>


int main(int argc, char** argv){

    // initialize ROS
    ros::init(argc, argv, "occupancy_map_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("manipulator_1");

    auto df = ims::getDistanceFieldMoveIt();
    // show the bounding box of the distance field
    ros::Publisher bb_pub = nh.advertise<visualization_msgs::Marker>("bb_marker", 10);
    // get the planning frame
    ims::visualizeBoundingBox(df, bb_pub, group.getPlanningFrame());

    // get the current robot state
    moveit::core::RobotState current_state = *group.getCurrentState();

    std::vector<std::vector<int>> occupancy_grid;
    ims::getRobotOccupancy(df, current_state, group, occupancy_grid);

    ims::countOccupiedCells(*df);
    // add markers to RVIZ to visualize the occupancy grid
    ros::Publisher one_marker_pub = nh.advertise<visualization_msgs::Marker>("occ_marker", 0);
    ros::Duration(1).sleep();
    // visualize all occupied cells

    visualization_msgs::Marker markers;
    markers.header.frame_id = group.getPlanningFrame();
    markers.header.stamp = ros::Time();
    markers.ns = "occupied_cells";
    markers.id = 1;
    markers.type = visualization_msgs::Marker::CUBE_LIST;
    markers.action = visualization_msgs::Marker::ADD;
    markers.scale.x = df->getResolution();
    markers.scale.y = df->getResolution();
    markers.scale.z = df->getResolution();
    markers.color.a = 0.2;
    markers.color.r = 0.0;
    markers.color.g = 0.0;
    markers.color.b = 1.0;
    markers.points.clear();
    markers.pose.orientation.w = 1.0;
    size_t num_occupied_cells = 0;
    for (int x {0} ; x < df->getXNumCells(); x++){
        for (int y{0}; y < df->getYNumCells(); y++){
            for (int z {0} ; z < df->getZNumCells(); z++ ){
                if (df->getCell(x, y, z).distance_square_ == 0){
                    geometry_msgs::Point p;
                    df->gridToWorld(x, y, z, p.x, p.y, p.z);
                    markers.points.push_back(p);
                    num_occupied_cells++;
                }
            }
        }
    }
    std::cout << "Added " << num_occupied_cells << " occupied cells to the marker array" << std::endl;
    one_marker_pub.publish(markers);


    // create a marker for each occupied cell
//    visualization_msgs::Marker marker;
//    // reference frame is the origin of the df
//    marker.header.frame_id = group.getPlanningFrame();
//    marker.header.stamp = ros::Time();
//    marker.ns = "ss";
//    marker.id = 1;
//    marker.type = visualization_msgs::Marker::CUBE_LIST;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.scale.x = df->getResolution();
//    marker.scale.y = df->getResolution();
//    marker.scale.z = df->getResolution();
//    marker.color.a = 0.2;
//    marker.color.r = 0.0;
//    marker.color.g = 0.0;
//    marker.color.b = 1.0;
//    marker.points.clear();
//    for (int i = 0; i < occupancy_grid.size(); i++) {
//        geometry_msgs::Point p;
//        df->gridToWorld(occupancy_grid[i][0], occupancy_grid[i][1], occupancy_grid[i][2],
//                        p.x, p.y, p.z);
//        marker.points.push_back(p);
//    }
//    std::cout << "Number of occupied cells: " << marker.points.size() << std::endl;
//    one_marker_pub.publish(marker);
//    ros::Duration(2).sleep();
//
//    ROS_INFO("Publishing markers");

    return 0;


}