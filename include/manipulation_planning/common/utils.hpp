//
// Created by itamar on 4/3/23.
//

#ifndef MANIPULATION_PLANNING_UTILS_HPP
#define MANIPULATION_PLANNING_UTILS_HPP

#include <vector>

#include <ros/ros.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/distance_field/distance_field.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <common/types.hpp>



namespace ims {

    /// \brief A function that converts a state from radians to degrees
    /// \param state The state
    template <typename T>
    void rad2deg(std::vector<T>& state) {
        for (auto& val : state) {
            val = val * 180 / M_PI;
        }
    }

    /// \brief A function that converts a state from degrees to radians
    /// \param state The state
    template <typename T>
    void deg2rad(std::vector<T>& state) {
        for (auto& val : state) {
            val = val * M_PI / 180;
        }
    }

    /// \brief A function that dealing with the discontinuity of the joint angles
    /// \param state The state to check
    /// \return The state with the joint angles in the range of [-pi, pi]
    template <typename T>
    void normalizeAngles(std::vector<T>& state){
        for (int i = 0; i < state.size(); ++i) {
            if (state[i] > M_PI){
                state[i] = state[i] - 2*M_PI;
            }
            else if (state[i] < -M_PI){
                state[i] = state[i] + 2*M_PI;
            }
        }
    }

    template <typename T>
    void checkFixGimbalLock(T y, T p, T r){
        // check if current state in gimbal lock
        /// TODO: Implement

    }

    template <typename T>
    void get_euler_zyx(const Eigen::Matrix<T, 3, 3>& rot, T& y, T& p, T& r)
    {
        y = std::atan2(rot(1, 0), rot(0, 0));
        p  = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
        r = std::atan2(rot(2, 1), rot(2, 2));
    }

    template <typename T>
    void get_euler_zyx(const Eigen::Quaternion<T>& rot, T& y, T& p, T& r)
    {
        Eigen::Matrix<T, 3, 3> R(rot);
        get_euler_zyx(R, y, p, r);
    }

    template <typename T>
    void from_euler_zyx(T y, T p, T r, Eigen::Matrix<T, 3, 3>& rot)
    {
        rot = Eigen::AngleAxis<T>(y, Eigen::Matrix<T, 3, 1>::UnitZ()) *
              Eigen::AngleAxis<T>(p, Eigen::Matrix<T, 3, 1>::UnitY()) *
              Eigen::AngleAxis<T>(r, Eigen::Matrix<T, 3, 1>::UnitX());
    }

    template <typename T>
    void from_euler_zyx(T y, T p, T r, Eigen::Quaternion<T>& q)
    {
        Eigen::Matrix<T, 3, 3> R;
        from_euler_zyx(y, p, r, R);
        q = Eigen::Quaternion<T>(R);
    }

    template <typename T>
    void normalize_euler_zyx(T& y, T& p, T& r)
    {
        Eigen::Matrix<T, 3, 3> rot;
        from_euler_zyx(y, p, r, rot);
        get_euler_zyx(rot, y, p, r);
    }

    template <typename T>
    void normalize_euler_zyx(T* angles) // in order r, p, y
    {
        Eigen::Matrix<T, 3, 3> rot;
        from_euler_zyx(angles[2], angles[1], angles[0], rot);
        get_euler_zyx(rot, angles[2], angles[1], angles[0]);
    }

    /// \brief Convert hopf coordinates to quaternion
    /// \param hopf The hopf coordinates vector
    /// \param q The quaternion (by reference)
    inline void hopfToQuaternion(const Eigen::Vector3d& hopf, Eigen::Quaterniond& q){
        double theta = hopf[0];
        double phi = hopf[1];
        double psi = hopf[2];
        q.w() = cos(theta/2.0)*cos(psi/2.0);
        q.x() = cos(theta/2.0)*sin(psi/2.0);
        q.y() = sin(theta/2.0)*cos(phi + psi/2.0);
        q.z() = sin(theta/2.0)*sin(phi + psi/2.0);
    }

    /// \brief Convert quaternion to hopf coordinates
    /// \param q The quaternion
    /// \param hopf The hopf coordinates vector (by reference)
    /// The range of the hopf coordinates is the following:
    /// theta: [0, pi]
    /// phi: [0, 2pi]
    /// psi: [0, 2pi]
    inline void quaternionToHopf(const Eigen::Quaterniond& q, Eigen::Vector3d& hopf){
        double x = q.x();
        double y = q.y();
        double z = q.z();
        double w = q.w();
        // Make sure that w >= 0 and if not, negate the quaternion
        if (w < 0.0){
            x = -x;
            y = -y;
            z = -z;
            w = -w;
        }
        hopf[2] = 2.0*atan2(x, w);
        hopf[1] = atan2(z, y) - hopf[2]/2.0;
        if (hopf[1] < 0.0){
            hopf[1] += 2.0*M_PI;
        }
        hopf[0] = 2.0*atan2(sqrt(y*y + z*z), sqrt(x*x + w*w));
    }


    /// \brief A function that takes the discretization vector and a state, and round it to the nearest discretization value
    /// \param state The state to check
    /// \param discretization The discretization vector
    inline void roundStateToDiscretization(stateType& state, const stateType& discretization){
        for (int i = 0; i < state.size(); ++i) {
            state[i] = round(state[i]/discretization[i])*discretization[i];
        }
    }


    /// \brief Read a txt file and return a vector of points
    /// \param file_path The path to the file
    /// \return The vector of points
    inline std::vector<std::vector<double>> readPointsFromFile(const std::string& file_path){
        std::vector<std::vector<double>> points;
        std::ifstream file(file_path);
        std::string line;
        while (std::getline(file, line)){
            std::vector<double> point;
            std::stringstream ss(line);
            std::string value;
            while (std::getline(ss, value, ',')){
                point.push_back(std::stod(value));
            }
            points.push_back(point);
        }
        return points;
    }



    /// \brief Search for the nearest point in a 1d vector in the most efficient way assuming the vector is sorted
    /// \param point The point to search for (int/double)
    /// \param points The vector of points (vector of int/double)
    /// \return The index of the nearest point
    template <typename T>
    inline int findNearestPoint(const T& point, const std::vector<T>& points){
        int index = 0;
        double min_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < points.size(); ++i) {
            double distance = std::abs(point - points[i]);
            if (distance < min_distance){
                min_distance = distance;
                index = i;
            }
        }
        return index;
    }

    /// \brief geodesic distance
    /// \param q1 The first hopf coordinates
    /// \param q2 The second hopf coordinates
    /// \return The geodesic distance
    inline double geodesicDistance(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2){
        double theta1 = q1[0];
        double phi1 = q1[1];
        double psi1 = q1[2];
        double theta2 = q2[0];
        double phi2 = q2[1];
        double psi2 = q2[2];
        double distance = acos(cos(theta1)*cos(theta2) + sin(theta1)*sin(theta2)*cos(phi1 - phi2 + psi1 - psi2));
        return distance;
    }


    /// \brief Profile the trajectory
    /// \param start The start joint state. type: stateType
    /// \param goal The goal joint state. type: stateType
    /// \param trajectory a vector of joint states. type: std::vector<stateType>
    /// \param move_group_ The move group object. type: moveit::planning_interface::MoveGroupInterface
    /// \param trajectory_msg The output trajectory. type: moveit_msgs::RobotTrajectory
    /// \return success bool
    inline bool profileTrajectory(const stateType & start,
                                  const stateType & goal,
                                  const std::vector<stateType> & trajectory,
                                  const moveit::planning_interface::MoveGroupInterface & move_group_,
                                  moveit_msgs::RobotTrajectory& trajectory_msg){

        trajectory_msg.joint_trajectory.header.frame_id = move_group_.getPlanningFrame();
        trajectory_msg.joint_trajectory.joint_names = move_group_.getActiveJoints();
        trajectory_msg.joint_trajectory.points.resize(trajectory.size());
        for (int i = 0; i < trajectory.size(); ++i) {
            trajectory_msg.joint_trajectory.points[i].positions = trajectory[i];
        }

        // Create a RobotTrajectory object
        robot_trajectory::RobotTrajectory robot_trajectory(move_group_.getRobotModel(), move_group_.getName());
        // convert the trajectory vector to trajectory message
        moveit::core::RobotState start_state_moveit(robot_trajectory.getRobotModel());
        // set start_state_moveit to the start state of the trajectory
        start_state_moveit.setJointGroupPositions(move_group_.getName(), trajectory[0]);
        robot_trajectory.setRobotTrajectoryMsg(start_state_moveit, trajectory_msg);

        // Trajectory processing
        trajectory_processing::IterativeParabolicTimeParameterization time_param(true);
        if (!time_param.computeTimeStamps(robot_trajectory)){
            ROS_ERROR("Failed to compute timestamps for trajectory");
            return false;
        }
        robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

        return true;
    }


    /// \brief Get the distance field
    inline std::shared_ptr<distance_field::PropagationDistanceField> getDistanceFieldMoveIt(double df_size_x = 3.0,
                                                                                            double df_size_y = 3.0,
                                                                                            double df_size_z = 3.0,
                                                                                            double df_res = 0.02,
                                                                                            double df_origin_x = -0.75,
                                                                                            double df_origin_y = -1.5,
                                                                                            double df_origin_z = 0.0,
                                                                                            double max_distance = 1.8){

        auto df = std::make_shared<distance_field::PropagationDistanceField>(df_size_x, df_size_y, df_size_z,
                                                                             df_res, df_origin_x, df_origin_y, df_origin_z,
                                                                             max_distance);
        // get the planning scene interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // get the collision objects
        auto objects = planning_scene_interface.getObjects();
        std::vector<moveit_msgs::CollisionObject> objs;
        for (auto& obj : objects) {
            objs.push_back(obj.second);
        }
        // fill the distance field
        for (auto& obj : objs) {

            Eigen::Isometry3d pose;
            // convert from pose to eigen
            geometry_msgs::Pose pose_msg = obj.pose;
            tf::poseMsgToEigen(pose_msg, pose);
            shapes::Shape* shape;
            // get shape from collision object
            if (obj.primitives.empty() && obj.meshes.empty()) {
                // raise exception
                std::cout << "No shape in collision object" << std::endl;
                std::exception e;
            }
            else if (!obj.primitives.empty()){
                // cnsrtuct shape from primitive;
                shape = shapes::constructShapeFromMsg(obj.primitives[0]);
            }
            else {
                shape = shapes::constructShapeFromMsg(obj.meshes[0]);
            }
            df->addShapeToField(shape, pose);
        }
        return df;
    }

    /// \brief Check if a point is in collision
    /// \param df The distance field
    /// \param point The point to check
    /// \return True if the point is in collision
    inline bool isPointInCollision(const distance_field::PropagationDistanceField &df, const Eigen::Vector3d &point){
        int x, y, z;
        df.worldToGrid(point[0], point[1], point[2], x, y, z);
        return df.getCell(x, y, z).distance_square_ == 0;
    }

    /// \brief Check if cell is occupied
    /// \param df The distance field
    /// \param cell The cell to check
    /// \return True if the cell is occupied
    inline bool isCellOccupied(const distance_field::PropagationDistanceField &df, const Eigen::Vector3i &cell){
        return df.getCell(cell[1], cell[1], cell[2]).distance_square_ == 0;
    }

    /// \brief Count occupied cells in the distance field
    /// \param df The distance field
    /// \return The number of occupied cells
    inline unsigned int countOccupiedCells(const distance_field::PropagationDistanceField &df){
        unsigned int count = 0;
        for (int z = 0; z < df.getZNumCells(); z++)
        {
            for (int x = 0; x < df.getXNumCells(); x++)
            {
                for (int y = 0; y < df.getYNumCells(); y++)
                {
                    if (df.getCell(x, y, z).distance_square_ == 0)
                    {
                        count++;
                    }
                }
            }
        }
        return count;
    }
}


#endif //MANIPULATION_PLANNING_UTILS_HPP