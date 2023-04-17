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
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include "KDTree.hpp"


#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


/// \brief A function that dealing with the discontinuity of the joint angles
/// \param state The state to check
/// \return The state with the joint angles in the range of [-pi, pi]
stateType normalizeAngles(const stateType& state){
    stateType normalized_state = state;
    for (int i = 0; i < state.size(); ++i) {
        if (normalized_state[i] > M_PI){
            normalized_state[i] = normalized_state[i] - 2*M_PI;
        }
        else if (normalized_state[i] < -M_PI){
            normalized_state[i] = normalized_state[i] + 2*M_PI;
        }
    }
    return normalized_state;
}

/// \brief Convert hopf coordinates to quaternion
/// \param hopf The hopf coordinates vector
/// \param q The quaternion (by reference)
void hopfToQuaternion(const Eigen::Vector3d& hopf, Eigen::Quaterniond& q){
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
void quaternionToHopf(const Eigen::Quaterniond& q, Eigen::Vector3d& hopf){
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
void roundStateToDiscretization(stateType& state, const stateType& discretization){
    for (int i = 0; i < state.size(); ++i) {
        state[i] = round(state[i]/discretization[i])*discretization[i];
    }
}

/// \brief Create a KDTree object from a vector of 2d points
/// \param points The vector of points
/// \return The KDTree object
KDTree<Eigen::Vector3d>* createKDTree(const std::vector<Eigen::Vector3d>& points){
    KDTree<Eigen::Vector3d>* tree;
    tree = new KDTree<Eigen::Vector3d>(points, 2);
    return tree;
}

/// \brief Read a txt file and return a vector of points
/// \param file_path The path to the file
/// \return The vector of points
std::vector<std::vector<double>> readPointsFromFile(const std::string& file_path){
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
/// \param point The point to search for
/// \param points The vector of points
/// \return The index of the nearest point
int findNearestPoint(const double& point, const std::vector<double>& points){
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
double geodesicDistance(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2){
    double theta1 = q1[0];
    double phi1 = q1[1];
    double psi1 = q1[2];
    double theta2 = q2[0];
    double phi2 = q2[1];
    double psi2 = q2[2];
    double distance = acos(cos(theta1)*cos(theta2) + sin(theta1)*sin(theta2)*cos(phi1 - phi2 + psi1 - psi2));
    return distance;
}

/// \brief Take a quaternion, and return the closest quaternion based on hopf coordinates
/// \param closest_q The quaternion to discretize (by reference) std::vector<double>
/// \param KDTree The KDTree object that contains the discretized hopf coordinates
/// \param psi_vec std::vector<double> The vector of psi values
void getClosestQuaternion(std::vector<double>& closest_q,
                          KDTree<Eigen::Vector3d>* KDTree,
                          const std::vector<double>& psi_vec){
    // TODO: Have to fix the KDTree!!!!
    Eigen::Vector3d hopf;
    Eigen::Quaterniond q(closest_q[3], closest_q[0], closest_q[1], closest_q[2]);
    quaternionToHopf(q, hopf);
//    Eigen::Vector3d closest_hopf = KDTree->nearestNeighbor(hopf);

    int psi_ind = findNearestPoint(hopf[2], psi_vec);
//    Eigen::Quaterniond closest_q_eigen;
//    hopfToQuaternion(Eigen::Vector3d(closest_hopf[0], closest_hopf[1], psi_vec[psi_ind]), closest_q_eigen);

    // make sure for debugging that this is really the nearwst
    int i {0}; double min_distance = std::numeric_limits<double>::max();
    for (int j {0}; j < KDTree->getData().size() ; ++j){
        Eigen::Vector3d hopf_3d;
        hopf_3d[0] = KDTree->getData()[j][0];
        hopf_3d[1] = KDTree->getData()[j][1];
        hopf_3d[2] = psi_vec[psi_ind];

        auto dis = geodesicDistance(hopf, hopf_3d);
        if (dis < min_distance){
            min_distance = dis;
            i = j;
        }
    }
//    std::cout << "The nearest neighbor found by the KDTree is: " << closest_hopf << std::endl;
//    std::cout << "The nearest neighbor is: " << KDTree->getData()[i] << std::endl;
    // check if the nearest neighbor is the same as the one found by the KDTree
//    if (closest_hopf != KDTree->getData()[i]){
//        std::cout << "The nearest neighbor is not the same as the one found by the KDTree" << std::endl;
//    }
    Eigen::Quaterniond closest_q_eigen;

//    hopfToQuaternion(Eigen::Vector3d(closest_hopf[0], closest_hopf[1], psi_vec[psi_ind]), closest_q_eigen);
    hopfToQuaternion(Eigen::Vector3d(KDTree->getData()[i][0], KDTree->getData()[i][1], psi_vec[psi_ind]), closest_q_eigen);
    closest_q = {closest_q_eigen.x(), closest_q_eigen.y(), closest_q_eigen.z(), closest_q_eigen.w()};
}


/// \brief Take the hopf coordinate and return the nearest neighbor in the discretized hopf coordinates
/// \param hopf The hopf coordinates
/// \param KDTree The KDTree object that contains the discretized hopf coordinates
/// \param psi_vec std::vector<double> The vector of psi values
void getClosestHopf(Eigen::Vector3d& hopf,
                    KDTree<Eigen::Vector3d>* KDTree,
                    const std::vector<double>& psi_vec){
//    Eigen::Vector3d closest_hopf = KDTree->nearestNeighbor(hopf);
//    int psi_ind = findNearestPoint(hopf[2], psi_vec);
//    hopf = Eigen::Vector3d(closest_hopf[0], closest_hopf[1], psi_vec[psi_ind]);
    int psi_ind = findNearestPoint(hopf[2], psi_vec);
    int i {0}; double min_distance = std::numeric_limits<double>::max();
    for (int j {0}; j < KDTree->getData().size() ; ++j){
        Eigen::Vector3d hopf_3d;
        hopf_3d[0] = KDTree->getData()[j][0];
        hopf_3d[1] = KDTree->getData()[j][1];
        hopf_3d[2] = psi_vec[psi_ind];

        auto dis = geodesicDistance(hopf, hopf_3d);
        if (dis < min_distance){
            min_distance = dis;
            i = j;
        }
    }
    hopf[0] = KDTree->getData()[i][0]; hopf[1] = KDTree->getData()[i][1]; hopf[2] = psi_vec[psi_ind];
}



/// \brief Profile the trajectory
/// \param start The start joint state. type: stateType
/// \param goal The goal joint state. type: stateType
/// \param trajectory a vector of joint states. type: std::vector<stateType>
/// \param move_group_ The move group object. type: moveit::planning_interface::MoveGroupInterface
/// \param trajectory_msg The output trajectory. type: moveit_msgs::RobotTrajectory
/// \return success bool
bool profileTrajectory(const stateType & start,
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
std::shared_ptr<distance_field::PropagationDistanceField> getDistanceFieldMoveIt(double df_size_x = 3.0,
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
bool isPointInCollision(const distance_field::PropagationDistanceField &df, const Eigen::Vector3d &point){
    int x, y, z;
    df.worldToGrid(point[0], point[1], point[2], x, y, z);
    return df.getCell(x, y, z).distance_square_ == 0;
}

/// \brief Check if cell is occupied
/// \param df The distance field
/// \param cell The cell to check
/// \return True if the cell is occupied
bool isCellOccupied(const distance_field::PropagationDistanceField &df, const Eigen::Vector3i &cell){
    return df.getCell(cell[1], cell[1], cell[2]).distance_square_ == 0;
}

/// \brief Count occupied cells in the distance field
/// \param df The distance field
/// \return The number of occupied cells
unsigned int countOccupiedCells(const distance_field::PropagationDistanceField &df){
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

int ring_above(double z) {
        int nside_ {8};
        double za = fabs(z);
        if (za > 2./3.) {
            int iring = int(nside_*sqrt(3.*(1. - za)));
            return (z > 0) ? iring : 4*nside_ - iring - 1;
        }
        else {
            return int (nside_*(2. - 1.5*za));
        }
}

void in_ring(int iz, double phi0, double dphi,
             std::vector<int> &listir) {
    int nside_ {8};
    auto npface_ = nside_*nside_;
    auto ncap_   = (npface_-nside_)<<1;
    auto npix_   = 12*npface_;
    int nr, ir, ipix1;
    double shift=0.5;
    if (iz<nside_) // north pole
    {
        ir = iz;
        nr = ir*4;
        ipix1 = 2*ir*(ir-1);        //    lowest pixel number in the ring
    }
    else if (iz>(3*nside_)) // south pole
    {
        ir = 4*nside_ - iz;
        nr = ir*4;
        ipix1 = npix_ - 2*ir*(ir+1); // lowest pixel number in the ring
    }
    else // equatorial region
    {
        ir = iz - nside_ + 1;           //    within {1, 2*nside + 1}
        nr = nside_*4;
        if ((ir&1)==0) shift = 0;
            ipix1 = ncap_ + (ir-1)*nr; // lowest pixel number in the ring
    }
    int ipix2 = ipix1 + nr - 1;       //    highest pixel number in the ring
    // ----------- constructs the pixel list --------------
    if (dphi > (M_PI-1e-7))
    for (int i=ipix1; i<=ipix2; ++i) listir.push_back(i);
    else
    {
    int ip_lo = (int)std::floor(nr*(1.0 / (2.0*M_PI))*(phi0-dphi) - shift)+1;
    int ip_hi = (int)std::floor(nr*(1.0 / (2.0*M_PI))*(phi0+dphi) - shift);
    int pixnum = ip_lo+ipix1;
    if (pixnum<ipix1) pixnum += nr;
    for (int i=ip_lo; i<=ip_hi; ++i, ++pixnum)
      {
      if (pixnum>ipix2) pixnum -= nr;
      listir.push_back(pixnum);
      }
    }
}

void query_disc(const std::vector<double> &ptg, double radius,
                 std::vector<int>& listpix) {
        int nside_ {8};
        auto npface_ = nside_*nside_;
        auto ncap_   = (npface_-nside_)<<1;
        auto npix_   = 12*npface_;
        auto fact2_  = 4./npix_;
        auto fact1_  = (nside_<<1)*fact2_;
        listpix.clear();

        double dth1 = fact2_;
        double dth2 = fact1_;
        double cosang = cos(radius);

        double z0 = cos(ptg[0]);
        double xa = 1./sqrt((1-z0)*(1+z0));

        double rlat1  = ptg[0] - radius;
        double zmax = cos(rlat1);
        int irmin = ring_above (zmax)+1;

        if (rlat1<=0) // north pole in the disc
            for (int m=1; m<irmin; ++m) // rings completely in the disc
            in_ring(m, 0, M_PI, listpix);

        double rlat2  = ptg[0] + radius;
        double zmin = cos(rlat2);
        int irmax = ring_above (zmin);

        // ------------- loop on ring number ---------------------
        for (int iz=irmin; iz<=irmax; ++iz) // rings partially in the disc
        {
            double z;
            if (iz<nside_) // north polar cap
                z = 1.0 - iz*iz*dth1;
            else if (iz <= (3*nside_)) // tropical band + equat.
                z = (2*nside_-iz) * dth2;
            else
                z = -1.0 + (4*nside_-iz)*(4*nside_-iz)*dth1;

            // --------- phi range in the disc for each z ---------
            double x = (cosang-z*z0)*xa;
            double ysq = 1-z*z-x*x;

    //        planck_assert(ysq>=0, "error in query_disc()");

            double dphi=atan2(sqrt(ysq),x);
            in_ring (iz, ptg[1], dphi, listpix);
        }

        if (rlat2>=M_PI) // south pole in the disc
            for (int m=irmax+1; m<(4*nside_); ++m)  // rings completely in the disc
                in_ring (m, 0, M_PI, listpix);
//
//        if (scheme_==NEST)
//            for (unsigned int m=0; m<listpix.size(); ++m)
//                listpix[m] = ring2nest(listpix[m]);
}

void nest2xyf (int pix, int &ix, int &iy, int &face_num) const
  {
  face_num = pix>>(2*order_);
  pix &= (npface_-1);
  int raw = (pix&0x5555) | ((pix&0x55550000)>>15);
  ix = ctab[raw&0xff] | (ctab[raw>>8]<<4);
  pix >>= 1;
  raw = (pix&0x5555) | ((pix&0x55550000)>>15);
  iy = ctab[raw&0xff] | (ctab[raw>>8]<<4);
  }
int xyf2nest (int ix, int iy, int face_num) const
  {
  return (face_num<<(2*order_)) +
      (utab[ix&0xff] | (utab[ix>>8]<<16)
    | (utab[iy&0xff]<<1) | (utab[iy>>8]<<17));
  }
void ring2xyf (int pix, int &ix, int &iy, int &face_num) const
  {
  int iring, iphi, kshift, nr;
  int nl2 = 2*nside_;
  if (pix<ncap_) // North Polar cap
    {
    iring = int(0.5*(1+isqrt(1+2*pix))); //counted from North pole
    iphi  = (pix+1) - 2*iring*(iring-1);
    kshift = 0;
    nr = iring;
    face_num=0;
    int tmp = iphi-1;
    if (tmp>=(2*iring))
      {
      face_num=2;
      tmp-=2*iring;
      }
    if (tmp>=iring) ++face_num;
    }
  else if (pix<(npix_-ncap_)) // Equatorial region
    {
    int ip = pix - ncap_;
    if (order_>=0)
      {
      iring = (ip>>(order_+2)) + nside_; // counted from North pole
      iphi  = (ip&(4*nside_-1)) + 1;
      }
    else
      {
      iring = (ip/(4*nside_)) + nside_; // counted from North pole
      iphi  = (ip%(4*nside_)) + 1;
      }
    kshift = (iring+nside_)&1;
    nr = nside_;
    unsigned int ire = iring-nside_+1;
    unsigned int irm = nl2+2-ire;
    int ifm, ifp;
    if (order_>=0)
      {
      ifm = (iphi - ire/2 + nside_ -1) >> order_;
      ifp = (iphi - irm/2 + nside_ -1) >> order_;
      }
    else
      {
      ifm = (iphi - ire/2 + nside_ -1) / nside_;
      ifp = (iphi - irm/2 + nside_ -1) / nside_;
      }
    if (ifp == ifm) // faces 4 to 7
      face_num = (ifp==4) ? 4 : ifp+4;
    else if (ifp<ifm) // (half-)faces 0 to 3
      face_num = ifp;
    else // (half-)faces 8 to 11
      face_num = ifm + 8;
    }
  else // South Polar cap
    {
    int ip = npix_ - pix;
    iring = int(0.5*(1+isqrt(2*ip-1))); //counted from South pole
    iphi  = 4*iring + 1 - (ip - 2*iring*(iring-1));
    kshift = 0;
    nr = iring;
    iring = 2*nl2-iring;
    face_num=8;
    int tmp = iphi-1;
    if (tmp>=(2*nr))
      {
      face_num=10;
      tmp-=2*nr;
      }
    if (tmp>=nr) ++face_num;
    }
  int irt = iring - (jrll[face_num]*nside_) + 1;
  int ipt = 2*iphi- jpll[face_num]*nr - kshift -1;
  if (ipt>=nl2) ipt-=8*nside_;
  ix =  (ipt-irt) >>1;
  iy =(-(ipt+irt))>>1;
  }
int xyf2ring (int ix, int iy, int face_num) const
  {
  int nl4 = 4*nside_;
  int jr = (jrll[face_num]*nside_) - ix - iy  - 1;
  int nr, kshift, n_before;
  if (jr<nside_)
    {
    nr = jr;
    n_before = 2*nr*(nr-1);
    kshift = 0;
    }
  else if (jr > 3*nside_)
    {
    nr = nl4-jr;
    n_before = npix_ - 2*(nr+1)*nr;
    kshift = 0;
    }
  else
    {
    nr = nside_;
    n_before = ncap_ + (jr-nside_)*nl4;
    kshift = (jr-nside_)&1;
    }
  int jp = (jpll[face_num]*nr + ix - iy + 1 + kshift) / 2;
  if (jp>nl4)
    jp-=nl4;
  else
    if (jp<1) jp+=nl4;
  return n_before + jp - 1;
  }



#endif //MANIPULATION_PLANNING_UTILS_HPP
