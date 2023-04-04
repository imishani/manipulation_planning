//
// Created by itamar on 4/3/23.
//

#include <manipulation_planning/test.hpp>
#include <manipulationActionSpace.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    manipulationType type;
    // get the path to the library (manipulation_planning)

    auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
    std::cout << full_path << std::endl;
    auto vec = type.readMPfile(full_path.string() + "/config/manip.mprim");

    return 0;
}