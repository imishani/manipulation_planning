
# Manipulation Planning

---

#### Itamar Mishani

---

### Description:

This is a ROS package contains implementations of search-based planning algorithms for manipulation, with moveit configuration.

### Dependencies:
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
* [moveit](http://docs.ros.org/noetic/api/moveit_tutorials/html/index.html)
* [search](https://gitlab.com/imishani/search)
    * After installing, it is possible that running `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH ` will be needed.

### Installation:
1. Install ROS Humble
2. Install MoveIt!2 with `sudo apt-get install ros-noetic-moveit`.
3. Create a colcon (ament) workspace:

TODO the rest.
```
mkdir -p ~/manipulation_ws/src
cd ~/manipulation_ws/
```
4. Install search (see [here](https://gitlab.com/imishani/search) for instructions)
5. Clone this repository to your catkin workspace
```
git clone https://gitlab.com/imishani/manipulation_planning.git
```
6. Run the following command to install the package dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```
7. Build your catkin workspace
```
catkin build
```
8. Source your catkin workspace
```
source ~/manipulation_ws/devel/setup.bash
```

### Usage:
make you have moveit running. For example, run the running example from moveit tutorial:
```
roslaunch panda_moveit_config demo.launch
```

Then, you can refer to the [src](https://gitlab.com/imishani/manipulation_planning/-/tree/main/src) folder for examples of how to use the package.


