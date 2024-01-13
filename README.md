# perspective_n_point_pose_computation

# Robot Perception ROS Workspace

This ROS workspace, named `robot_perception_ws`, contains a package named `robot_perception` with nodes for image points publishing and pose estimation.

## Getting Started

These instructions will guide you through setting up and running the nodes in this ROS workspace.

### Prerequisites

- ROS (Robot Operating System) installed. You can follow the installation instructions [here](http://wiki.ros.org/noetic/Installation).

### Installation

1. **Create a ROS Workspace:**
    ```bash
    mkdir -p ~/robot_perception_ws/src
    cd ~/robot_perception_ws/src
    catkin_init_workspace
    ```

2. **Create a Package and Nodes:**
    ```bash
    cd ~/robot_perception_ws/src
    catkin_create_pkg robot_perception rospy std_msgs sensor_msgs cv_bridge
    ```

3. **Place Nodes in the Package:**
    - Create two scripts, `image_points_publisher.py` and `pose_estimation_node.py`, inside the `src/robot_perception` directory.

4. **Edit CMakeLists.txt:**
    - Open `CMakeLists.txt` in `robot_perception` and modify it as follows:
        ```cmake
        find_package(catkin REQUIRED COMPONENTS
          rospy
          std_msgs
          sensor_msgs
          cv_bridge
        )

        catkin_python_setup()

        catkin_package()

        install(PROGRAMS
          scripts/image_points_pub.py
          scripts/pnp_sub.py
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        ```

5. **Build the Workspace:**
    ```bash
    cd ~/robot_perception_ws
    catkin_make
    ```

6. **Source the Workspace:**
    ```bash
    source devel/setup.bash
    ```

## Running the Nodes

Open multiple terminals:
1. Terminal 1: 
    ```bash
    roscore
    ```
2. Terminal 2: 
    ```bash
    rosrun robot_perception image_points_pub.py
    ```
3. Terminal 3: 
    ```bash
    rosrun robot_perception pnp_sub.py
    ```

Adjust the names, scripts, and dependencies based on your project's requirements.

## Author

- K M LINGAUDHAYA


