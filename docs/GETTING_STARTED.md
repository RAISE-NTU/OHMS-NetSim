# Getting Started 

## Installation

These instructions assume you are using **Ubuntu 22.04** with **ROS 2 Humble Hawksbill** installed.

### 1. Prerequisites

* **ROS 2 Humble:** Ensure you have a working installation of ROS 2 Humble. You can follow the official installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
* **Gazebo Fortress:** OHMS-NetSim uses Gazebo (formerly Ignition Gazebo). The required version, Fortress, is typically installed as a dependency of ROS 2 Humble (`ros-humble-ros-gz`).
* **Colcon:** The standard ROS 2 build tool.
* **Git:** For cloning the repository.

### 2. Build Instructions

1.  **Create a Colcon Workspace:**
    If you do not have one already, create a new ROS 2 workspace.
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone the Repository:**
    Clone this repository into your workspace's `src` directory.
    ```bash
    git clone https://github.com/RAISE-NTU/ohms_netsim.git
    ```

3.  **Install Dependencies:**
    Use `rosdep` to install any missing dependencies.
    ```bash
    rosdep install -i --from-path src -y --rosdistro humble
    ```

4.  **Build the Workspace:**
    Build the `ohms_netsim` package using `colcon`.
    ```
    cd ~/ros2_ws
    ```
    ```bash
    colcon build --packages-select ohms_netsim
    ```

---

## Running a Simulation

Once the package has been built successfully, you can run the various simulation scenarios.

1.  **Source the Workspace:**
    Before running any ROS 2 commands, you must source the workspace's setup file in every new terminal.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  **Launch a Simulation:**
    Use the `ros2 launch` command to start a simulation. Several pre-configured scenarios are available.

    * **Forest**
        This will launch the `forest` world.
        ```bash
        ros2 launch ohms_netsim flatforest.launch.py 
        ```

    * **Dual UGV Team**
        This launches a team consisting of one Husky UGV and one X4 UAV.
        ```bash
        ros2 launch ohms_netsim dual_robot_ugv_ugv_sim.launch.py 
        ```

        > **Tip:** Check the [`/launch`](./launch) folder for more simulation launch files and scenario options.

3. **Setup Docker**
    **[Setup Docker](./SETTING_UP_DOCKER.md)** 
    You have to identify veth names of docker network bridges for comms emulator (next step), easiest way is to create and run docker 
    container for each robot and check veth name using
    ```
    ip link show
    ```
    one after other.

4. **Comms Emulator**
    **[Comms Emulator](./COMMS_EMULATOR.md)**
---