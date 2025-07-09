# OHMS-NetSim: Outdoor Heterogeneous Multi-Robot System Network Simulator

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-orange)](https://gazebosim.org/docs/fortress)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](./LICENSE)

OHMS-NetSim is a high-fidelity simulation framework for developing and testing algorithms for Outdoor Heterogeneous Multi-Robot Systems. Built on ROS 2 Humble and Gazebo, it provides a realistic platform for researchers to validate navigation, communication, and coordination strategies in challenging outdoor environments.

The key feature of OHMS-NetSim is its custom Gazebo plugin that simulates realistic, obstacle-dependent communication degradation based on established scientific models, enabling the study of communication-aware robotics.

![OHMS-NetSim Forest Environment](./docs/images/ohms_sim_screenshot.png)

---

## Table of Contents
- [Key Features](#key-features)
- [Installation](#installation)
- [Getting Started: Running a Simulation](#getting-started-running-a-simulation)
- [Project Structure](#project-structure)
- [Community and Contribution](#community-and-contribution)
- [Citation](#citation)
- [Acknowledgements](#acknowledgements)

---

## Key Features

* **Heterogeneous Teams:** Simulate complex teams of Unmanned Ground Vehicles (UGVs) and Unmanned Aerial Vehicles (UAVs).
* **Realistic Outdoor Worlds:** Includes several pre-built environments, such as a marsyard and dense forests.
* **Modular Scenario Definition:** Easily define and launch complex multi-robot scenarios using a combination of YAML configuration files and ROS 2 launch files.
* **Scientifically-Grounded Comms Emulation:** A custom Gazebo plugin simulates path loss and packet drop rates based on peer-reviewed models, allowing for realistic testing of communication-aware algorithms.
* **ROS 2 Humble Integration:** Fully integrated with the ROS 2 ecosystem, providing a standard platform for robotics research and development.

---

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
    cd ~/ros2_ws
    ```

2.  **Clone the Repository:**
    Clone this repository into your workspace's `src` directory.
    ```bash
    git clone <your-repository-url> 
    ```

3.  **Install Dependencies:**
    Use `rosdep` to install any missing dependencies.
    ```bash
    rosdep install -i --from-path src -y --rosdistro humble
    ```

4.  **Build the Workspace:**
    Build the `ohms_sim` package using `colcon`.
    ```bash
    colcon build --packages-select ohms_sim
    ```

---

## Getting Started: Running a Simulation

Once the package has been built successfully, you can run the various simulation scenarios.

1.  **Source the Workspace:**
    Before running any ROS 2 commands, you must source the workspace's setup file in every new terminal.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  **Launch a Simulation:**
    Use the `ros2 launch` command to start a simulation. Several pre-configured scenarios are available.

    * **Example 1: Single UGV in a Forest**
        This will launch a single Husky UGV in the `forest` world.
        ```bash
        ros2 launch ohms_sim single_robot_sim.launch.py world:=forest
        ```

    * **Example 2: Dual UGV-UAV Team**
        This launches a team consisting of one Husky UGV and one X4 UAV.
        ```bash
        ros2 launch ohms_sim dual_robot_uav_ugv_sim.launch.py world:=forest
        ```

    * **Example 3: Large Heterogeneous Team**
        This launches a team of two UGVs and two UAVs in a larger, denser forest environment.
        ```bash
        ros2 launch ohms_sim multi_robot_2uav_2ugv_sim.launch.py world:=forest5x
        ```

---

## Project Structure

The repository is organised to allow for easy customisation and extension.

* `/launch`: Contains the top-level ROS 2 launch files for starting different simulation scenarios.
* `/config`: Holds the YAML files used to configure the parameters for each simulation, such as robot names, initial poses, and sensor loadouts.
* `/worlds`: Contains the Gazebo `.sdf` world files.
* `/models`: Contains the Gazebo `.sdf` model files for the robots and other objects in the world.
* `/src`: Contains the C++ source code for the `CommsEmulatorHelper` Gazebo plugin.
* `/docs`: Contains detailed project documentation, including the project roadmap.

To create a new scenario, you can create a new YAML file in the `/config` directory and a corresponding launch file in the `/launch` directory.

---

## Community and Contribution

We welcome contributions from the community! Whether you are reporting a bug, suggesting a new feature, or submitting code, your input is valuable. Please consult our community guidelines to get started.

* **[Project Roadmap](./docs/ROADMAP.md):** Outlines our phased release plan, from the current prototype to future stable versions.
* **[Contribution Guidelines](./CONTRIBUTING.md):** Explains how to contribute to the project.
* **[Code of Conduct](./CODE_OF_CONDUCT.md):** Our commitment to a welcoming and inclusive community.

---

## Citation

If you use OHMS-NetSim in your research, please cite our work.
*(Please add your publication details here once available)*

```bibtex
@inproceedings{your-lastname-2025-ohms,
  title     = {OHMS‑NetSim: Enabling Obstacle‑Aware Network‑Physics Co-Simulation for Outdoor Multi‑Robot Applications},
  author    = {Kalhan Boralessa, David Adama, Ahmad Lotfi, and João Filipe Ferreira},
  booktitle = {UKCI},
  year      = {2025},
  address   = {Edinburg, UK}
}
```

---

## Acknowledgements

The simulation worlds `marsyard2020`, `marsyard2020forest`, and `marsyard2020forest5x` are adapted from the [leo_simulator-ros2](https://github.com/LeoRover/leo_simulator-ros2) package, developed by the Mars Society Polska and the Leo Rover team. We extend our gratitude for their excellent work.

---
