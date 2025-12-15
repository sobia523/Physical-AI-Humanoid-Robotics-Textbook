# Environment Setup for Module 2: Digital Twin

This document provides instructions for setting up the necessary development environments for Module 2, which involves ROS 2, Gazebo, and Unity 3D.

## 1. ROS 2 (Humble/Iron) Setup

### Prerequisites
- **Operating System**: Ubuntu 20.04 (Focal Fossa) or 22.04 (Jammy Jellyfish) is recommended for ROS 2. For Windows, official installation instructions are available but may have limitations.
- **Internet Connection**: Required for downloading packages.

### Installation Steps (Ubuntu)

1.  **Set up your sources.list**:
    ```bash
    sudo apt update && sudo apt install -y software-properties-common lsb-release
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

2.  **Install ROS 2 packages**:
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install -y ros-humble-desktop # For ROS 2 Humble
    # OR
    # sudo apt install -y ros-iron-desktop # For ROS 2 Iron
    ```

3.  **Source the setup script**:
    ```bash
    source /opt/ros/humble/setup.bash # For ROS 2 Humble
    # OR
    # source /opt/ros/iron/setup.bash # For ROS 2 Iron
    ```
    For convenience, you can add this line to your `~/.bashrc` file.

4.  **Install `rosdep`**:
    ```bash
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
    ```

5.  **Python dependencies**:
    ```bash
    sudo apt install -y python3-pip
    pip install -U colcon-common-extensions rosdepc
    ```

### Verification
- Open a new terminal and type `ros2 run demo_nodes_cpp talker`.
- Open another terminal and type `ros2 run demo_nodes_cpp listener`.
- You should see messages being published and subscribed to, verifying your ROS 2 installation.

## 2. Gazebo Setup

Gazebo is a powerful 3D robot simulator. It's often installed alongside ROS 2.

### Prerequisites
- **ROS 2 Installation**: Gazebo is tightly integrated with ROS 2. Ensure ROS 2 (Humble/Iron) is already installed.
- **Operating System**: Ubuntu (recommended), Windows or macOS (limited support).

### Installation Steps (Ubuntu - Integrated with ROS 2)

If you installed `ros-humble-desktop` or `ros-iron-desktop` as per the ROS 2 setup, Gazebo (specifically Gazebo Garden for Humble, or Gazebo Harmonic for Iron) should have been installed automatically. You can verify this by running:

```bash
gazebo
```

If it's not installed or you need a specific version, you can install it manually:

1.  **Add Gazebo repositories**:
    ```bash
    sudo apt update
    sudo apt install -y lsb-release wget gnupg curl
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-daily `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-daily.list'
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

2.  **Install Gazebo**:
    ```bash
    sudo apt update
    # For ROS 2 Humble (Gazebo Garden)
    sudo apt install -y ros-humble-gazebo-ros-pkgs # This installs Gazebo Garden and ROS-Gazebo bridge
    # OR for ROS 2 Iron (Gazebo Harmonic)
    sudo apt install -y ros-iron-gazebo-ros-pkgs # This installs Gazebo Harmonic and ROS-Gazebo bridge
    ```

3.  **Source ROS 2 setup**: Always ensure your ROS 2 environment is sourced.
    ```bash
    source /opt/ros/humble/setup.bash # or iron
    ```

### Verification
- Open a terminal and type `gazebo`. A Gazebo GUI window should appear.
- Open another terminal and type `ros2 run gazebo_ros_pkgs gazebo --ros-args -p use_sim_time:=true`. This launches Gazebo with ROS 2 integration.

## 3. Unity 3D Setup

Unity is a powerful cross-platform game engine often used for high-fidelity simulations and visualizations in robotics.

### Prerequisites
- **Operating System**: Windows, macOS, or Linux.
- **Hardware**: A capable graphics card is recommended for smooth Unity performance.

### Installation Steps

1.  **Download Unity Hub**:
    Go to the official Unity website (unity.com) and download Unity Hub. Unity Hub is a desktop application that helps you manage your Unity Projects and installations.

2.  **Install Unity Editor**:
    - Open Unity Hub.
    - Go to the 'Installs' tab.
    - Click 'Install Editor'.
    - Choose a recommended version (e.g., LTS release). Ensure it is compatible with the version mentioned in the module.
    - During installation, make sure to include the `Linux Build Support (IL2CPP)` component if you plan to integrate with ROS 2 on Linux, and `Windows Build Support (IL2CPP)` if on Windows.

3.  **Install ROS-Unity Integration Package (Unity Robotics Hub)**:
    - In Unity Hub, create a new 3D project.
    - Once the project is open, go to `Window > Package Manager`.
    - Click the `+` icon in the top-left corner and select `Add package from git URL...`.
    - Enter `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-connector#main` (or the appropriate branch/tag). This will install the ROS-TCP-Connector.
    - Explore the Unity Robotics Hub repository for additional packages like URDF Importer (`https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.urdf-importer#main`) for importing URDF files.

### Verification
- Create a new Unity 3D project through Unity Hub.
- Import a simple 3D model into your project.
- Play the scene to confirm everything is working as expected.
- For ROS-Unity integration, refer to the documentation for `ROS-TCP-Connector` within Unity for basic communication tests.
