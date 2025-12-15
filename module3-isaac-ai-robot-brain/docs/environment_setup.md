# Environment Setup for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This document provides instructions for setting up the necessary development environments for Module 3, which involves NVIDIA Isaac Sim, Isaac ROS, and their integration with ROS 2.

## 1. NVIDIA Isaac Sim Setup

NVIDIA Isaac Sim is a scalable robotics simulation application and synthetic data generation tool built on NVIDIA Omniverse.

### Prerequisites
- **Operating System**: Ubuntu 20.04 or 22.04 LTS (recommended).
- **Hardware**: NVIDIA RTX GPU (40 series, 30 series, 20 series, or Quadro) with a minimum of 8 GB VRAM. NVIDIA drivers must be installed.
- **Docker / NVIDIA Container Toolkit**: Required for running Isaac Sim as a container.
- **Internet Connection**: Required for downloading Isaac Sim.

### Installation Steps (Containerized via Docker)

1.  **Install NVIDIA Container Toolkit**:
    Follow the official NVIDIA Container Toolkit documentation to install it for your Linux distribution. This enables Docker to access your NVIDIA GPU.

2.  **Install Docker**:
    Ensure Docker is installed and running. Add your user to the `docker` group:
    ```bash
    sudo usermod -aG docker $USER
    newgrp docker
    ```

3.  **Download Isaac Sim Container**:
    Isaac Sim is typically pulled from NVIDIA's NGC (NVIDIA GPU Cloud) registry. You will need an NGC API key.
    ```bash
    # Log in to NGC Docker registry (replace with your NGC API key)
    echo "YOUR_NGC_API_KEY" | docker login nvcr.io --username '$oauthtoken' --password-stdin

    # Pull the desired Isaac Sim container image (e.g., 2023.1.1)
    # Check NVIDIA's official documentation for the latest recommended tag.
    docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
    ```

4.  **Launch Isaac Sim**:
    You can launch Isaac Sim using a provided script or directly via Docker. A common way is to use the `run_scripts/docker_run.sh` script available after extracting the Isaac Sim package.

    Example direct Docker run:
    ```bash
    # Basic launch command (adjust paths and versions as needed)
    docker run --gpus all -it --rm \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --network host \
        nvcr.io/nvidia/isaac-sim:2023.1.1
    ```
    This will launch the Isaac Sim application. You might need to set up X-server forwarding or VNC for remote access.

### Verification
- Launch Isaac Sim.
- Open one of the example scenes (e.g., `Isaac Sim Examples > Hello World`).
- Ensure the simulation runs, and you can interact with the environment.

## 2. Isaac ROS Setup

Isaac ROS is a collection of hardware-accelerated packages for ROS 2, designed to improve performance on NVIDIA hardware.

### Prerequisites
-   **NVIDIA Jetson or GPU Workstation**: Isaac ROS requires NVIDIA hardware (Jetson or discrete GPU).
-   **ROS 2 (Humble/Iron)**: Ensure a compatible ROS 2 distribution is installed (as per Module 1 setup or your own).
-   **Docker / NVIDIA Container Toolkit**: Required for running Isaac ROS containers.

### Installation Steps (Containerized via Docker)

Isaac ROS typically runs within Docker containers, which provides a consistent and accelerated environment.

1.  **Install Docker and NVIDIA Container Toolkit**:
    Ensure these are installed and configured correctly (refer to Isaac Sim setup or official NVIDIA documentation).

2.  **Pull Isaac ROS Containers**:
    Isaac ROS provides various containers for different functionalities (e.g., `isaac_ros_common`, `isaac_ros_vslam`, `isaac_ros_nav2`). You'll typically pull the `isaac_ros_common` container and then build your specific ROS 2 workspaces within it or use specialized containers for applications.

    Example: Pulling common Isaac ROS base container
    ```bash
    docker pull nvcr.io/nvidia/isaac-ros/ros_humble/isaac_ros_common:3.0.0 # or appropriate version
    ```
    Refer to the official Isaac ROS documentation for specific module containers (e.g., `isaac_ros_vslam`, `isaac_ros_nav2`) and their versions.

3.  **Create a ROS 2 Workspace within the Container**:
    You often set up a development workspace inside the Isaac ROS container.
    ```bash
    # Example: Start a shell in the common container
    docker run --gpus all -it --rm -v /path/to/your/ros_ws:/ros_ws --network host nvcr.io/nvidia/isaac-ros/ros_humble/isaac_ros_common:3.0.0 bash
    
    # Inside the container:
    cd /ros_ws
    mkdir -p src
    # Clone Isaac ROS packages or your custom packages
    git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_visual_slam.git src/isaac_ros_visual_slam
    # Build your workspace
    colcon build
    source install/setup.bash
    ```

### Verification
- Launch one of the Isaac ROS example applications (e.g., a sample VSLAM pipeline if you've installed it).
- Verify that ROS 2 nodes are running and publishing expected data.

## 3. ROS 2 and Nav2 Setup

Nav2 (Navigation2) is the standard ROS 2 navigation stack used for autonomous mobile robot navigation. This section assumes you have a functional ROS 2 installation (Humble/Iron) as detailed in Module 1.

### Prerequisites
-   **ROS 2 (Humble/Iron)**: A working ROS 2 environment.
-   **System Dependencies**: Standard ROS 2 tools and libraries.

### Installation Steps

1.  **Install Nav2 Packages**:
    Nav2 packages are available through the ROS 2 apt repositories.
    ```bash
    sudo apt update
    sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup # For ROS 2 Humble
    # OR
    # sudo apt install -y ros-iron-navigation2 ros-iron-nav2-bringup # For ROS 2 Iron
    ```
    If you're using a specific Isaac ROS container, Nav2 might already be pre-installed or available as a separate Isaac ROS Nav2 container. Refer to Isaac ROS Nav2 documentation.

2.  **Verify Nav2 Installation**:
    ```bash
    ros2 pkg list | grep nav2
    ```
    You should see several `nav2_` packages listed.

### Adapting Nav2 for Bipedal Humanoids

Nav2 is primarily designed for wheeled robots. Adapting it for bipedal humanoids requires careful consideration of:
-   **Kinematics and Dynamics**: Humanoid specific plugins for robot control.
-   **Footprint**: Defining the robot's physical size and shape, which can change with gait.
-   **Planners and Controllers**: Customizing or developing planners and controllers that account for balance, stability, and specific walking patterns of bipedal robots. This often involves integrating with a whole-body controller.
-   **Odometry**: Providing stable and accurate odometry sources for humanoid movement.

These adaptations will involve modifying Nav2 configuration files and potentially developing custom ROS 2 nodes, which will be covered in Chapter 4.
