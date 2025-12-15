# ROS 2 Workspace Setup for Module 3

This directory will contain all ROS 2 packages for Module 3.

## Initial Setup

1.  Create a ROS 2 workspace:
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    # Clone this repository into src/
    git clone <repository_url> .
    ```
2.  Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  Build the workspace:
    ```bash
    colcon build --packages-above-depth 1 # Adjust depth as needed
    ```
4.  Source the workspace:
    ```bash
    . install/setup.bash # or setup.zsh, setup.ps1
    ```

## `colcon test` Configuration

`colcon test` will be used for testing ROS 2 packages. To run it:
```bash
colcon test --packages-select <package_name>
colcon test-result # To see test results
```

## `pytest` Configuration

`pytest` will be used for Python unit testing. To run it:
```bash
# From the root of your ROS 2 package (e.g., in ros2_packages/src/your_package/)
pytest
```
Configuration (e.g., `pytest.ini`) will be placed in individual Python packages as needed.

## `ament_lint` Configuration

`ament_lint` will be used for code quality checks. To run it:
```bash
colcon test --packages-select <package_name> --ament-lint
```
Configuration files (e.g., `.flake8`, `mypy.ini`) will be placed in individual ROS 2 packages.
