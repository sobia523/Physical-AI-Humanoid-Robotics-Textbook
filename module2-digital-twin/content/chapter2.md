# Chapter 2: Gazebo Physics Simulation

This chapter delves into setting up Gazebo worlds, simulating physics with gravity and collisions, and configuring various sensors like LiDAR, IMUs, and depth cameras to gather simulated data.

## Setting Up a Gazebo World

Gazebo worlds define the environment in which robots operate. They can include terrain, buildings, obstacles, and lighting.

To create a new Gazebo world, you typically define an SDF (Simulation Description Format) file. We've already created a basic `empty_world.world` in the previous foundational tasks.

<!-- Screenshot: A Gazebo window showing a simple environment, perhaps with a ground plane and a few basic shapes like boxes or spheres. A humanoid robot model is standing in the center of this world. -->


## Simulating Gravity, Collisions, and Robot Dynamics

Gazebo's strength lies in its accurate physics engine. When you load a robot model (defined by a URDF or SDF) into a Gazebo world, its physics properties (mass, inertia, collision geometry) are automatically simulated.

### Gravity

Gravity is enabled by default in Gazebo. Objects with mass will fall downwards unless supported.

### Collisions

Collision detection is crucial for realistic robot interaction with its environment. Each link in a robot model or object in the world can have a collision geometry defined. When these geometries intersect, Gazebo's physics engine calculates the forces to prevent interpenetration.

### Robot Dynamics

Robot dynamics refer to how a robot moves in response to forces and torques. This includes joint movements, balance, and interaction with external forces.

## Sensor Simulation: LiDAR, IMUs, Depth Cameras

Gazebo provides powerful sensor plugins that can mimic real-world sensors, generating data streams that ROS 2 nodes can subscribe to.

<!-- Screenshot: A composite image showing visualizations of different sensor outputs. For example, a 2D plot of LiDAR scan data, a 3D point cloud from a depth camera, and a graph showing IMU orientation data over time. Tools like RViz could be used to generate these visualizations. -->


### LiDAR (Light Detection and Ranging)

LiDAR sensors measure distances to objects by emitting laser pulses. Gazebo's LiDAR sensor plugin simulates this by casting rays and returning distance measurements.

### IMU (Inertial Measurement Unit)

IMUs measure a robot's orientation, angular velocity, and linear acceleration. Gazebo's IMU sensor plugin can simulate these outputs, often including realistic noise.

### Depth Cameras

Depth cameras provide depth information, typically as a point cloud or depth map. Gazebo's depth camera plugin can simulate this by rendering the scene from the camera's perspective and converting pixel distances to depth values.

## Mini-lab: Load a Humanoid URDF and Simulate Simple Movement

In this mini-lab, you will load the `humanoid.urdf` model into Gazebo and observe its behavior.

### Prerequisites

- You have a working ROS 2 and Gazebo installation.
- You have built the ROS 2 packages in this module by running `colcon build` from the `Physical-AI-Humanoid-Robotics-Textbook` directory.
- You have sourced the ROS 2 workspace: `source install/setup.bash`.

### Steps

1.  **Open a new terminal** and launch the Gazebo simulation:

    ```bash
    ros2 launch module2_digital_twin humanoid_simulation.launch.py
    ```

2.  **Observe the Simulation**: A Gazebo window should open, showing the humanoid robot in the empty world. The robot will likely be unstable and may fall over. This is expected as we have not yet implemented any control logic to keep it balanced.

3.  **Verify Robot Spawning**: Confirm that the humanoid robot model appears in the Gazebo world.

4.  **Check for Joint States (Optional)**: In a separate terminal, you can listen to the `/joint_states` topic to see the data being published from the simulation:

    ```bash
    ros2 topic echo /joint_states
    ```
    This will show the current position, velocity, and effort of each joint in the robot model.

5.  **Close the Simulation**: To stop the simulation, press `Ctrl+C` in the terminal where you launched it.

### Expected Outcome

- The Gazebo simulator launches and displays the humanoid robot.
- The robot model may fall or be unstable, demonstrating the effect of gravity and the lack of active control.
- You can (optionally) inspect the joint state data being published on the `/joint_states` topic.

