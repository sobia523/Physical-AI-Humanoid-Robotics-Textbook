# Chapter 3: Unity for High-Fidelity Rendering

This chapter explores Unity's capabilities for creating visually rich and interactive digital twins of humanoid robots. We will cover importing robot models, setting up realistic lighting and textures, and developing basic human-robot interaction scenarios within the Unity environment. A mini-lab will guide you through building a foundational Unity scene.

## 3.1 Importing Robots and Environments into Unity

Unity supports various 3D model formats (FBX, OBJ, etc.). For robots defined in URDF, you'll typically need to convert them or use specialized Unity packages. The Unity Robotics Hub provides tools like the URDF Importer to streamline this process.

### URDF Importer Package

1.  **Install Unity Robotics Hub's URDF Importer**:
    (Refer to `environment_setup.md` for installing Unity Robotics Hub packages via Package Manager. The URL for URDF Importer is `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.urdf-importer#main`).
2.  **Import URDF File**:
    -   Once installed, you can import a URDF file directly into your Unity project. Go to `Robotics > URDF Importer > Import URDF`.
    -   Select your `simple_humanoid.urdf` file (from `module2-digital-twin/gazebo_simulations/models/`).
    -   The importer will convert the URDF into a Unity Prefab with appropriate GameObjects, Colliders, and Rigidbodies.

### Best Practices for Model Import

-   **Scale**: Ensure your models are imported with the correct scale to match real-world units (Unity typically uses meters).
-   **Materials and Textures**: Verify that materials and textures are correctly applied. Sometimes, they need manual adjustment after import.
-   **Colliders**: The URDF Importer will generate colliders based on your URDF's collision geometry. Review and adjust these if necessary for accurate physics in Unity.
-   **Joints**: Unity's `ArticulationBody` component can be used for advanced robot joints, providing accurate physics simulation compatible with ROS 2 (via ROS-TCP-Connector).

## 3.2 Realistic Lighting, Textures, and Human-Robot Interaction Scenarios

Visual fidelity is key for high-fidelity digital twins. Unity offers extensive tools for lighting and materials.

### Lighting

-   **Directional Light**: Simulates the sun, providing global illumination. Adjust its rotation to control the direction of shadows.
-   **Point Lights / Spot Lights**: Used for local illumination or to highlight specific areas.
-   **Area Lights**: Provide soft, diffused lighting over a rectangular or disc area.
-   **Global Illumination (GI)**: Unity's GI systems (realtime or baked) simulate how light bounces off surfaces, contributing significantly to realism.
-   **Post-Processing**: Effects like Ambient Occlusion, Bloom, and Color Grading can dramatically enhance visual quality.

### Textures and Materials

-   **Standard Shader**: Unity's default shader is versatile, supporting various properties like Albedo (color/texture), Metallic, Smoothness, Normal Map, and Emission.
-   **PBR (Physically Based Rendering)**: Use PBR materials for realistic interaction with light. This requires textures like Albedo, Normal, Metallic, and Smoothness maps.
-   **Custom Shaders**: For highly specific visual effects, you can write custom shaders using Unity's Shader Graph or HLSL.

---
**Figure 3.1: Unity Lighting and Material Workflow Diagram**

*This diagram illustrates the workflow for achieving realistic visuals in Unity. It starts with 3D models and UV unwrapping. Then, various textures (Albedo, Normal, Metallic, Smoothness, AO) are applied to PBR materials. These materials are then assigned to models in a Unity scene. Light sources (Directional, Point, Spot) are added, and their properties (intensity, color, shadows) are configured. Finally, post-processing effects (e.g., Bloom, Ambient Occlusion, Color Grading) are applied to the camera to enhance the final rendered output.*
---

### Human-Robot Interaction (HRI) Scenarios

Unity is excellent for prototyping HRI.

-   **UI Elements**: Create interactive dashboards, control panels, or augmented reality overlays using Unity UI.
-   **Input Systems**: Utilize Unity's input system to allow users to interact with the robot (e.g., virtual joysticks, button presses).
-   **Animation**: Create complex robot animations (e.g., gestures, expressive movements) using Unity's animation system.
-   **Physics-based Interaction**: Allow users to directly manipulate objects in the scene, which the robot can then react to (e.g., pushing a block for the robot to pick up).

## 3.3 Visualizing Robot Sensors in Unity

While Gazebo provides the raw sensor data, Unity can offer a more intuitive and visually rich representation of that data.

-   **LiDAR Visualization**: Render simulated LiDAR points as individual spheres or lines in Unity.
-   **Camera Feeds**: Display camera feeds from simulated cameras directly onto UI textures or 3D planes in the scene.
-   **IMU Data**: Visualize IMU (Inertial Measurement Unit) data by applying rotations to the robot model in Unity, showing its orientation.
-   **Depth Cameras**: Render point clouds or color-coded depth maps.

---
**Figure 3.2: Unity Sensor Visualization Pipeline Diagram**

*This diagram illustrates how simulated sensor data from ROS 2 can be visualized in Unity. It starts with ROS 2 sensor topics (e.g., `/scan`, `/camera/image_raw`, `/imu/data`). These data streams are received in Unity (e.g., via ROS-TCP-Connector). A Unity script processes the incoming data (e.g., LiDAR points are converted to GameObjects, image data to textures, IMU data to rotations). Finally, the processed data is rendered within the Unity scene, providing a visual representation of the robot's perception.*
---

## 3.4 Mini-Lab: Create a Basic Interaction Scene with Humanoid Robot

In this mini-lab, you will:
1.  Import your `simple_humanoid.urdf` into Unity.
2.  Set up a basic scene with lighting.
3.  Implement a simple script to make the robot wave its arm or react to user input.

### Step 1: Prepare Unity Project

1.  Open your Unity project (`module2-digital-twin/unity_projects/`) from Unity Hub.
2.  Ensure the URDF Importer package is installed (refer to `environment_setup.md`).

### Step 2: Import Humanoid URDF

1.  In Unity, navigate to `Robotics > URDF Importer > Import URDF`.
2.  Select `module2-digital-twin/gazebo_simulations/models/simple_humanoid.urdf`.
3.  After import, the humanoid robot will appear as a Prefab in your Project window. Drag and drop it into your scene hierarchy.

### Step 3: Set Up Basic Scene

1.  Create a `Plane` GameObject for the ground (`GameObject > 3D Object > Plane`).
2.  Adjust the robot's position and scale if necessary to stand on the plane.
3.  Add a `Directional Light` (`GameObject > Light > Directional Light`) and adjust its rotation for realistic shadows.
4.  Optionally, add a `Post-process Volume` (requires Post Processing package from Package Manager) to enhance visuals.

### Step 4: Implement Basic Interaction Script (`BasicInteraction.cs`)

1.  In your Project window, navigate to `Assets/Scripts/`. If the `Scripts` folder doesn't exist, create it.
2.  Create a new C# script (`Assets/Scripts/BasicInteraction.cs`).

```csharp
using UnityEngine;
using System.Collections.Generic; // Required for List

public class BasicInteraction : MonoBehaviour
{
    public float waveSpeed = 1.0f;
    public float waveAngle = 45.0f; // degrees

    private ArticulationBody shoulderArticulation;
    private float initialShoulderPosition;

    void Start()
    {
        // Find the ArticulationBody for the shoulder_joint.
        // This assumes the shoulder joint is part of the imported URDF.
        // You might need to adjust the path based on your URDF's joint naming and hierarchy.
        // A robust way would be to search children of the robot's root GameObject.
        List<ArticulationBody> articulationBodies = new List<ArticulationBody>();
        gameObject.GetComponentsInChildren(true, articulationBodies); // Get all ArticulationBodies in children

        foreach (ArticulationBody ab in articulationBodies)
        {
            if (ab.name == "shoulder_joint") // Assuming the ArticulationBody's name matches the joint name
            {
                shoulderArticulation = ab;
                break;
            }
        }

        if (shoulderArticulation != null)
        {
            // Store the initial position of the shoulder joint
            // For a revolute joint, this is usually jointPosition[0]
            initialShoulderPosition = shoulderArticulation.jointPosition[0];
            Debug.Log("Found shoulder_joint ArticulationBody.");
        }
        else
        {
            Debug.LogError("Shoulder_joint ArticulationBody not found! Check URDF import and hierarchy.");
        }
    }

    void Update()
    {
        // Simple waving motion based on Sine wave
        if (shoulderArticulation != null)
        {
            float targetAngle = Mathf.Sin(Time.time * waveSpeed) * waveAngle;
            
            // Convert degrees to radians for ArticulationBody.jointPosition
            float targetPosition = targetAngle * Mathf.Deg2Rad; 

            // Set the target position for the shoulder joint
            var drive = shoulderArticulation.xDrive;
            drive.target = targetPosition;
            shoulderArticulation.xDrive = drive;
        }

        // Example: React to spacebar press (e.g., reset position)
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ResetShoulderPosition();
        }
    }

    void ResetShoulderPosition()
    {
        if (shoulderArticulation != null)
        {
            var drive = shoulderArticulation.xDrive;
            drive.target = initialShoulderPosition;
            shoulderArticulation.xDrive = drive;
            Debug.Log("Shoulder position reset.");
        }
    }
}
```

3.  Drag and drop the `BasicInteraction.cs` script onto the root GameObject of your imported humanoid robot in the Hierarchy window.
4.  Run the scene (press the Play button). You should see the robot's arm waving. Press Space to reset its position.