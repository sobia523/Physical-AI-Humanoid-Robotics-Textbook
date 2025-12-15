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