---
sidebar_position: 3
title: "High-Fidelity Human-Robot Interaction in Unity"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>

# High-Fidelity Human-Robot Interaction in Unity

## Overview
This lesson explores how to leverage Unity for creating visually rich and interactive human-robot interaction scenarios. Learn to import robot models, set up realistic rendering, and design intuitive user interfaces for human-robot interaction.

## Learning Objectives
By the end of this lesson, you should be able to:
- Set up Unity for robotics simulation and visualization
- Import and configure robot models in Unity
- Implement realistic rendering and lighting for robot visualization
- Design user interfaces for human-robot interaction
- Create interactive elements for controlling robots in Unity
- Understand the integration possibilities between Unity and ROS/ROS 2

## Introduction to Unity for Robotics

### Why Unity for Robotics?
Unity provides several advantages for robotics applications:
- High-quality real-time rendering capabilities
- Extensive asset store with 3D models and environments
- Powerful animation and physics systems
- Cross-platform deployment options
- Strong community and extensive documentation
- Integration capabilities with ROS/ROS 2 through ROS# and other packages

### Unity Robotics Setup
To get started with robotics in Unity:
1. Install Unity Hub and a recent version of Unity (2021.3 LTS or newer recommended)
2. Create a new 3D project
3. Install the Unity Robotics packages via the Package Manager
4. Set up the necessary ROS/ROS 2 bridge if needed

## Importing Robot Models

### Supported Model Formats
Unity supports several 3D model formats:
- **FBX**: Most commonly used, supports animations and materials
- **OBJ**: Simple geometry, good for static models
- **DAE**: Collada format, supports complex scenes
- **GLTF/GLB**: Modern format with good performance

### Preparing Robot Models for Unity
Before importing, consider:
- Scale: Unity typically uses meters as units
- Axis orientation: Unity uses Y-up (ROS uses Z-up)
- Mesh optimization: Reduce polygon count for real-time performance
- Material preparation: Convert materials to Unity-compatible formats

### Import Process
1. Place your robot model files in the Assets folder
2. Unity will automatically import and process the model
3. Adjust import settings in the Inspector:
   - Scale Factor: Adjust to match real-world scale
   - Mesh Compression: Choose appropriate level
   - Read/Write Enabled: Enable if you need runtime mesh manipulation
   - Rig: Configure if the model has animations

### Example Robot Model Setup
```csharp
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public float moveSpeed = 5.0f;
    public float rotationSpeed = 100.0f;

    [Header("Joint Control")]
    public Transform[] joints;
    public float[] jointLimitsMin;
    public float[] jointLimitsMax;

    void Start()
    {
        // Initialize robot state
        InitializeRobot();
    }

    void Update()
    {
        // Handle robot movement and joint control
        HandleMovement();
        HandleJointControl();
    }

    void InitializeRobot()
    {
        // Set up initial robot configuration
        Debug.Log("Robot initialized with " + joints.Length + " joints");
    }

    void HandleMovement()
    {
        // Example: Basic movement with keyboard input
        float translation = Input.GetAxis("Vertical") * moveSpeed * Time.deltaTime;
        float rotation = Input.GetAxis("Horizontal") * rotationSpeed * Time.deltaTime;

        transform.Translate(0, 0, translation);
        transform.Rotate(0, rotation, 0);
    }

    void HandleJointControl()
    {
        // Example: Control joints with keyboard input
        for (int i = 0; i < joints.Length; i++)
        {
            if (Input.GetKey(KeyCode.Alpha1 + i))
            {
                // Move joint (example: rotate around local X-axis)
                float jointMovement = Input.GetAxis("Vertical") * Time.deltaTime;
                float currentAngle = joints[i].localEulerAngles.x;

                // Apply limits
                float newAngle = Mathf.Clamp(currentAngle + jointMovement,
                                           jointLimitsMin[i],
                                           jointLimitsMax[i]);

                joints[i].localEulerAngles = new Vector3(newAngle,
                                                         joints[i].localEulerAngles.y,
                                                         joints[i].localEulerAngles.z);
            }
        }
    }
}
```

## Realistic Rendering and Lighting

### Physically-Based Rendering (PBR)
Unity's PBR materials provide realistic surface appearance:
- **Albedo**: Base color of the surface
- **Metallic**: How metallic the surface appears
- **Smoothness**: How smooth/reflective the surface is
- **Normal Map**: Surface detail without geometry complexity
- **Occlusion**: Ambient light occlusion

### Setting Up Lighting
For realistic robot visualization:
```csharp
using UnityEngine;

public class RobotLightingSetup : MonoBehaviour
{
    [Header("Lighting Configuration")]
    public Light mainLight;
    public Light[] additionalLights;
    public Material robotMaterial;

    [Header("Environment")]
    public Material environmentMaterial;
    public ReflectionProbe reflectionProbe;

    void Start()
    {
        SetupLighting();
        ConfigureMaterials();
    }

    void SetupLighting()
    {
        // Configure main directional light
        if (mainLight != null)
        {
            mainLight.type = LightType.Directional;
            mainLight.intensity = 1.0f;
            mainLight.color = Color.white;
            mainLight.shadows = LightShadows.Soft;
        }

        // Configure additional lights (fill, rim, etc.)
        foreach (var light in additionalLights)
        {
            if (light != null)
            {
                light.enabled = true;
            }
        }

        // Set up reflection probe for realistic reflections
        if (reflectionProbe != null)
        {
            reflectionProbe.mode = ReflectionProbeMode.Realtime;
            reflectionProbe.size = new Vector3(10, 10, 10);
        }
    }

    void ConfigureMaterials()
    {
        // Apply PBR properties to robot material
        if (robotMaterial != null)
        {
            robotMaterial.SetColor("_Color", Color.gray);
            robotMaterial.SetFloat("_Metallic", 0.7f);  // Metallic surface
            robotMaterial.SetFloat("_Smoothness", 0.8f); // Smooth surface
        }
    }
}
```

### Post-Processing Effects
Enhance visual quality with post-processing:
- Ambient Occlusion: Adds depth and realism
- Bloom: Simulates bright light bleeding
- Color Grading: Adjusts overall color tone
- Depth of Field: Focuses attention on specific areas

## User Interface Design for Human-Robot Interaction

### Unity UI System
Unity's UI system provides flexible tools for creating interfaces:
- **Canvas**: Root object for all UI elements
- **Panels**: Organize UI elements
- **Buttons**: Interactive controls
- **Sliders**: Continuous value control
- **Text**: Display information
- **Images**: Visual feedback

### Example HRI Interface
```csharp
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;

public class RobotHRIController : MonoBehaviour
{
    [Header("UI Elements")]
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;
    public Slider speedSlider;
    public Text statusText;
    public Text jointAngleText;

    [Header("Robot Components")]
    public RobotController robotController;

    [Header("Events")]
    public UnityEvent onRobotMove;
    public UnityEvent onRobotStop;

    void Start()
    {
        SetupUI();
        UpdateStatus();
    }

    void SetupUI()
    {
        // Configure button events
        if (moveForwardButton != null)
            moveForwardButton.onClick.AddListener(() => MoveRobot(Vector3.forward));

        if (moveBackwardButton != null)
            moveBackwardButton.onClick.AddListener(() => MoveRobot(Vector3.back));

        if (turnLeftButton != null)
            turnLeftButton.onClick.AddListener(() => RotateRobot(-1));

        if (turnRightButton != null)
            turnRightButton.onClick.AddListener(() => RotateRobot(1));

        // Configure slider events
        if (speedSlider != null)
        {
            speedSlider.onValueChanged.AddListener(ChangeSpeed);
            speedSlider.minValue = 0.1f;
            speedSlider.maxValue = 10.0f;
            speedSlider.value = robotController.moveSpeed;
        }
    }

    void MoveRobot(Vector3 direction)
    {
        if (robotController != null)
        {
            // In a real implementation, this would send commands to the robot
            Debug.Log("Moving robot: " + direction);
            robotController.transform.Translate(direction * robotController.moveSpeed * Time.deltaTime);
            onRobotMove?.Invoke();
        }
        UpdateStatus();
    }

    void RotateRobot(float direction)
    {
        if (robotController != null)
        {
            // Rotate the robot
            robotController.transform.Rotate(0, direction * robotController.rotationSpeed * Time.deltaTime, 0);
            onRobotMove?.Invoke();
        }
        UpdateStatus();
    }

    void ChangeSpeed(float newSpeed)
    {
        if (robotController != null)
        {
            robotController.moveSpeed = newSpeed;
        }
        UpdateStatus();
    }

    void UpdateStatus()
    {
        if (statusText != null && robotController != null)
        {
            statusText.text = $"Status: Active | Speed: {robotController.moveSpeed:F1} m/s";
        }

        if (jointAngleText != null && robotController != null && robotController.joints.Length > 0)
        {
            // Display first joint angle as example
            jointAngleText.text = $"Joint 1: {robotController.joints[0].localEulerAngles.x:F1}�";
        }
    }

    public void StopRobot()
    {
        onRobotStop?.Invoke();
        UpdateStatus();
    }
}
```

## Interactive Elements

### Raycasting for Object Selection
Enable users to interact with specific robot parts:

```csharp
using UnityEngine;

public class RobotInteraction : MonoBehaviour
{
    [Header("Interaction Settings")]
    public LayerMask robotLayer;
    public float interactionDistance = 10f;
    public Material selectedMaterial;
    public Material defaultMaterial;

    private GameObject selectedObject;
    private Material originalMaterial;

    void Update()
    {
        HandleMouseInteraction();
    }

    void HandleMouseInteraction()
    {
        if (Input.GetMouseButtonDown(0)) // Left click
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, interactionDistance, robotLayer))
            {
                SelectObject(hit.collider.gameObject);
            }
            else
            {
                DeselectObject();
            }
        }
    }

    void SelectObject(GameObject obj)
    {
        // Deselect previous object
        DeselectObject();

        selectedObject = obj;
        originalMaterial = selectedObject.GetComponent<Renderer>().material;
        selectedObject.GetComponent<Renderer>().material = selectedMaterial;

        Debug.Log("Selected: " + selectedObject.name);
    }

    void DeselectObject()
    {
        if (selectedObject != null)
        {
            selectedObject.GetComponent<Renderer>().material = originalMaterial;
            selectedObject = null;
        }
    }

    public GameObject GetSelectedObject()
    {
        return selectedObject;
    }
}
```

## ROS/ROS 2 Integration

### Unity Robotics Package
The Unity Robotics Package provides tools for ROS/ROS 2 integration:
- ROS-TCP-Connector: Enables communication between Unity and ROS
- Message handling: Automatic serialization/deserialization of ROS messages
- Transform synchronization: Keep Unity and ROS coordinate systems in sync

### Basic ROS Communication Example
```csharp
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class ROSRobotController : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Control")]
    public string cmdVelTopic = "/cmd_vel";
    public string jointStateTopic = "/joint_states";

    private ROSConnection ros;
    private Vector3 targetVelocity;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIPAddress, rosPort);

        // Subscribe to topics if needed
        // ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
    }

    void Update()
    {
        // Send velocity commands periodically
        SendVelocityCommand();
    }

    void SendVelocityCommand()
    {
        // Create and send a velocity command message
        var twist = new TwistMsg();
        twist.linear = new Vector3Msg(targetVelocity.x, targetVelocity.y, targetVelocity.z);
        twist.angular = new Vector3Msg(0, 0, 0); // For rotation

        ros.Publish(cmdVelTopic, twist);
    }

    public void SetTargetVelocity(Vector3 velocity)
    {
        targetVelocity = velocity;
    }

    void OnJointStateReceived(JointStateMsg msg)
    {
        // Handle received joint state data
        Debug.Log("Received joint state with " + msg.name.Length + " joints");
    }
}
```

## Performance Optimization

### Level of Detail (LOD)
For complex robot models:
```csharp
using UnityEngine;

[RequireComponent(typeof(LODGroup))]
public class RobotLODController : MonoBehaviour
{
    public LODGroup lodGroup;
    public float[] lodDistances = { 10f, 30f, 50f }; // Distances for each LOD

    void Start()
    {
        if (lodGroup == null)
            lodGroup = GetComponent<LODGroup>();

        SetupLOD();
    }

    void SetupLOD()
    {
        LOD[] lods = new LOD[lodDistances.Length];

        for (int i = 0; i < lodDistances.Length; i++)
        {
            // Create LODs with decreasing detail
            // This is a simplified example - actual implementation would vary
            lods[i] = new LOD(1.0f - (i * 0.3f), new Renderer[0]); // Placeholder
        }

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
}
```

### Occlusion Culling
Enable occlusion culling to improve performance in complex scenes:
- Go to Window > Rendering > Occlusion Culling
- Bake occlusion data for your scene
- Unity will automatically hide objects not visible to the camera

## Hands-On Exercise
1. Create a new Unity 3D project named "RobotHRI_Simulation"
2. Import a simple robot model (or create basic primitives to represent a robot)
3. Set up a basic scene with lighting and environment
4. Create a UI panel with controls for moving the robot
5. Implement basic movement controls using keyboard input
6. Add a simple interaction system that highlights robot parts when clicked
7. (Advanced) Set up a basic ROS connection using the Unity Robotics Package

## Troubleshooting Tips
- Ensure your robot model is properly scaled (Unity typically uses meters)
- Check that materials and textures are correctly imported
- Verify that colliders are properly configured for interaction
- Monitor frame rate and optimize if performance is poor
- Check ROS connection parameters if integrating with ROS/ROS 2

## Summary
This lesson covered creating high-fidelity human-robot interaction in Unity. You learned how to import robot models, set up realistic rendering, design user interfaces, and implement interactive elements. The next lesson will explore bridging Gazebo/ROS 2 with Unity for enhanced simulation capabilities.
</ChapterTranslator>