# Tasks: Module 2 — The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/003-digital-twin-module/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Test tasks are not explicitly requested in the feature specification, so they will not be generated. However, validation of examples and simulations is a core task.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All paths are relative to the project root: `Physical-AI-Humanoid-Robotics-Textbook/`

## Phase 1: Setup (Module Initialization)

**Purpose**: Initialize the module's directory structure and integrate it into the Docusaurus site.

- [x] T001 Create `module2-digital-twin` directory in the project root
- [x] T002 Create `module2-digital-twin/README.md`
- [x] T003 Create `module2-digital-twin/content/` directory
- [x] T004 Create `module2-digital-twin/ros2_packages/` directory
- [x] T005 Create `module2-digital-twin/gazebo_simulations/` directory
- [x] T006 Create `module2-digital-twin/unity_projects/` directory
- [x] T007 Create `module2-digital-twin/content/_category_.json` for Docusaurus sidebar
- [x] T008 Update `website/sidebars.ts` to include `module2-digital-twin` category

---

## Phase 2: Foundational (Environment & Docusaurus Integration)

**Purpose**: Establish core environments for development and ensure Docusaurus readiness for the module.

- [x] T009 [P] Document environment setup for ROS 2 (Humble/Iron) in a new markdown file (e.g., `module2-digital-twin/docs/environment_setup.md`)
- [x] T010 [P] Document environment setup for Gazebo (Humble/Iron) in `module2-digital-twin/docs/environment_setup.md`
- [x] T011 [P] Document environment setup for Unity 3D in `module2-digital-twin/docs/environment_setup.md`
- [x] T012 Create base `module2-digital-twin/ros2_packages/package.xml`
- [x] T013 Create base `module2-digital-twin/ros2_packages/setup.py`
- [x] T014 Create base `module2-digital-twin/ros2_packages/setup.cfg`
- [x] T015 Initialize a basic Unity project structure in `module2-digital-twin/unity_projects/`
- [x] T016 Initialize a basic Gazebo simulation structure in `module2-digital-twin/gazebo_simulations/`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 5 - Understand Digital Twin Concepts (Priority: P5) 🎯 MVP

**Goal**: Introduce fundamental concepts of digital twins in robotics, Gazebo, and Unity.

**Independent Test**: Student can accurately explain digital twin concepts and the roles of Gazebo/Unity.

### Implementation for User Story 5

- [x] T017 [US5] Write content for Chapter 1: Introduction to Digital Twin Simulation in `module2-digital-twin/content/chapter1.md`
- [x] T018 [US5] Add conceptual diagrams for digital twins and simulation pipelines to `module2-digital-twin/content/chapter1.md`
- [x] T019 [US5] Define key terminology related to digital twins in `module2-digital-twin/content/chapter1.md`

---

## Phase 4: User Story 1 - Simulate Humanoid Movement in Gazebo (Priority: P1)

**Goal**: Students can apply physics simulation principles in Gazebo for humanoid robot movement.

**Independent Test**: Successfully load a humanoid URDF in Gazebo and simulate simple movement with correct physical behavior.

### Implementation for User Story 1

- [x] T020 [P] [US1] Create content for Chapter 2: Gazebo Physics Simulation in `module2-digital-twin/content/chapter2.md`
- [x] T021 [P] [US1] Create a basic Gazebo world file (`.world`) for humanoid simulation in `module2-digital-twin/gazebo_simulations/worlds/basic_humanoid_world.world`
- [x] T022 [P] [US1] Create a sample humanoid URDF model in `module2-digital-twin/gazebo_simulations/models/simple_humanoid.urdf`
- [x] T023 [P] [US1] Develop a ROS 2 Python node to publish joint commands for the humanoid in `module2-digital-twin/ros2_packages/src/gazebo_humanoid_control/simple_joint_controller.py`
- [x] T024 [P] [US1] Create ROS 2 launch file for Gazebo world and humanoid controller in `module2-digital-twin/ros2_packages/launch/humanoid_gazebo.launch.py`
- [x] T024.1 [P] [US1] Create controller configuration file `module2-digital-twin/ros2_packages/config/simple_humanoid_controller.yaml`
- [x] T025 [US1] Document mini-lab steps for loading URDF and simulating movement in `module2-digital-twin/content/chapter2.md`
- [x] T026 [US1] Add diagrams illustrating Gazebo physics components in `module2-digital-twin/content/chapter2.md`

---

## Phase 5: User Story 2 - Visualize and Interact with Robots in Unity (Priority: P2)

**Goal**: Students can visualize and interact with robots in Unity with high-fidelity rendering.

**Independent Test**: Robot model imported into Unity, scene renders correctly, and basic interactivity works.

### Implementation for User Story 2

- [x] T027 [P] [US2] Create content for Chapter 3: Unity for High-Fidelity Rendering in `module2-digital-twin/content/chapter3.md`
- [x] T028 [P] [US2] Export/Prepare the humanoid URDF model for Unity import (e.g., convert to FBX, adjust materials)
- [x] T029 [P] [US2] Create a Unity scene for humanoid robot rendering in `module2-digital-twin/unity_projects/Assets/Scenes/HumanoidVizScene.unity`
- [x] T030 [P] [US2] Implement basic lighting and texture configurations in `module2-digital-twin/unity_projects/Assets/Scenes/HumanoidVizScene.unity`
- [x] T031 [P] [US2] Develop a Unity script for basic human-robot interaction (e.g., object manipulation) in `module2-digital-twin/unity_projects/Assets/Scripts/BasicInteraction.cs`
- [x] T032 [US2] Document mini-lab steps for Unity scene creation and interaction in `module2-digital-twin/content/chapter3.md`
- [x] T033 [US2] Add diagrams illustrating Unity rendering pipeline in `module2-digital-twin/content/chapter3.md`

---

## Phase 6: User Story 3 - Integrate Simulated Sensor Data with ROS 2 (Priority: P3)

**Goal**: Students can integrate simulated sensor data from Gazebo/Unity with ROS 2 topics.

**Independent Test**: Launch Gazebo simulation with sensors, verify ROS 2 topics are publishing data, and process with a simple ROS 2 node.

### Implementation for User Story 3

- [x] T034 [P] [US3] Create content for Chapter 4: Integrating Sensors and Perception in `module2-digital-twin/content/chapter4.md`
- [x] T035 [P] [US3] Update Gazebo humanoid URDF to include LiDAR, IMU, and depth camera sensors in `module2-digital-twin/gazebo_simulations/models/simple_humanoid_sensors.urdf`
- [x] T036 [P] [US3] Create ROS 2 launch file to bring up Gazebo with sensor-equipped humanoid and `ros_gz_bridge` in `module2-digital-twin/ros2_packages/launch/humanoid_sensors_gz.launch.py`
- [x] T037 [P] [US3] Develop a ROS 2 Python node to subscribe to simulated LiDAR data and perform basic obstacle detection in `module2-digital-twin/ros2_packages/src/perception/lidar_obstacle_detector.py`
- [x] T038 [P] [US3] Develop a ROS 2 Python node to subscribe to simulated IMU and depth camera data for visualization/processing in `module2-digital-twin/ros2_packages/src/perception/sensor_data_processor.py`
- [x] T039 [US3] Document hands-on steps for sensor integration and data processing in `module2-digital-twin/content/chapter4.md`
- [x] T040 [US3] Add diagrams illustrating ROS 2 - simulation sensor data flow in `module2-digital-twin/content/chapter4.md`

---

## Phase 7: User Story 4 - Complete Micro-Project: Simulated Humanoid Environment (Priority: P4)

**Goal**: Students integrate all concepts to build a comprehensive simulated humanoid environment.

**Independent Test**: Build a simulated room, deploy the robot, and verify basic navigation and object interaction.

### Implementation for User Story 4

- [x] T041 [P] [US4] Create content for Chapter 5: Micro-Project: Simulated Humanoid Environment in `module2-digital-twin/content/chapter5.md`
- [x] T042 [P] [US4] Design and create a simple indoor room Gazebo world in `module2-digital-twin/gazebo_simulations/worlds/small_room.world`
- [x] T043 [P] [US4] Integrate the sensor-equipped humanoid robot into the `small_room.world` Gazebo file
- [x] T044 [P] [US4] Develop a ROS 2 Python node for basic navigation in the simulated room (e.g., path following or simple obstacle avoidance) in `module2-digital-twin/ros2_packages/src/navigation/simple_navigator.py`
- [x] T045 [P] [US4] Develop a ROS 2 Python node for basic object interaction control in `module2-digital-twin/ros2_packages/src/interaction/object_manipulator.py`
- [x] T046 [US4] Document the micro-project steps, including building the environment, integrating the robot, and testing navigation/interaction in `module2-digital-twin/content/chapter5.md`
- [x] T047 [US4] Outline requirements for the lab report (screenshots, sensor plots, observations) in `module2-digital-twin/content/chapter5.md`
- [x] T048 [US4] Add diagrams for the micro-project environment and robot interaction in `module2-digital-twin/content/chapter5.md`

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Final review, quality assurance, and integration tasks.

- [x] T049 Review all Markdown content in `module2-digital-twin/content/` for clarity, accuracy, and Docusaurus compatibility. (Manual verification)
- [x] T050 Validate all code samples in `module2-digital-twin/ros2_packages/` for correctness and reproducibility. (Manual verification)
- [x] T051 Verify all Gazebo simulations in `module2-digital-twin/gazebo_simulations/` launch and run as expected. (Manual verification)
- [x] T052 Verify all Unity projects in `module2-digital-twin/unity_projects/` open and scenes render correctly. (Manual verification)
- [x] T053 Ensure all diagrams and screenshots are included (or text descriptions are adequate) in `module2-digital-twin/content/`. (Manual verification)
- [x] T054 Perform a final check of `module2-digital-twin/README.md` and `website/sidebars.ts` for correct integration. (Manual verification)
- [x] T055 Run Docusaurus build process locally to ensure the module integrates flawlessly with the overall website. (Manual verification)

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion.
    -   User stories can then proceed in parallel (if staffed)
    -   Or sequentially in priority order (P5 → P1 → P2 → P3 → P4). Note: User Story 5 is a conceptual introduction and can be written relatively early, while P1-P4 are more implementation-heavy.
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 5 (P5)**: Can start after Foundational (Phase 2) - No dependencies on other stories. (Conceptual content)
-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories. (Gazebo simulation)
-   **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May use output/models from US1 but should be independently testable. (Unity visualization)
-   **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Requires sensor-equipped Gazebo models (from US1 extension) and ROS 2 setup.
-   **User Story 4 (P4)**: Depends heavily on US1, US2, and US3 being largely complete as it integrates their concepts into a micro-project.

### Within Each User Story

-   Content creation (Markdown) can precede code development for a chapter.
-   Code examples and assets must be created before documentation of their use.
-   Core implementation tasks before integration tasks.

### Parallel Opportunities

-   All Setup tasks can run in parallel.
-   Several Foundational tasks can run in parallel.
-   Once Foundational phase completes, User Story 5 (conceptual content) and User Story 1 (Gazebo simulation) can begin in parallel.
-   User Story 2 (Unity visualization) could potentially start in parallel with US1 if the robot model export is independent.
-   Within each user story, tasks marked [P] can run in parallel.

---

## Parallel Example: User Story 1

```bash
# Content and initial Gazebo setup can proceed in parallel:
Task: "Create content for Chapter 2: Gazebo Physics Simulation in module2-digital-twin/content/chapter2.md"
Task: "Create a basic Gazebo world file (.world) for humanoid simulation in module2-digital-twin/gazebo_simulations/worlds/basic_humanoid_world.world"
Task: "Create a sample humanoid URDF model in module2-digital-twin/gazebo_simulations/models/simple_humanoid.urdf"

# ROS 2 development can run in parallel:
Task: "Develop a ROS 2 Python node to publish joint commands for the humanoid in module2-digital-twin/ros2_packages/src/gazebo_humanoid_control/simple_joint_controller.py"
Task: "Create ROS 2 launch file for Gazebo world and humanoid controller in module2-digital-twin/ros2_packages/launch/humanoid_gazebo.launch.py"
```

---

## Implementation Strategy

### MVP First (User Story 5 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 5 (Conceptual Introduction)
4.  **STOP and VALIDATE**: Ensure Chapter 1 content is clear and accurate.

### Incremental Delivery

1.  Complete Setup + Foundational → Module structure and base environments ready.
2.  Add User Story 5 → Chapter 1 content complete.
3.  Add User Story 1 → Chapter 2 content with Gazebo simulation.
4.  Add User Story 2 → Chapter 3 content with Unity visualization.
5.  Add User Story 3 → Chapter 4 content with ROS 2 sensor integration.
6.  Add User Story 4 → Chapter 5 micro-project integrating all.
7.  Each story adds value incrementally.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    *   Developer A: User Story 5 (Content)
    *   Developer B: User Story 1 (Gazebo simulation and ROS 2 control)
    *   Developer C: User Story 2 (Unity visualization and interaction)
3.  US3 and US4 would then build upon A, B, C's work.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
