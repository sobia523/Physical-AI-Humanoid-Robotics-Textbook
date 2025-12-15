# Tasks for Feature: Module 4 — Vision-Language-Action (VLA) Robotics

**Branch**: `004-vla-robotics-module` | **Date**: 2025-12-15 | **Plan**: [plan.md](plan.md)

This document outlines the development tasks for the VLA Robotics module, organized by implementation phase and user story.

## Phase 1: Project Setup & Configuration

- [ ] T001 Create the directory structure for the new module `module4-vla-robotics` as defined in the plan.
- [ ] T002 Create a new ROS 2 package named `vla_robotics_package` inside `module4-vla-robotics/ros2_packages`.
- [ ] T003 Define custom ROS 2 messages and services in `vla_robotics_package/msg` and `vla_robotics_package/srv` based on `contracts/ros2-interfaces.md`.
- [ ] T004 Create a basic launch file `vla_pipeline.launch.py` in `vla_robotics_package/launch` that will eventually launch all the nodes.

## Phase 2: Foundational Components

- [ ] T005 [P] Create the main simulation world file in `module4-vla-robotics/simulations/gazebo/` with a humanoid robot, a table, and some objects (e.g., a cube, a bottle).
- [ ] T006 [P] Create a Docusaurus placeholder file for the new module at `website/docs/module4-vla-robotics/index.md`.

## Phase 3: User Story 1 - Voice Command Transcription

**Goal**: As a student, I want to give a voice command to the robot simulation and see the transcribed text.

- [ ] T007 [US1] Implement the `voice_capture_node` in `vla_robotics_package/src/voice_capture_node.py` to capture audio from a microphone and publish it to the `/voice_audio` topic.
- [ ] T008 [US1] Implement the `voice_transcription_node` in `vla_robotics_package/src/voice_transcription_node.py` to subscribe to `/voice_audio`, use the OpenAI Whisper API to transcribe the audio, and publish the result to `/transcribed_text`.
- [ ] T009 [US1] Add the new nodes to the `vla_pipeline.launch.py` launch file.
- [ ] T010 [US1] Write a simple test script to subscribe to `/transcribed_text` and print the output to the console for verification.

## Phase 4: User Story 2 - Task Planning from Text

**Goal**: As a student, I want to provide a transcribed command and see the system generate a sequence of actions.

- [ ] T011 [US2] Implement the `cognitive_planner_node` in `vla_robotics_package/src/cognitive_planner_node.py` to subscribe to `/transcribed_text`.
- [ ] T012 [US2] In `cognitive_planner_node`, implement the logic to send the transcribed text to the OpenAI GPT API to generate a task plan.
- [ ] T013 [US2] Implement the service server for `/generate_task_plan` in `cognitive_planner_node`.
- [ ] T014 [US2] Publish the generated plan to the `/task_plan` topic.
- [ ] T015 [US2] Add the `cognitive_planner_node` to the `vla_pipeline.launch.py` launch file.

## Phase 5: User Story 3 - Autonomous Execution

**Goal**: As a student, I want to see the simulated humanoid robot autonomously execute the full sequence of actions.

- [ ] T016 [US3] Implement the `task_executor_node` in `vla_robotics_package/src/task_executor_node.py` to subscribe to `/task_plan`.
- [ ] T017 [US3] Implement the logic in `task_executor_node` to parse the task plan and call the appropriate ROS 2 actions (e.g., `/navigate_to`, `/manipulate_object`).
- [ ] T018 [US3] Implement the `perception_node` in `vla_robotics_package/src/perception_node.py` to detect objects in the simulation and publish their information to `/detected_objects`.
- [ ] T019 [US3] Implement the `manipulation_controller_node` action server in `vla_robotics_package/src/manipulation_controller_node.py` to handle the `/manipulate_object` action.
- [ ] T020 [US3] Add the remaining nodes to the `vla_pipeline.launch.py` launch file.

## Phase 6: Documentation & Polishing

- [ ] T021 [P] Write the content for the 5 chapters of Module 4 in `module4-vla-robotics/content/`, explaining the concepts and the implementation of each node.
- [ ] T022 [P] Create diagrams and visualizations for the VLA pipeline and add them to the documentation.
- [ ] T023 [P] Add the new module to the Docusaurus sidebar in `website/sidebars.ts`.
- [ ] T024 Finalize the `quickstart.md` with detailed instructions and screenshots.
- [ ] T025 Review and test the entire module for clarity, accuracy, and reproducibility.

## Dependencies

-   User Story 1 is a prerequisite for User Story 2.
-   User Story 2 is a prerequisite for User Story 3.
-   Foundational Components should be completed before starting User Story 1.

## Parallel Execution Examples

-   Within Phase 2, the simulation world (T005) and the Docusaurus placeholder (T006) can be created in parallel.
-   Within Phase 6, writing the chapter content (T021), creating diagrams (T022), and updating the sidebar (T023) can be done in parallel.

## Implementation Strategy

The implementation will follow the phases outlined above, starting with the foundational setup and then implementing each user story incrementally. This allows for testing and verification at each stage of the process, ensuring that the final integrated system works as expected. The MVP (Minimum Viable Product) is the completion of User Story 1, which provides the basic voice-to-text functionality.
