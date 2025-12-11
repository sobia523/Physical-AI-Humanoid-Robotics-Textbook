---
id: 1
title: Update project constitution
stage: constitution
date_iso: 2025-12-11
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["documentation", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: []
prompt_text: |-
  /sp.constitution

  # Project: Physical AI & Humanoid Robotics — AI/Spec-Driven Docusaurus Book

  ## Core Principles
  - **Accuracy:** All technical content, simulations, and code examples must be verified against authoritative sources, including official ROS 2 documentation, NVIDIA Isaac manuals, Gazebo/Unity guides, and peer-reviewed AI/robotics papers.
  - **Clarity:** Content should be precise, structured, and suitable for advanced learners in AI and robotics. Use clear headings, subheadings, diagrams, and step-by-step explanations.
  - **Reproducibility:** All code snippets, simulations, and examples must be testable and reproducible in ROS 2, Gazebo, Unity, or NVIDIA Isaac.
  - **Rigor:** Prefer peer-reviewed research, official documentation, and validated robotics/AI frameworks.
  - **Professional Presentation:** Book must be deployable as a polished Docusaurus site with diagrams, sidebars, navigation, and search functionality.

  ---

  ## Key Standards
  - **Source Verification:** Every factual claim or technical explanation must be traceable to authoritative sources.
  - **Citation Style:** APA format for all references.
  - **Source Types:** At least 50% of sources must be peer-reviewed or official documentation.
  - **Plagiarism:** 0% tolerance; all content must be original or properly cited.
  - **Writing Clarity:** Flesch-Kincaid grade 10–12 readability.
  - **Terminology Consistency:** Units, naming conventions, code formatting, and diagrams must follow robotics and AI best practices.

  ---

  ## Constraints
  - **Format:** Markdown files compatible with Docusaurus + Spec-Kit Plus.
  - **Structure:** Organize chapters under four modules, each with 3–5 chapters.
  - **Diagrams:** Include illustrations for ROS 2 nodes, digital twins, AI-brain architectures, and VLA pipelines.
  - **Code Examples:** All examples must run in ROS 2, Gazebo, NVIDIA Isaac, or Unity; include inline comments and step-by-step instructions.
  - **Capstone Project:** Autonomous humanoid robot that receives voice commands, plans paths, navigates obstacles, identifies objects with vision, and manipulates them.

  ---

  ## Module Overview

  ### Module 1: The Robotic Nervous System (ROS 2)
  - **Focus:** Middleware for robot control.
  - **Learning Objectives:**
    - ROS 2 nodes, topics, and services
    - Bridging Python agents to ROS controllers using `rclpy`
    - Understanding URDF (Unified Robot Description Format) for humanoids
  - **Deliverables:** Working ROS 2 examples, diagrams of node connections, sample URDF humanoid model

  ### Module 2: The Digital Twin (Gazebo & Unity)
  - **Focus:** Physics simulation and environment building
  - **Learning Objectives:**
    - Simulate physics, gravity, and collisions in Gazebo
    - High-fidelity rendering and human-robot interaction in Unity
    - Sensor simulation: LiDAR, depth cameras, IMUs
  - **Deliverables:** Simulated humanoid in Gazebo, Unity environment with interactive elements, sensor data examples

  ### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
  - **Focus:** Advanced perception and training
  - **Learning Objectives:**
    - NVIDIA Isaac Sim: photorealistic simulation and synthetic data generation
    - Isaac ROS: hardware-accelerated VSLAM and navigation
    - Nav2: path planning for bipedal humanoid movement
  - **Deliverables:** Navigation demos, AI perception pipelines, sample VSLAM implementations

  ### Module 4: Vision-Language-Action (VLA)
  - **Focus:** Convergence of LLMs and robotics
  - **Learning Objectives:**
    - Voice-to-action using OpenAI Whisper
    - Cognitive planning: converting natural language commands into ROS 2 action sequences
    - Capstone: integrate perception, planning, and action in a humanoid
  - **Deliverables:** Capstone autonomous humanoid project, demonstration videos/screenshots, integrated code

  ---

  ## Success Criteria
  - Technical accuracy verified against sources and simulation results
  - Reproducible code and working simulations for all modules
  - Book fully deployable as a Docusaurus site using Spec-Kit Plus
  - Clear diagrams and professional formatting
  - Zero plagiarism, all references cited in APA style
  - Expert-reviewed for clarity, completeness, and rigor

  ---

  ## Additional Notes
  - **Navigation:** Include sidebars, navbar, and search functionality in Docusaurus
  - **Design:** Professional, modern color theme suitable for technical books
  - **Documentation:** Include diagrams, tables, and charts to aid understanding
  - **Testing:** Validate code examples in simulation and real robot environments if available
response_text: |-
  The project constitution has been updated to version 1.0.0. This is a major version bump as a complete new constitution has been adopted, replacing the previous placeholder template.

  **Files flagged for manual follow-up:**
  The following files might contain outdated references or structures that need to be aligned with the new constitution's principles and standards. Please review them manually:
  *   `.specify/templates/plan-template.md`
  *   `.specify/templates/spec-template.md`
  *   `.specify/templates/tasks-template.md`
  *   `.gemini/commands/sp.adr.toml`
  *   `.gemini/commands/sp.analyze.toml`
  *   `.gemini/commands/sp.checklist.toml`
  *   `.gemini/commands/sp.clarify.toml`
  *   `.gemini/commands/sp.constitution.toml`
  *   `.gemini/commands/sp.git.commit_pr.toml`
  *   `.gemini/commands/sp.implement.toml`
  *   `.gemini/commands/sp.phr.toml`
  *   `.gemini/commands/sp.plan.toml`
  *   `.gemini/commands/sp.specify.toml`
  *   `.gemini/commands/sp.tasks.toml`

  **Suggested commit message:**
  ```
  docs: amend constitution to v1.0.0 (initial concrete constitution and governance)
  ```
outcome:
  evaluation: The constitution was successfully updated and a sync impact report was generated. The PHR was created as well.
  follow_up_actions: Manual review of listed files to ensure alignment with the new constitution.
---