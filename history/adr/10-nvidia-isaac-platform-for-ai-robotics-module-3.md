# ADR-10: NVIDIA Isaac Platform for AI Robotics (Module 3)

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-14
- **Feature:** 002-isaac-ai-robot-brain
- **Context:** Module 3 focuses on advanced perception, navigation, and AI-driven control for humanoid robots, specifically leveraging NVIDIA Isaac tools. The choice of simulation and robotics software platform forms the core of this module's curriculum and practical exercises.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Utilize NVIDIA Isaac Sim as the primary photorealistic simulation environment and Isaac ROS for hardware-accelerated ROS 2 packages within Module 3. This combination serves as the core platform for teaching and implementing AI-driven robotics.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

**High Fidelity Simulation**: Isaac Sim provides photorealistic rendering and physically accurate simulations essential for training and testing advanced AI models.
**Synthetic Data Generation**: Enables generation of high-quality synthetic datasets (LiDAR, RGB-D) for training machine learning models, overcoming limitations of real-world data collection.
**Hardware Acceleration**: Isaac ROS provides pre-built, hardware-accelerated ROS 2 packages, optimizing performance for perception and navigation tasks on NVIDIA platforms.
**Industry Relevance**: Aligns with NVIDIA's growing ecosystem in robotics, exposing students to cutting-edge tools.
**Integrated Workflow**: Offers a streamlined workflow from simulation to accelerated ROS 2 components.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

**Hardware Requirements**: Requires specific NVIDIA GPU hardware, potentially limiting accessibility for some students.
**Learning Curve**: Isaac Sim and Isaac ROS can have a steeper learning curve compared to more general-purpose simulators or basic ROS 2 packages.
**Platform Lock-in**: Strong dependency on the NVIDIA ecosystem, which might not be transferable to all other robotics platforms.
**Installation Complexity**: Installation and setup can be more involved compared to simpler simulation environments.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Using Gazebo/Unity (as in Module 2) for advanced AI robotics**: Rejected. While versatile, these platforms lack the specific NVIDIA hardware acceleration and advanced synthetic data generation features inherent to Isaac Sim/ROS. They are not optimized for the scale of AI/ML training and high-performance robotics targeted by this module.
*   **Other specialized robotics simulation platforms**: Rejected. Alternatives might not offer the same level of integration with ROS 2, hardware acceleration, or the robust synthetic data generation capabilities provided by the NVIDIA Isaac ecosystem. Custom integration would be time-consuming and detract from learning objectives.

## References

- Feature Spec: specs/002-isaac-ai-robot-brain/spec.md
- Implementation Plan: specs/002-isaac-ai-robot-brain/plan.md
- Related ADRs: ADR-2 (Technology Stack for Content and Examples)
- Evaluator Evidence: null
