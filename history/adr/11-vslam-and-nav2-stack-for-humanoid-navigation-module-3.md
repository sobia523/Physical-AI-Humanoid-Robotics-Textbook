# ADR-11: VSLAM and Nav2 Stack for Humanoid Navigation (Module 3)

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-14
- **Feature:** 002-isaac-ai-robot-brain
- **Context:** Module 3 aims to teach advanced perception and navigation for bipedal humanoid robots using NVIDIA Isaac tools. Selecting robust and performant VSLAM (Visual Simultaneous Localization and Mapping) and navigation frameworks is critical for enabling autonomous capabilities in simulation.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement VSLAM using NVIDIA Isaac ROS packages (e.g., `isaac_ros_vslam`) for hardware-accelerated localization and mapping. For path planning and execution, adapt and configure the Nav2 navigation stack (ROS 2) for bipedal humanoid robots within Isaac Sim.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

**Hardware-Accelerated Perception**: Isaac ROS VSLAM provides highly optimized and hardware-accelerated solutions, enabling real-time performance in complex simulated environments.
**Industry Standard Navigation**: Nav2 is a well-established and actively developed ROS 2 navigation framework, offering modularity, flexibility, and a rich set of features (planners, controllers, recovery behaviors).
**Integrated Pipeline**: Combining Isaac ROS with Nav2 creates a complete perception-to-action pipeline suitable for autonomous navigation, reflecting industry best practices.
**Learning Opportunities**: Exposes students to advanced robotics concepts and robust, production-grade frameworks.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

**Configuration Complexity**: Adapting Nav2 for bipedal humanoids can be complex due to unique locomotion constraints (e.g., balance, gait, foot placement) compared to wheeled robots. This may require significant tuning and custom development.
**Debugging Challenges**: Debugging a complex integrated VSLAM and Nav2 pipeline, especially with hardware acceleration, can be challenging.
**Performance Tuning**: Achieving optimal real-time performance might require careful tuning of both VSLAM and Nav2 parameters, which can be time-consuming.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Custom VSLAM Implementation**: Rejected. Developing VSLAM from scratch is a significant undertaking, requiring deep knowledge of computer vision and state estimation. This would shift the module's focus from using advanced tools to low-level algorithm development.
*   **Other Navigation Frameworks**: Rejected. While other navigation frameworks exist, Nav2 is the most prominent and feature-rich for ROS 2. Using a less mature or less integrated framework would reduce the educational value and practical applicability.
*   **Simplified Navigation Approaches**: Rejected. Relying on very simple navigation strategies (e.g., reactive obstacle avoidance only) would not adequately cover advanced path planning concepts and would limit the scope of the micro-project's autonomous capabilities.

## References

- Feature Spec: specs/002-isaac-ai-robot-brain/spec.md
- Implementation Plan: specs/002-isaac-ai-robot-brain/plan.md
- Related ADRs: ADR-2 (Technology Stack for Content and Examples), ADR-10 (NVIDIA Isaac Platform for AI Robotics (Module 3))
- Evaluator Evidence: null
