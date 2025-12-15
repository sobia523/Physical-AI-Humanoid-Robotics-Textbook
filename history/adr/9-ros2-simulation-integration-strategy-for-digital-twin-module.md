# ADR-9: ROS 2 - Simulation Integration Strategy for Digital Twin Module

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-14
- **Feature:** 003-digital-twin-module
- **Context:** Module 2 requires robust and efficient communication between the chosen simulation platforms (Gazebo and Unity) and the ROS 2 robotics middleware to enable realistic sensor data processing and robot control. A clear integration strategy is crucial for reproducibility and student learning.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement ROS 2 integration for Module 2 using `ros_gz_bridge` for Gazebo-ROS 2 communication, and custom Python ROS 2 nodes for both sensor data processing and commanding robot actions. For Unity, develop custom ROS 2-Unity interfaces as needed (e.g., specific ROS 2 topics/services for data exchange). Action representation for robot control will primarily utilize standard ROS 2 topics and actions.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

Leverages `ros_gz_bridge` which is the official and well-supported tool for Gazebo-ROS 2 communication, ensuring stability and performance. Utilizing Python ROS 2 nodes simplifies development and learning for students familiar with Python. Standard ROS 2 topics and actions provide a consistent and extensible interface for robot control and perception.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

Custom Unity-ROS 2 interfaces may require more development effort and maintenance compared to a pre-built solution if one were available and suitable. Performance implications of Python ROS 2 nodes for high-frequency control loops might need careful consideration, potentially requiring C++ for demanding tasks (though less likely for an educational module). Dependency on specific versions of `ros_gz_bridge` and ROS 2, requiring careful version management.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Custom communication protocols (e.g., raw sockets, shared memory)**: Rejected. High development overhead, lacks the robustness, standardization, and ecosystem benefits of ROS 2. Would also complicate integration with the broader ROS 2 ecosystem.
*   **Other ROS 2 bridging tools**: Rejected. `ros_gz_bridge` is the recommended and actively maintained solution for Gazebo. For Unity, while some third-party plugins exist, a custom solution offers more control and learning opportunities within the context of the module.
*   **Focus solely on simulation-internal control**: Rejected. This would defeat the purpose of bridging the digital twin with the robotics middleware, which is a core learning objective for integrating simulated robots with real-world control architectures.

## References

- Feature Spec: specs/003-digital-twin-module/spec.md
- Implementation Plan: specs/003-digital-twin-module/plan.md
- Related ADRs: ADR-2 (Technology Stack for Content and Examples), ADR-8 (Digital Twin Simulation Platform Strategy (Gazebo & Unity))
- Evaluator Evidence: null