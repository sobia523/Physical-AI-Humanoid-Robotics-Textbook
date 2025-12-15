# ADR-8: Digital Twin Simulation Platform Strategy (Gazebo & Unity)

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-14
- **Feature:** 003-digital-twin-module
- **Context:** Module 2 focuses on teaching high-fidelity digital twin simulations of humanoid robots. This requires selecting and integrating appropriate simulation platforms that offer both accurate physics/sensor modeling and advanced rendering/interaction capabilities.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a dual-platform strategy utilizing Gazebo (Humble/Iron) for physics and sensor simulation, and Unity 3D for high-fidelity rendering, visualization, and human-robot interaction within Module 2.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

Leverages the strengths of each platform: Gazebo for robust robotics-centric physics and sensor models, Unity for superior graphical fidelity, advanced rendering pipelines, and rich interaction possibilities. Provides students with exposure to two widely used simulation tools, enhancing their versatility. Enables creation of visually compelling and technically accurate digital twin examples.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

Increases complexity in setup and content creation due to managing two distinct simulation environments. Requires a robust integration strategy (e.g., via ROS 2) to synchronize states and data between Gazebo and Unity. Potentially higher system resource requirements compared to a single-platform approach.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Solely use Gazebo for all aspects**: Rejected. While strong in physics and ROS 2 integration, Gazebo's rendering capabilities and tools for advanced human-robot interaction or visually rich environments are less developed compared to Unity. This would compromise the "high-fidelity" aspect of the digital twin.
*   **Solely use Unity (with a physics engine) for all aspects**: Rejected. While Unity has powerful rendering and a built-in physics engine, it may not be as optimized or mature for robotics-specific physics (e.g., complex joint dynamics, contact forces) and sensor simulation compared to Gazebo. Its ROS 2 integration for advanced robotics might also require more custom development.
*   **Use NVIDIA Isaac Sim**: Rejected. Explicitly out of scope for Module 2 (covered in Module 3). While offering integrated high-fidelity simulation, including physics and rendering, its complexity and focus align better with later, more advanced topics.

## References

- Feature Spec: specs/003-digital-twin-module/spec.md
- Implementation Plan: specs/003-digital-twin-module/plan.md
- Related ADRs: ADR-2 (Technology Stack for Content and Examples)
- Evaluator Evidence: null