# ADR-2: Technology Stack for Content and Examples

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-11
- **Feature:** 001-ros2-humanoid-control
- **Context:** The module aims to teach practical humanoid robotics with ROS 2 and requires robust, industry-standard tools for both content delivery and code example execution/simulation.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

The technology stack includes Python (3.10+) and ROS 2 (a recent stable distribution like Humble or Iron) for robotics programming. Docusaurus is chosen as the static site generator for the book content. Simulation environments will leverage Gazebo, NVIDIA Isaac Sim, and Unity.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

Leverages industry-standard tools, ensuring relevance and broad applicability of skills learned. Docusaurus provides a professional, extensible platform for technical documentation. The chosen simulators offer high-fidelity and diverse simulation capabilities essential for humanoid robotics.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

Creates dependency on multiple external tools and their respective versions, which may evolve or have specific system requirements. Docusaurus has a learning curve for advanced customization. NVIDIA Isaac Sim and Unity may require specific hardware, potentially limiting accessibility for some learners.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Book Platform Alternatives**:
    *   **MkDocs**: Rejected. Simpler and easier to set up, but offers less feature-richness and extensibility compared to Docusaurus, especially for large, structured technical books with advanced navigation, versioning, and search needs.
    *   **Jupyter Book**: Rejected. While excellent for code-centric documents and interactive content, its integration with pure Markdown content and customization options for a polished Docusaurus-like experience are less direct and flexible.
*   **Robotics Framework Alternatives**:
    *   **ROS 1**: Rejected. Considered outdated; ROS 2 is the current and future direction for robotics development, offering improved performance, security, and a more modern design, which aligns better with teaching current best practices.
    *   **Custom Robotics Framework**: Rejected. Reinventing the wheel to build a custom robotics framework would be highly inefficient, detract from the educational focus on existing tools, and negate the goal of teaching industry-standard practices.
*   **Simulator Alternatives**:
    *   **V-REP/CoppeliaSim**: Rejected. While capable, Gazebo, Isaac Sim, and Unity offer more widespread community support, better integration with ROS 2 (especially Isaac Sim), and are more prevalent in academic and industrial settings relevant to the target audience.

## References

- Feature Spec: specs/001-ros2-humanoid-control/spec.md
- Implementation Plan: specs/001-ros2-humanoid-control/plan.md
- Related ADRs: null
- Evaluator Evidence: null
