# ADR-3: Code Example and Diagram Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-11
- **Feature:** 001-ros2-humanoid-control
- **Context:** The educational module emphasizes practical application. Code examples and clear diagrams are crucial for demonstrating concepts, facilitating hands-on learning, and ensuring reproducibility of the material.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

All code examples will be fully reproducible in specified simulation environments (ROS 2, Gazebo, NVIDIA Isaac Sim, Unity) and accompanied by step-by-step instructions. Diagrams will be used extensively to illustrate complex concepts, such as ROS 2 node graphs, digital twins, AI architectures, and URDF structures. APA-compliant citations will be used for all factual claims and borrowed diagrams/code.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

Enhances learning by providing practical, verifiable examples. Improves comprehension of complex systems through visual aids. Ensures academic rigor and builds trust through proper citation. Directly addresses the project's core principles of Accuracy, Clarity, and Reproducibility.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

Requires significant effort to develop, test, and maintain all code examples across potentially evolving environments. Diagram creation can be time-consuming and requires specialized tools. Strict adherence to citation standards adds overhead to content creation.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Text-only Explanations**:
    *   **Why rejected**: Without code examples and diagrams, the practical and visual aspects of robotics would be severely hindered, making it difficult for learners to apply theoretical knowledge and understand system interactions.
*   **Non-Reproducible Code Snippets**:
    *   **Why rejected**: Code examples that do not run or are difficult to set up would frustrate learners, undermine confidence in the material, and directly violate the core principle of Reproducibility.
*   **Generic or Minimal Diagrams**:
    *   **Why rejected**: Diagrams that are too abstract or lack specific relevance to the humanoid robotics context would fail to effectively convey complex system interactions and structures, leading to ambiguity.

## References

- Feature Spec: specs/001-ros2-humanoid-control/spec.md
- Implementation Plan: specs/001-ros2-humanoid-control/plan.md
- Related ADRs: null
- Evaluator Evidence: null
