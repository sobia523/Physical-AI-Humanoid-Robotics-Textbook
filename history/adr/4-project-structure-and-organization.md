# ADR-4: Project Structure and Organization

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-11
- **Feature:** 001-ros2-humanoid-control
- **Context:** The module's content includes both Markdown files for chapters and executable code examples. An effective project structure is necessary to ensure easy navigation, reproducibility, and maintainability for both authors and learners.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Content will be organized as a single project within a dedicated module directory (`module1-ros2-humanoid-control/`) at the repository root. This directory will co-locate Markdown chapter files (`content/`), Python ROS 2 code (`src/`), and any associated test files (`tests/`).

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

Simplifies navigation for learners as content and corresponding code are found in close proximity. Enhances reproducibility by bundling all necessary assets for a specific module. Streamlines maintenance and updates as all module-specific files are self-contained.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

A large number of modules might make the repository root slightly cluttered, though this is mitigated by dedicated module directories. Potential for confusion if similar file names exist across different modules (mitigated by clear module naming conventions).

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Separated Documentation and Code Repositories**:
    *   **Why rejected**: While it can enforce strict separation, it significantly complicates the linking of content to code examples, hinders reproducibility for learners, and adds overhead for managing two repositories.
*   **Monorepo with Dispersed Code/Content**:
    *   **Why rejected**: Placing all code in a single `src/` directory at the root and all content in a single `docs/` directory would make it harder to quickly find the code relevant to a specific chapter or module, especially for new users.
*   **Deeply Nested Module Structure**:
    *   **Why rejected**: Overly complex directory hierarchies could make the project harder to navigate and understand for learners, especially when dealing with ROS 2 package structures which already have their own nesting.

## References

- Feature Spec: specs/001-ros2-humanoid-control/spec.md
- Implementation Plan: specs/001-ros2-humanoid-control/plan.md
- Related ADRs: null
- Evaluator Evidence: null
