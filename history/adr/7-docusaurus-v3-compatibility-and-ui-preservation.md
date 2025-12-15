# ADR-7: Docusaurus v3 Compatibility and UI Preservation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-fix-docusaurus-routes-search
- **Context:** The feature aims to fix routing and enable search functionality within an existing Docusaurus v3 website. A core constraint is to achieve these goals without altering the current UI, layout, styling, or existing components.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

All implementation efforts will strictly adhere to Docusaurus v3 compatibility and ensure that no changes are made to the existing UI, layout, styling, or components (including the homepage UI, cards, hero section, navbar design, and footer). The focus will be solely on functional fixes for routing and search integration.

## Consequences

### Positive

-   Maintains a consistent user experience and brand identity, as no visual changes are introduced.
-   Reduces development time and risk by narrowing the scope and avoiding design/styling discussions.
-   Ensures compatibility with Docusaurus v3, leveraging its features and stability.

### Negative

-   May limit potential for minor UI improvements that could enhance usability alongside functional fixes.
-   Requires careful review during implementation to prevent accidental UI changes.

## Alternatives Considered

-   **Allow minor UI/styling adjustments**: Rejected as it would expand the scope of the feature, introduce potential for design creep, and violate the explicit constraint of "Do not change any existing UI, layout, styling, or components."
-   **Implement without explicit Docusaurus v3 compatibility**: Rejected as it would risk introducing regressions or requiring significant rework for future Docusaurus upgrades.

## References

- Feature Spec: specs/001-fix-docusaurus-routes-search/spec.md
- Implementation Plan: specs/001-fix-docusaurus-routes-search/plan.md
- Related ADRs: null
- Evaluator Evidence: null
