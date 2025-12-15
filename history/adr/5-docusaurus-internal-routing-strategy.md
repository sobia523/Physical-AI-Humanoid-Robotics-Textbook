# ADR-5: Docusaurus Internal Routing Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-fix-docusaurus-routes-search
- **Context:** The existing Docusaurus website has broken internal links, leading to "Page Not Found" errors. This ADR addresses the strategy for fixing these broken links to ensure a smooth user navigation experience.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement internal routing fixes using `@docusaurus/Link` for all internal navigation, including the "Start Reading" button, module cards, and navbar links. The "Start Reading" button will specifically point to `/docs/intro` to ensure it always lands on an existing page. All existing internal links will be audited and updated to point to valid Docusaurus pages.

## Consequences

### Positive

-   Eliminates "Page Not Found" errors, greatly improving user experience.
-   Ensures reliable and consistent navigation throughout the Docusaurus site.
-   Leverages official Docusaurus mechanisms, promoting maintainability and compatibility.

### Negative

-   Requires a thorough audit of all existing internal links, which can be time-consuming.
-   Care must be taken to ensure all new links correctly point to valid content.

## Alternatives Considered

-   **Custom JavaScript routing**: Rejected because it would introduce unnecessary complexity, potentially conflict with Docusaurus's built-in routing, and violate the constraint of "No custom JavaScript search logic."
-   **Ignoring the issue**: Rejected due to severe negative impact on user experience and the core usability of the website.

## References

- Feature Spec: specs/001-fix-docusaurus-routes-search/spec.md
- Implementation Plan: specs/001-fix-docusaurus-routes-search/plan.md
- Related ADRs: null
- Evaluator Evidence: null
