# ADR-6: Docusaurus Search Solution

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-fix-docusaurus-routes-search
- **Context:** The existing global search bar on the Docusaurus website is not functional or does not return relevant results. This ADR documents the decision to implement a functional search solution.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Utilize an official Docusaurus search solution to enable global search functionality. The preferred options are `@easyops-cn/docusaurus-search-local` or Algolia DocSearch (if credentials exist). The chosen solution will be properly registered in `docusaurus.config.js`, ensuring documentation content is included for indexing. Any conflicting custom or fake search components will be removed, and only the official Docusaurus search component will be used.

## Consequences

### Positive

-   Enables users to quickly find relevant content within the book, significantly improving content discoverability.
-   Adheres to the constraint of using official Docusaurus solutions, ensuring compatibility and maintainability.
-   Avoids the complexity and potential issues of custom JavaScript search implementations.

### Negative

-   Integration with Algolia DocSearch may require setting up an Algolia account and managing credentials.
-   The performance and relevance of search results will depend on the capabilities and configuration of the chosen plugin.

## Alternatives Considered

-   **Custom JavaScript search implementation**: Rejected because it would violate the constraint of "No custom JavaScript search logic," introduce unnecessary complexity, and potentially lead to maintenance burden.
-   **Ignoring search functionality**: Rejected because a functional search bar is a critical requirement for a technical textbook website to enhance user experience and content discoverability.

## References

- Feature Spec: specs/001-fix-docusaurus-routes-search/spec.md
- Implementation Plan: specs/001-fix-docusaurus-routes-search/plan.md
- Related ADRs: null
- Evaluator Evidence: null
