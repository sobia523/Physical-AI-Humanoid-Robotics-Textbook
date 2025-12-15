# Research for Fix Broken Routes & Enable Working Search in Docusaurus v3

## Key Decisions:

### 1. Routing Fix Strategy:
- **Decision**: Audit and fix all internal links (`Start Reading` button, module cards, navbar links) using `@docusaurus/Link`. Point `Start Reading` to a guaranteed existing page (`/docs/intro`). Remove or update links pointing to non-existent modules.
- **Rationale**: Directly addresses "Page Not Found" errors by ensuring all internal navigation targets valid Docusaurus pages. Using `@docusaurus/Link` is the official and recommended way for internal navigation in Docusaurus.

### 2. Search Functionality Strategy:
- **Decision**: Utilize an official Docusaurus search solution, preferably `@easyops-cn/docusaurus-search-local` or Algolia (if credentials exist). Ensure the search plugin is properly registered in `docusaurus.config.js` and docs are included for indexing. Remove any conflicting custom/fake search components and use only the official Docusaurus search component.
- **Rationale**: Adheres to constraints of using official Docusaurus search, ensures robustness and compatibility with Docusaurus v3, and avoids custom JavaScript logic. This will make the global search bar functional for modules and chapters.

### 3. Validation Strategy:
- **Decision**: Implement a comprehensive validation phase including:
    -   Clicking `Start Reading` and all navbar links to ensure no 404s.
    -   Performing search tests for module titles, chapter titles, and page headings.
    -   Confirming search results appear, no console errors, no UI duplication, and no "Page Not Found" screens.
- **Rationale**: Ensures the fixes for routing and search are effective, stable, and do not introduce new issues or break existing UI/functionality.

### 4. Chosen Search Solution:
- **Decision**: For this implementation, `@easyops-cn/docusaurus-search-local` will be used as the official Docusaurus search solution.
- **Rationale**: This choice avoids the need for external credentials (Algolia) and provides an immediate, functional search without violating any constraints. It is a suitable local search solution for a textbook website.