# Research for Docusaurus Homepage UI

This document outlines the research tasks to be conducted during the implementation of the Docusaurus Homepage UI.

## Research Objectives:

1.  **UI Layout Study**:
    *   **Goal**: Understand modern, premium documentation site layouts.
    *   **Sources**: OpenAI, NVIDIA, Tailwind, React Docs UI.
    *   **Output**: Key layout patterns, component structures, and overall visual hierarchies.

2.  **Color Consistency & Themability**:
    *   **Goal**: Ensure seamless light/dark mode transitions and brand consistency.
    *   **Sources**: Analysis of target websites, Docusaurus theme customization guides.
    *   **Output**: Best practices for color palette management, light/dark mode implementation.

3.  **Search Plugin Evaluation**:
    *   **Goal**: Select the most suitable search solution for Docusaurus.
    *   **Options**: Algolia vs. cmfcmf local search.
    *   **Output**: Decision with rationale, pros and cons of each option.

4.  **Iconography Selection**:
    *   **Goal**: Choose appropriate icons from `lucide-react` that align with the robotics theme.
    *   **Sources**: `lucide-react` library, examples of robotics iconography.
    *   **Output**: List of selected icons for various UI elements.

5.  **Styling Benchmarking (Spacing, Typography, Cards)**:
    *   **Goal**: Define a premium and consistent visual system.
    *   **Sources**: Premium documentation sites, Docusaurus styling capabilities.
    *   **Output**: Specific guidelines for spacing scales, typography choices (fonts, sizes, weights), and card design (shadows, border-radius, hover effects).

## Key Decisions (To be filled during research):

### Search System:
- **Decision**: Algolia DocSearch
- **Rationale**: Algolia DocSearch offers robust, fast, and customizable search capabilities specifically designed for documentation websites. It provides instant results, typo tolerance, and easy integration with Docusaurus, aligning with the "premium" and "instant results" requirements in the `spec.md`.
- **Alternatives Considered**: cmfcmf local search was considered for its simplicity and local data processing, but Algolia's superior performance for large datasets, advanced features (e.g., analytics, relevance tuning), and proven track record with Docusaurus made it the preferred choice for a production-quality experience.

### Color Palette:
- **Decision**: Implemented in `website/src/css/custom.css`.
- **Rationale**: Adheres to specified color scheme for light and dark modes, ensuring consistency across the UI.
- `light: --ifm-background-color: #ffffff; --ifm-card-background-color: #f8f8f8;`
- `dark: --ifm-background-color: #0c0c0c; --ifm-card-background-color: #151515;`
- `Accent: --ifm-color-primary: #1e80ff;`

### Icon Set:
- **Decision**: `lucide-react`
- **Rationale**: Provides a comprehensive and highly customizable set of SVG icons, aligning with modern UI design principles and offering a wide range of icons suitable for robotics themes.
- **Current Status**: Placeholder icon names (e.g., 'Brain', 'Feather', 'Zap', 'Speech') are used in `ModulesGrid.tsx`. Final selection and integration of actual `lucide-react` components will be refined during implementation of individual module cards.

### UI Layout:
- **Decision**: Adopt a modern, clean layout with generous whitespace, centered content containers (max-width 1180px), and clear visual hierarchy. Inspired by documentation sites from OpenAI, NVIDIA.
- **Rationale**: Enhances readability and provides a premium, spacious feel.

### Styling Benchmarking:
- **Decision**: Utilize a modular spacing scale (e.g., `4px`, `8px`, `16px`, `24px`, `32px`, `48px`) for consistent element spacing. Employ clean, sans-serif typography with strong headings and readable body text. Card designs feature 18px rounded corners, subtle shadows, and hover lift effects, as per `spec.md`.
- **Rationale**: Ensures visual consistency and aligns with premium documentation site aesthetics.

## Dependencies & Integrations:

- Docusaurus theming system
- React component architecture
- Potential search plugin APIs