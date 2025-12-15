# Data Model: Docusaurus Homepage UI

This document defines the key entities and their relationships relevant to the Docusaurus Homepage UI.

## Entities:

### 1. Module

*   **Description**: Represents a distinct section of the book, displayed as a card on the homepage.
*   **Attributes**:
    *   `id`: Unique identifier (string)
    *   `title`: Main title of the module (string)
    *   `subtitle`: A brief description of the module (string)
    *   `icon`: Name of the `lucide-react` icon associated with the module (string)
    *   `tags`: An array of strings representing keywords or categories for the module (array of string)
    *   `link`: URL to the module's main page (string)
*   **Relationships**: Can contain multiple Chapters.

### 2. Chapter

*   **Description**: A subsection within a Module.
*   **Attributes**:
    *   `id`: Unique identifier (string)
    *   `title`: Title of the chapter (string)
    *   `content_summary`: A brief summary of the chapter's content (string, for search indexing)
    *   `link`: URL to the chapter's page (string)
    *   `module_id`: Foreign key linking to its parent Module (string)
*   **Relationships**: Belongs to one Module.

### 3. Search Result

*   **Description**: An item returned by the search functionality, which can link to either a Module or a Chapter.
*   **Attributes**:
    *   `id`: Unique identifier (string)
    *   `type`: Type of the result (e.g., "module", "chapter") (string)
    *   `title`: Title of the found item (string)
    *   `description_snippet`: A short context snippet from the content where the search term was found (string)
    *   `link`: URL to the corresponding Module or Chapter page (string)
*   **Relationships**: Can reference either a Module or a Chapter.

## Relationships:

*   One-to-Many: `Module` to `Chapter` (One Module can have many Chapters).
*   Search results can point to either `Module` or `Chapter` entities.

## Data Flow for UI:

1.  **Module Data**: Fetched from a static source (e.g., `docusaurus.config.ts` or a dedicated data file) to populate the `ModulesGrid`.
2.  **Search Data**: Indexed from all Modules and Chapters (titles, content) by the chosen search plugin (Algolia or local search).
3.  **UI Components**: Consume these data structures to render the Navbar, Hero Section, Modules Grid, and Search Results.
