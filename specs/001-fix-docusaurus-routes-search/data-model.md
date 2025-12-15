# Data Model: Fix Broken Routes & Enable Working Search in Docusaurus v3

## Entities:

### 1. Module
*   **Description**: Represents a distinct section of the book.
*   **Attributes**: Implied attributes include `title`, `link`.
*   **Relationships**: Can contain multiple Chapters. Searchable. Navigable.

### 2. Chapter
*   **Description**: A subsection within a Module.
*   **Attributes**: Implied attributes include `title`, `link`.
*   **Relationships**: Belongs to one Module. Searchable. Navigable.

### 3. Page
*   **Description**: Any content page in the Docusaurus site.
*   **Attributes**: Implied attributes include `title` (from heading), `link`.
*   **Relationships**: Can contain multiple headings. Searchable. Navigable.
