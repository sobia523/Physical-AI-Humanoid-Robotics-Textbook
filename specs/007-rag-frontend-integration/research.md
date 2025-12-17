# Research & Decisions: RAG Frontend Integration

This document records key technical decisions made during the planning phase for the RAG Frontend and Backend Integration.

## R-01: Specific Docusaurus integration points for custom UI components

-   **Decision**: Utilize Docusaurus theme swizzling and custom React components. A dedicated React component for the chat widget will be created and injected into the theme layout (e.g., `Layout.js`).
-   **Rationale**: Theme swizzling is the recommended Docusaurus approach for customizing UI components. It allows overriding or wrapping existing theme components (`@docusaurus/theme-classic`) without needing to eject the entire theme, ensuring maintainability and compatibility with future Docusaurus updates. This provides the necessary control over the chat UI's placement and behavior while adhering to Docusaurus best practices.
-   **Alternatives considered**:
    -   Docusaurus plugins: While powerful, creating a full plugin for a UI component is often overkill and more complex than necessary for simple component injection.
    -   Direct DOM manipulation (e.g., jQuery): Highly discouraged in React-based frameworks like Docusaurus as it can lead to conflicts and is not idiomatic.

## R-02: Cross-browser compatibility for selected text capture

-   **Decision**: Implement text selection capture using standard JavaScript DOM APIs: `window.getSelection()` along with `getRangeAt(0)` to retrieve the `Range` object. The selected text (`range.toString()`) and contextual information (page URL via `window.location.href`, and closest heading/section via DOM traversal from the `range.startContainer` or `range.commonAncestorContainer`) will be extracted.
-   **Rationale**: `window.getSelection()` is a widely supported and robust browser API for handling user text selections. This approach ensures broad cross-browser compatibility. Extracting contextual information from the DOM allows precise scoping for "selected text only" queries.
-   **Alternatives considered**:
    -   Third-party text selection libraries: Adds unnecessary dependency for a feature that can be implemented with native APIs, potentially introducing overhead or compatibility issues.

## R-03: Detailed design of the floating chat widget (UI/UX, expandable/collapsible behavior)

-   **Decision**:
    1.  **Placement**: A small, circular, floating button will be positioned at the bottom-right corner of the browser viewport.
    2.  **Toggle Behavior**: Clicking the button will toggle the visibility of a larger chat panel (which will appear as an overlay or slide in from the side).
    3.  **Draggability**: The floating button (and potentially the expanded chat panel header) will be draggable by the user, allowing them to reposition it on the screen.
-   **Rationale**: This UI/UX pattern is familiar to users from many modern websites and applications. It's non-intrusive, can be accessed quickly, and draggable functionality enhances user customization and accessibility, preventing the widget from obstructing critical content.
-   **Alternatives considered**:
    -   Sidebar integration: Requires modifying the Docusaurus theme layout significantly and might reduce available content area.
    -   Inline component: Interrupts the reading flow and would need to be present on every page, potentially leading to visual clutter.

## R-04: Implementation approach for making citations clickable and scrolling to sections within Docusaurus

-   **Decision**:
    1.  **Rendering**: Citations returned in the agent's response will be rendered within the chat UI as clickable anchor (`<a>`) tags.
    2.  **Navigation**: Clicking a citation link will trigger client-side navigation to the `source_url` within the Docusaurus site.
    3.  **Scrolling**: Upon navigation, JavaScript will attempt to scroll the user to the `section` by finding an element with an appropriate ID (derived from the section heading).
-   **Rationale**: This design provides intuitive traceability, allowing users to verify the agent's answers directly within the authoritative book content. Docusaurus pages (being single-page applications) handle client-side routing well, and standard HTML anchor links (`#section-id`) combined with JavaScript `scrollIntoView()` are effective for precise navigation.
-   **Alternatives considered**:
    -   Displaying full chunk text in chat: Overloads the chat UI and defeats the purpose of linking to the source.
    -   Opening new tabs for citations: Disrupts user flow and creates tab clutter.
