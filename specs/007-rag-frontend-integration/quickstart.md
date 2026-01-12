# Quickstart: RAG Frontend Integration

This guide provides the steps to set up the Docusaurus development environment and integrate the RAG chat UI.

## 1. Prerequisites

-   Node.js (LTS version) installed.
-   `npm` or `yarn` installed.
-   A running FastAPI agent backend (from Spec-006) accessible via HTTP.
-   An OpenAI API Key configured in the backend.

## 2. Docusaurus Development Setup

1.  **Navigate to the `website` Directory**:
    ```bash
    cd website
    ```

2.  **Install Frontend Dependencies**:
    ```bash
    npm install
    # OR
    yarn install
    ```

3.  **Start Docusaurus Development Server**:
    ```bash
    npm start
    # OR
    yarn start
    ```
    This will open the Docusaurus site in your browser, typically at `http://localhost:3000`.

## 3. Configuration

1.  **Backend API URL**:
    The frontend needs to know the URL of the FastAPI agent backend. This should be configured via environment variables accessible to the Docusaurus build process.

    In `website/.env` (or similar build-time config for Docusaurus):
    ```ini
    # website/.env
    REACT_APP_RAG_AGENT_API_URL="http://localhost:8000/agent/ask"
    # Replace with your deployed backend URL in production
    ```
    (Note: `REACT_APP_` prefix is typical for Create React App/Docusaurus to expose env vars to frontend code.)

## 4. Integrating the Chat UI Component

1.  **Create Custom React Component**:
    Develop a React component (e.g., `src/components/ChatWidget.js`) that handles:
    -   Displaying the floating button.
    -   Expanding/collapsing the chat panel.
    -   Input field for queries.
    -   Displaying agent responses and citations.
    -   Capturing selected text (`window.getSelection()`) and sending it as `selected_text_constraint`.
    -   Making HTTP POST requests to the `REACT_APP_RAG_AGENT_API_URL`.
    -   Handling loading states, errors, and empty responses.

2.  **Swizzle Docusaurus Theme Layout**:
    Use the Docusaurus `swizzle` command to customize the theme's `Layout` component.
    ```bash
    npm run swizzle @docusaurus/theme-classic Layout -- --eject
    # OR
    yarn swizzle @docusaurus/theme-classic Layout -- --eject
    ```
    This creates `src/theme/Layout/index.js` (or similar).

3.  **Inject `ChatWidget` into Layout**:
    Import and render your `ChatWidget` component within `src/theme/Layout/index.js` (or `index.tsx` for TypeScript) to ensure it appears on all pages.

## 5. Testing the Integration

-   **Manual Testing**: Browse the Docusaurus site, submit queries through the chat UI, and verify responses. Test selected text functionality.
-   **Automated Tests**: Implement Playwright or Cypress tests to simulate user interactions and validate UI behavior, API calls, and response rendering.
