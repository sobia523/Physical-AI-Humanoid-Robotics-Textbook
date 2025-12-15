# Quickstart Guide: Fix Broken Routes & Enable Working Search in Docusaurus v3

## Prerequisites

*   Node.js (LTS version recommended)
*   Yarn or npm package manager

## Steps to Verify the Fixes

1.  **Navigate to the `website` directory**:
    Open your terminal or command prompt and change the current directory to the `website` folder within the project root:

    ```bash
    cd website
    ```

2.  **Install Dependencies (if not already installed)**:
    Install all required Node.js packages. This is crucial for Docusaurus to function correctly and for search plugins to be available.

    ```bash
    npm install
    # OR
    yarn install
    ```

3.  **Start the Docusaurus Development Server**:
    Launch the Docusaurus development server. This will build the website and serve it locally, typically at `http://localhost:3000`.

    ```bash
    npm run start
    # OR
    yarn start
    ```

4.  **Verify Routing Fixes**:
    Once the development server is running and the website is open in your browser:
    -   Click the "Start Reading" button on the homepage and ensure it navigates to a valid page (e.g., `/docs/intro`) without a "Page Not Found" error.
    -   Click on several module cards and navbar links to confirm they navigate to their respective pages correctly and do not show any "Page Not Found" errors.

5.  **Verify Search Functionality**:
    -   Locate the global search bar (typically in the navbar).
    -   Enter various search terms related to module titles (e.g., "ROS 2"), chapter titles (e.g., "Middleware"), and page headings.
    -   Confirm that relevant search results appear in the dropdown/search overlay.
    -   Click on a few search results to ensure they navigate to the correct content.
    -   Ensure there are no console errors related to search while typing or clicking results.

## Viewing Production Build (Optional)

To see how the website will appear in a production environment, you can build the static files:

1.  **Build the Website**:

    ```bash
    npm run build
    # OR
    yarn build
    ```

    This command generates static content into the `build` directory.

2.  **Serve the Built Website (Optional)**:
    You can use a simple static server to view the `build` output. For example, using `serve`:

    ```bash
    npx serve build
    # OR
    yarn global add serve # if not already installed
    serve build
    ```

    Then open your browser to the address provided by `serve` (e.g., `http://localhost:5000`).
