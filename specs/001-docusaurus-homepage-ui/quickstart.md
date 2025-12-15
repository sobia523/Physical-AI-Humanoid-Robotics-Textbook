# Quickstart Guide: Docusaurus Homepage UI

This guide provides instructions to quickly set up and view the Docusaurus Homepage UI locally.

## Prerequisites

*   Node.js (LTS version recommended)
*   Yarn or npm package manager

## Steps to Run the Homepage UI

1.  **Navigate to the `website` directory**:
    Open your terminal or command prompt and change the current directory to the `website` folder within the project root:

    ```bash
    cd website
    ```

2.  **Install Dependencies**:
    Install all required Node.js packages.

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

    The homepage UI, including all new components (Navbar, Hero Section, Modules Grid, Search, Footer), will be accessible in your web browser. Any changes you make to the source files (`src/components`, `src/pages/index.tsx`, `src/css/custom.css`, etc.) will trigger a live reload in the browser.

## Viewing Production Build

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