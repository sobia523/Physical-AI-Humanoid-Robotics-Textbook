# RAG Ingestion Pipeline - Backend

This directory contains the backend services for the RAG (Retrieval Augmented Generation) content ingestion pipeline. This pipeline is responsible for discovering content from a Docusaurus website, processing it, generating embeddings, and storing them in a Qdrant vector database.

## Quickstart

This guide provides the steps to set up the Python environment and run the content ingestion pipeline.

### 1. Prerequisites

-   Python 3.10 or higher installed.
-   `uv` installed. If not, run: `pip install uv`.
-   A running instance of Qdrant. You can run it via Docker: `docker run -p 6333:6333 qdrant/qdrant`

### 2. Environment Setup

1.  **Navigate to the Backend Directory**:
    ```bash
    cd backend
    ```

2.  **Create a Virtual Environment**:
    Use `uv` to create and activate a virtual environment.
    ```bash
    uv venv
    # On Linux/macOS
    source .venv/bin/activate
    # On Windows
    .venv\Scripts/activate
    ```

3.  **Install Dependencies**:
    Install all required packages from the `requirements.txt` file.
    ```bash
    uv pip install -r requirements.txt
    ```

### 3. Configuration

1.  **Create an Environment File**:
    Copy the example environment file to create your own local configuration.
    ```bash
    cp .env.example .env
    ```

2.  **Add Your API Keys**:
    Open the `.env` file in a text editor and add your API keys for Cohere and Qdrant, along with the sitemap URL and any filtering options. **Also, add your OpenAI API Key.**

    ```ini
    # backend/.env
    COHERE_API_KEY="your-cohere-api-key"
    QDRANT_URL="http://localhost:6333"
    QDRANT_API_KEY="your-qdrant-api-key-if-any" # Leave empty if not using API key for local instance
    OPENAI_API_KEY="your-openai-api-key" # NEW: Required for OpenAI Agents SDK
    SITEMAP_URL="https://physical-ai-humanoid-robotics-textb-beta-two.vercel.app/sitemap.xml"
    BASE_URL_TO_FILTER="https://your-docusaurus-site.example.com" # Placeholder, update if real sitemap has this
    DOCS_PATH_SEGMENT="/docs/"
    QDRANT_COLLECTION_NAME="rag_content_chunks"
    MAX_CHUNK_TEXT_LENGTH="500" # Max tokens for Cohere model input
    ```

### 4. Running the Pipeline

-   **Execute the Main Script**:
    Run the `run_pipeline.py` script from within the `backend` directory to start the full ingestion process.
    ```bash
    .venv/Scripts/python.exe scripts/run_pipeline.py
    ```
    (On Linux/macOS, use `python scripts/run_pipeline.py` after activating the venv, or `~/.venv/bin/python scripts/run_pipeline.py`)

    The script will:
    1.  Discover URLs from the sitemap.
    2.  Extract and chunk content from each page.
    3.  Generate embeddings using Cohere.
    4.  Store the vectors and metadata in your Qdrant instance.

-   **Expected Output**:
    The script will log its progress through each phase to the console and to `backend/logs/pipeline.log`. Upon completion, the vector database will be populated with the content from the Docusaurus website.

### 5. Running Data Validation

-   **Execute the Validation Script**:
    Run the `validate_data.py` script from within the `backend` directory to check the integrity of the stored data.
    ```bash
    .venv/Scripts/python.exe scripts/validate_data.py
    ```

    The script will:
    1.  Connect to Qdrant.
    2.  Fetch a random sample of records.
    3.  Verify that critical metadata fields are present and non-empty.
    4.  Log its findings to the console and `backend/logs/validation.log`.

### 6. Running Retrieval Tests

-   **Execute the Retrieval Test Script**:
    Run the `test_retrieval.py` script from within the `backend` directory to perform semantic searches and validate retrieval behavior.
    ```bash
    .venv/Scripts/python.exe scripts/test_retrieval.py
    ```

    The script will:
    1.  Perform various semantic queries, both unfiltered and filtered (by source URL and section).
    2.  Validate that retrieved chunks contain complete and accurate metadata.
    3.  Measure and report retrieval latency.
    4.  Log its findings to the console and `backend/logs/retrieval_test.log`.

### 7. Running the FastAPI Agent API

-   **Start the Uvicorn Server**:
    From the `backend` directory, run the FastAPI application using Uvicorn.
    ```bash
    .venv/Scripts/uvicorn.exe main:app --reload
    ```
    (On Linux/macOS, use `uvicorn main:app --reload` after activating the venv, or `~/.venv/bin/uvicorn main:app --reload`)

-   **Access API Documentation**:
    Once the server is running, you can access the interactive API documentation (Swagger UI) at:
    `http://127.0.0.1:8000/docs`

### 8. Running Agent API Tests

-   **Execute the Agent API Test Script**:
    Run the `test_agent_api.py` script from within the `backend` directory to test the FastAPI agent endpoint.
    ```bash
    .venv/Scripts/python.exe scripts/test_agent_api.py
    ```

    The script will:
    1.  Send various queries to the `/agent/ask` endpoint, including basic questions and constrained queries.
    2.  Validate the agent's responses for accuracy, citation presence, constraint adherence, and latency.
    3.  Log its findings to the console and `backend/logs/agent_api_test.log`.