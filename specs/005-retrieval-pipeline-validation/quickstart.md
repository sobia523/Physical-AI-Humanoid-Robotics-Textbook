# Quickstart: RAG Retrieval Pipeline Validation

This guide provides the steps to set up the Python environment and run the retrieval validation scripts.

## 1. Prerequisites

-   Python 3.10 or higher installed.
-   `uv` installed.
-   A running Qdrant instance with the `rag_content_chunks` collection populated by the ingestion pipeline (from Spec-1).
-   Your Cohere API Key must be configured in `backend/.env`.

## 2. Environment Setup

1.  **Navigate to the Backend Directory**:
    ```bash
    cd backend
    ```

2.  **Activate Virtual Environment**:
    Ensure your Python virtual environment is activated. If not, create and activate it:
    ```bash
    uv venv
    source .venv/bin/activate  # On Windows, use `.venv\Scripts\activate`
    ```

3.  **Install Dependencies**:
    Ensure all required packages are installed.
    ```bash
    uv pip install -r requirements.txt
    ```

## 3. Configuration

1.  **Environment File**:
    Ensure your `backend/.env` file is properly configured with your `COHERE_API_KEY` and `QDRANT_URL`. The ingestion pipeline should have already created an `.env` file if it was run.

    ```ini
    # backend/.env (example)
    COHERE_API_KEY="your-cohere-api-key"
    QDRANT_URL="http://localhost:6333"
    QDRANT_API_KEY="your-qdrant-api-key-if-any"
    ```

## 4. Running Retrieval Validation

-   **Execute the Validation Script**:
    The main validation script (`test_retrieval.py`) will perform semantic searches, optionally with filters, against the Qdrant database and report its findings.

    ```bash
    .venv/Scripts/python.exe scripts/test_retrieval.py
    ```
    (On Linux/macOS, use `python scripts/test_retrieval.py` after activating the venv, or `~/.venv/bin/python scripts/test_retrieval.py`)

-   **Expected Output**:
    The script will log its progress and the results of the semantic queries, including retrieved chunks, metadata, and scores. It will indicate whether the retrieved content matches expectations (e.g., relevance, correct filtering).
