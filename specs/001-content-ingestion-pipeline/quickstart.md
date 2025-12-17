# Quickstart: RAG Ingestion Pipeline

This guide provides the steps to set up the Python environment and run the content ingestion pipeline.

## 1. Prerequisites

- Python 3.10 or higher installed.
- `uv` installed. If not, run: `pip install uv`.

## 2. Environment Setup

1.  **Navigate to the Backend Directory**:
    ```bash
    cd backend
    ```

2.  **Create a Virtual Environment**:
    Use `uv` to create and activate a virtual environment.
    ```bash
    uv venv
    source .venv/bin/activate  # On Windows, use `.venv\Scripts\activate`
    ```

3.  **Install Dependencies**:
    Install all required packages from the `requirements.txt` file.
    ```bash
    uv pip install -r requirements.txt
    ```

## 3. Configuration

1.  **Create an Environment File**:
    Copy the example environment file to create your own local configuration.
    ```bash
    cp .env.example .env
    ```

2.  **Add Your API Keys**:
    Open the `.env` file in a text editor and add your API keys for Cohere and Qdrant.
    ```ini
    # .env
    COHERE_API_KEY="your-cohere-api-key"
    QDRANT_URL="http://localhost:6333"
    QDRANT_API_KEY="your-qdrant-api-key-if-any"
    ```

## 4. Running the Pipeline

- **Execute the Main Script**:
  Run the `run_pipeline.py` script from within the `backend` directory to start the full ingestion process.
  ```bash
  python scripts/run_pipeline.py
  ```

  The script will:
  1. Discover URLs from the sitemap.
  2. Extract and chunk content from each page.
  3. Generate embeddings using Cohere.
  4. Store the vectors and metadata in your Qdrant instance.

- **Expected Output**:
  The script will log its progress through each phase. Upon completion, the vector database will be populated with the content from the Docusaurus website.
