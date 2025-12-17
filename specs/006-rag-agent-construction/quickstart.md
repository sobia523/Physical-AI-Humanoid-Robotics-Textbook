# Quickstart: RAG Agent with FastAPI

This guide provides the steps to set up the Python environment, run the FastAPI application, and interact with the RAG agent endpoint.

## 1. Prerequisites

-   Python 3.10 or higher installed.
-   `uv` installed.
-   A running Qdrant instance with the `rag_content_chunks` collection populated by the ingestion pipeline (from Spec-1).
-   Your Cohere API Key must be configured in `backend/.env`.
-   Your OpenAI API Key must be configured in `backend/.env` for the OpenAI Agents SDK.

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
    Ensure all required packages are installed. You will need to add `fastapi`, `uvicorn`, and `openai` (for the Agents SDK) to `requirements.txt` if they are not already present.
    ```bash
    uv pip install -r requirements.txt
    ```

## 3. Configuration

1.  **Environment File**:
    Ensure your `backend/.env` file is properly configured with your `COHERE_API_KEY`, `QDRANT_URL`, and `OPENAI_API_KEY`.

    ```ini
    # backend/.env (example)
    COHERE_API_KEY="your-cohere-api-key"
    QDRANT_URL="http://localhost:6333"
    QDRANT_API_KEY="your-qdrant-api-key-if-any"
    OPENAI_API_KEY="your-openai-api-key"
    ```

## 4. Running the FastAPI Application

-   **Start the Uvicorn Server**:
    From the `backend` directory, run the FastAPI application using Uvicorn.
    ```bash
    uvicorn main:app --reload
    ```

-   **Access API Documentation**:
    Once the server is running, you can access the interactive API documentation (Swagger UI) at:
    `http://127.0.0.1:8000/docs`

## 5. Interacting with the Agent

-   **Using Swagger UI**:
    Navigate to `http://127.0.0.1:8000/docs` in your browser.
    Find the `/agent/ask` endpoint, click "Try it out", fill in the `query` and optional constraints, and click "Execute".

-   **Using `curl`**:
    ```bash
    curl -X POST "http://127.0.0.1:8000/agent/ask" \
         -H "Content-Type: application/json" \
         -d 
             {
               "query": "What is the Robotic Nervous System?",
               "source_url_constraint": null,
               "section_constraint": null
             }
    ```

-   **Expected Output**:
    The API will return a JSON response containing the agent's `answer` and a list of `citations` from the retrieved book content. If the agent cannot answer, the `answer` field might be empty or contain a message, and the `message` field in `AgentResponse` might be populated.
