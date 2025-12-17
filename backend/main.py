import uvicorn
from fastapi import FastAPI, APIRouter
from typing import List, Dict, Any, Optional
import os # Import os for environment variables
import sys

# Get the path to the 'backend' directory (where this script resides or a parent)
current_script_dir = os.path.dirname(os.path.abspath(__file__))
backend_root_dir = os.path.abspath(current_script_dir) # main.py is directly in backend
if backend_root_dir not in sys.path:
    sys.path.insert(0, backend_root_dir)

# --- App Initialization ---
app = FastAPI(
    title="RAG Agent API",
    description="API for a Retrieval-Augmented Generation (RAG) agent that answers questions about book content.",
    version="1.0.0"
)

# --- Agent Router ---
agent_router = APIRouter(
    prefix="/agent",
    tags=["Agent"]
)

# Import AgentRequest and AgentResponse models
from agent.models import AgentRequest, AgentResponse
from agent.agent_orchestrator import AgentOrchestrator # Import the orchestrator

# Initialize AgentOrchestrator globally to avoid re-initialization per request
try:
    rag_agent_orchestrator = AgentOrchestrator()
except Exception as e:
    # Log error and potentially raise, but allow app to start for other endpoints
    print(f"ERROR: Failed to initialize RAG Agent Orchestrator: {e}")
    rag_agent_orchestrator = None # Set to None if initialization fails


@agent_router.post("/ask", response_model=AgentResponse)
async def ask_agent_endpoint(request: AgentRequest):
    """
    Submits a natural language query to the RAG agent and receives an answer
    augmented with retrieved book content and citations.
    """
    if rag_agent_orchestrator is None:
        return AgentResponse(
            answer="Agent service is unavailable due to initialization error.",
            citations=[],
            message="Agent not initialized."
        )
    
    response = rag_agent_orchestrator.ask_agent(request)
    return response

app.include_router(agent_router)


# --- Basic Endpoints ---
@app.get("/")
async def root():
    return {"message": "RAG Agent API is running. Visit /docs for OpenAPI documentation."}

@app.get("/health")
async def health_check():
    """
    A simple health check endpoint.
    """
    return {"status": "ok"}

# --- Main entry point for development ---
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)