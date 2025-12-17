```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
import uvicorn
from contextlib import asynccontextmanager
from dotenv import load_dotenv

# Explicitly load .env from parent directory
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../.env"))

from .rag_pipeline import get_answer, ingest_docs

app = FastAPI()

# Configure CORS
origins = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://localhost:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Citation(BaseModel):
    source_url: str
    section: str
    raw_text_snippet: str

class AgentResponse(BaseModel):
    answer: str
    citations: List[Citation]
    message: Optional[str] = None

class FrontendRequest(BaseModel):
    query: str
    source_url_constraint: Optional[str] = None
    section_constraint: Optional[str] = None
    selected_text_constraint: Optional[str] = None

@app.post("/agent/ask", response_model=AgentResponse)
async def ask_agent(request: FrontendRequest):
    try:
        response = await get_answer(
            query=request.query,
            selected_text=request.selected_text_constraint
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
def read_root():
    return {"message": "RAG Agent API is running"}
