from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field
import os
import sys

# Get the path to the project root, assuming it's the directory containing 'backend'
# This script is in backend/retrieval, so '..' is backend, '..'+'..' is project_root
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)


class RetrievedChunk(BaseModel):
    text: str = Field(..., description="The raw text content of the retrieved chunk.")
    metadata: Dict[str, Any] = Field(..., description="A JSON object containing all associated metadata from the original ingested VectorRecord payload.")
    score: float = Field(..., description="The similarity score between the query embedding and the chunk's embedding. Higher is more similar.")
