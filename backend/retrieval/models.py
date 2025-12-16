from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field

class RetrievedChunk(BaseModel):
    text: str = Field(..., description="The raw text content of the retrieved chunk.")
    metadata: Dict[str, Any] = Field(..., description="A JSON object containing all associated metadata from the original ingested VectorRecord payload.")
    score: float = Field(..., description="The similarity score between the query embedding and the chunk's embedding. Higher is more similar.")
