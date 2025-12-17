from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field

class Citation(BaseModel):
    source_url: str = Field(..., description="The full URL of the original document.")
    section: str = Field(..., description="The specific section or heading within the document.")
    raw_text_snippet: str = Field(..., description="A short snippet of the raw text from the cited chunk that was most relevant to the answer.")

class AgentRequest(BaseModel):
    query: str = Field(..., description="The user's natural language question.")
    source_url_constraint: Optional[str] = Field(None, description="Optional URL to limit the retrieval to specific source.")
    section_constraint: Optional[str] = Field(None, description="Optional section title to limit the retrieval to a specific section.")

class AgentResponse(BaseModel):
    answer: str = Field(..., description="The agent's synthesized answer to the user's query.")
    citations: List[Citation] = Field(..., description="A list of citations or source references used to form the answer.")
    message: Optional[str] = Field(None, description="An informational message (e.g., if the agent cannot answer the question).")
