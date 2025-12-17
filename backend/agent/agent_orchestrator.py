import os
import sys
from typing import List, Dict, Any, Optional
import openai
# from openai.lib.tool_streaming import ToolStream # Removed as it's not directly exposed or needed for tool definition
import json # For parsing tool outputs

# Get the path to the project root, assuming it's the directory containing 'backend'
current_script_dir = os.path.dirname(os.path.abspath(__file__))
backend_root_dir = os.path.abspath(os.path.join(current_script_dir, '..'))
if backend_root_dir not in sys.path:
    sys.path.insert(0, backend_root_dir)

from agent.tools import RetrieverTool
from retrieval.models import RetrievedChunk
from config.logging_config import setup_logging
from agent.models import AgentRequest, AgentResponse, Citation # Corrected import

logger = setup_logging()

# Define the Agent's system instructions
SYSTEM_INSTRUCTIONS = """
You are a highly intelligent and helpful RAG (Retrieval Augmented Generation) agent.
Your primary goal is to answer user questions about the provided book content accurately and thoroughly.

Key Rules:
1. ONLY use the information provided in the CONTEXT below to answer the user's question.
2. If the CONTEXT does not contain enough information to answer the question, state that you cannot answer based on the provided knowledge. DO NOT use your prior knowledge.
3. Always include citations for every piece of information you extract from the CONTEXT. Format citations as follows:
   [Source: {source_url}, Section: {section}]
   For example: "ROS 2 is a flexible framework for robot development [Source: example.com/ros2, Section: Introduction]."
4. Prefer to cite the most specific section possible.
5. If constraints (like source_url or section) were provided in the original query, your answer MUST strictly adhere to them. If the retrieved content does not match the constraints, indicate that you cannot answer within those constraints.
6. Be concise and direct.
"""

class AgentOrchestrator:
    def __init__(self, openai_api_key: Optional[str] = None):
        self.openai_api_key = openai_api_key if openai_api_key else os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            logger.error("OPENAI_API_KEY not found in environment variables.")
            raise ValueError("OPENAI_API_KEY not found in environment variables.")
        
        self.client = openai.OpenAI(api_key=self.openai_api_key)
        self.retriever_tool = RetrieverTool() # Initialize the custom tool
        self.retriever_tool_definition = {
            "type": "function",
            "function": {
                "name": "retrieve_chunks",
                "description": "Retrieves relevant text chunks from the book content based on a semantic query and optional filters.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {"type": "string", "description": "The natural language query string to search for."},
                        "top_k": {"type": "integer", "description": "The number of top similar results to retrieve.", "default": 5},
                        "source_url_constraint": {"type": "string", "nullable": True, "description": "Optional URL to limit retrieval to a specific source."},
                        "section_constraint": {"type": "string", "nullable": True, "description": "Optional section title to limit retrieval to a specific section."
                    },
                    "required": ["query"]
                }
            }
        }
        self.llm_model = "gpt-4-turbo-preview" # Or "gpt-3.5-turbo", configurable

        logger.info("AgentOrchestrator initialized with OpenAI client and RetrieverTool.")

    def _format_chunks_for_llm(self, chunks: List[RetrievedChunk]) -> str:
        """
        Formats a list of RetrievedChunk objects into a string suitable for LLM context.
        Includes metadata for citation.
        """
        if not chunks:
            return "No relevant documents found."

        formatted_context = []
        for i, chunk in enumerate(chunks):
            source_url = chunk.metadata.get('source_url', 'N/A')
            section = chunk.metadata.get('section', 'N/A')
            # Assuming page_title is part of metadata
            page_title = chunk.metadata.get('page_title', 'N/A')

            formatted_context.append(
                f"[DOCUMENT {i+1} - Source: {source_url}, Section: {section}, Page: {page_title}]\n"
                f"{chunk.text}\n"
            )
        return "\n---\n".join(formatted_context)

    def _parse_citations_from_llm_response(self, llm_response_text: str) -> List[Citation]:
        """
        Parses citations from the LLM's response based on the expected format.
        (R-04: Citation parsing and formatting from the LLM's response)
        """
        citations = []
        # Regex to find citations like [Source: URL, Section: Heading]
        # This is a simplified regex and might need refinement based on actual LLM output
        import re
        citation_pattern = re.compile(r'\[Source: (.*?)(?:, Section: (.*?))?\]') # Make section optional

        matches = citation_pattern.finditer(llm_response_text) # Corrected to findall
        
        # This will need more sophisticated logic to get raw_text_snippet
        # For now, we'll just extract source and section
        for match in matches:
            source_url = match.group(1).strip()
            section = match.group(2).strip() if match.group(2) else "Unknown Section" # Handle optional section
            
            # This is a placeholder for raw_text_snippet - in a real scenario,
            # we'd need to map the cited section/URL back to the original chunk content
            # or have the LLM provide a snippet.
            raw_text_snippet = "Snippet from source." 
            citations.append(Citation(source_url=source_url, section=section, raw_text_snippet=raw_text_snippet))
        
        # Deduplicate citations if the same source is cited multiple times
        deduplicated_citations = []
        seen_citations = set()
        for c in citations:
            citation_key = (c.source_url, c.section)
            if citation_key not in seen_citations:
                deduplicated_citations.append(c)
                seen_citations.add(citation_key)

        return deduplicated_citations


    def ask_agent(self, request: AgentRequest) -> AgentResponse:
        """
        Orchestrates the RAG agent to answer a user query.
        """
        messages = [{"role": "system", "content": SYSTEM_INSTRUCTIONS}]
        
        # Step 1: Generate query embedding and retrieve chunks using the tool
        # The agent will call this tool
        tool_outputs = self.retriever_tool.run(
            query=request.query, 
            top_k=5, # Default top_k
            filters={
                k: v for k, v in {
                    "source_url": request.source_url_constraint,
                    "section": request.section_constraint
                }.items() if v is not None
            }
        )
        
        if not tool_outputs:
            logger.warning(f"No relevant chunks retrieved for query: '{request.query}'")
            return AgentResponse(
                answer="I cannot answer this question based on the provided knowledge. No relevant content was found.",
                citations=[],
                message="No relevant content retrieved."
            )

        # Step 2: Construct context for the LLM
        formatted_context = self._format_chunks_for_llm(tool_outputs)
        
        # Add constraint specific instructions to the user message
        user_message_content = f"Here is the context from the book:\n{formatted_context}\n\nUser's Question: {request.query}"
        if request.source_url_constraint:
            user_message_content += f"\n\nIMPORTANT: Only use content from the following source URL: {request.source_url_constraint}"
        if request.section_constraint:
            user_message_content += f"\n\nIMPORTANT: Only use content from the following section: {request.section_constraint}"

        messages.append({
            "role": "user",
            "content": user_message_content
        })

        # Step 3: Invoke the LLM
        try:
            chat_completion = self.client.chat.completions.create(
                model=self.llm_model,
                messages=messages,
                tools=[self.retriever_tool_definition], # Provide the tool definition to the LLM
                tool_choice="auto", # Allow LLM to decide if it wants to use the tool
                temperature=0.0 # Ensure deterministic responses
            )

            # This part will be tricky with OpenAI Agents SDK
            # For now, we assume the LLM directly gives the answer + citations
            # The actual agent orchestration from OpenAI SDK might involve more steps.
            llm_answer = chat_completion.choices[0].message.content
            
            citations = self._parse_citations_from_llm_response(llm_answer)

            return AgentResponse(
                answer=llm_answer,
                citations=citations
            )
        except Exception as e:
            logger.error(f"Error during LLM reasoning for query '{request.query}': {e}")
            return AgentResponse(
                answer="I encountered an error while trying to answer your question.",
                citations=[],
                message=f"LLM error: {str(e)}"
            )

if __name__ == '__main__':
    # This block is for testing the agent directly
    # Requires configured backend/.env (COHERE_API_KEY, QDRANT_URL, OPENAI_API_KEY),
    # running Qdrant with data, and a working OpenAI API key.
    try:
        logger.info("Starting AgentOrchestrator direct test.")
        orchestrator = AgentOrchestrator()

        test_request = AgentRequest(
            query="What is the Robotic Nervous System?",
            source_url_constraint=None,
            section_constraint=None
        )

        response = orchestrator.ask_agent(test_request)
        
        logger.info("\n--- Agent Response ---")
        logger.info(f"Answer: {response.answer}")
        if response.citations:
            logger.info("Citations:")
            for cit in response.citations:
                logger.info(f"  - Source: {cit.source_url}, Section: {cit.section}, Snippet: {cit.raw_text_snippet}")
        if response.message:
            logger.info(f"Message: {response.message}")

        # Test with constraints
        test_request_constrained = AgentRequest(
            query="What is a `RetrievedChunk`?",
            source_url_constraint="http://your-docusaurus-site.example.com/docs/module5/data-model", # Placeholder
            section_constraint="Data Model: Retrieved Chunk" # Placeholder
        )
        response_constrained = orchestrator.ask_agent(test_request_constrained)
        logger.info("\n--- Agent Response (Constrained) ---")
        logger.info(f"Answer: {response_constrained.answer}")
        if response_constrained.citations:
            logger.info("Citations:")
            for cit in response_constrained.citations:
                logger.info(f"  - Source: {cit.source_url}, Section: {cit.section}, Snippet: {cit.raw_text_snippet}")
        if response_constrained.message:
            logger.info(f"Message: {response_constrained.message}")


    except Exception as e:
        logger.error(f"Error during AgentOrchestrator direct test: {e}")