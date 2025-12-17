import os
import sys
from typing import List, Dict, Any, Optional

# Get the path to the project root, assuming it's the directory containing 'backend'
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Corrected imports
from backend.retrieval import retriever # Import the module, not the class directly
from backend.retrieval.models import RetrievedChunk
from backend.config.logging_config import setup_logging

logger = setup_logging()

class RetrieverTool:
    """
    A tool that encapsulates the RAG retrieval logic to be used by an agent.
    """
    def __init__(self):
        # Access the Retriever class through the imported module
        self.retriever = retriever.Retriever() 
        logger.info("RetrieverTool initialized with Retriever instance.")

    def run(self, query: str, top_k: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[RetrievedChunk]:
        """
        Executes a retrieval query against the Qdrant vector store.

        Args:
            query: The natural language query string.
            top_k: The number of top similar results to retrieve.
            filters: Optional dictionary of payload filters (e.g., {"source_url": "some_url"}).

        Returns:
            A list of RetrievedChunk objects.
        """
        logger.info(f"RetrieverTool invoked with query: '{query[:50]}...', top_k: {top_k}, filters: {filters}")
        try:
            results = self.retriever.search(query=query, top_k=top_k, filters=filters)
            logger.info(f"RetrieverTool returned {len(results)} chunks.")
            return results
        except Exception as e:
            logger.error(f"Error during retrieval in RetrieverTool for query '{query[:50]}...': {e}")
            return []

if __name__ == '__main__':
    # This block is for testing the tool directly
    # Requires configured backend/.env and a running Qdrant with data
    try:
        logger.info("Starting RetrieverTool direct test.")
        tool = RetrieverTool()
        test_query = "What are the core principles of ROS 2?"
        results = tool.run(test_query, top_k=3)

        if results:
            logger.info(f"Tool returned {len(results)} results:")
            for i, chunk in enumerate(results):
                logger.info(f"  --- Chunk {i+1} (Score: {chunk.score:.4f}) ---")
                logger.info(f"    Source URL: {chunk.metadata.get('source_url')}")
                logger.info(f"    Text: {chunk.text[:100]}...")
        else:
            logger.warning("No results from RetrieverTool test.")

    except Exception as e:
        logger.error(f"Error during RetrieverTool direct test: {e}")
