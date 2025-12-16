import os
import sys
from typing import List, Dict, Any, Optional

# Add the 'backend' directory to the Python path for absolute imports
backend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Corrected imports
from embeddings.generator import EmbeddingGenerator
from vector_store.qdrant_manager import QdrantManager
from config.logging_config import setup_logging
from retrieval.models import RetrievedChunk # Import the new model

logger = setup_logging()

class Retriever:
    def __init__(self, collection_name: str = "rag_content_chunks"):
        self.collection_name = collection_name
        try:
            self.embedding_generator = EmbeddingGenerator()
            self.qdrant_manager = QdrantManager(collection_name=self.collection_name)
            self.qdrant_manager.ensure_collection_exists() # Ensure collection is ready
            logger.info("EmbeddingGenerator and QdrantManager initialized successfully in Retriever.")
        except Exception as e:
            logger.error(f"Failed to initialize services in Retriever: {e}")
            raise

    def _get_query_embedding(self, query: str) -> List[float]:
        """
        Generates an embedding for the given query string.
        """
        try:
            # Use input_type="search_query" for query embeddings
            original_input_type = self.embedding_generator.input_type
            self.embedding_generator.input_type = "search_query"
            embedding = self.embedding_generator.generate_embeddings([query])[0]
            self.embedding_generator.input_type = original_input_type # Reset
            logger.info(f"Generated embedding for query: '{query[:50]}...'")
            return embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding for query '{query[:50]}...': {e}")
            raise

    def search(self, query: str, top_k: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[RetrievedChunk]:
        """
        Performs a semantic search in Qdrant for top-k similar vectors and formats results as RetrievedChunk objects.

        Args:
            query: The natural language query string.
            top_k: The number of top similar results to retrieve.
            filters: Optional dictionary of payload filters (e.g., {"source_url": "some_url"}).

        Returns:
            A list of RetrievedChunk objects.
        """
        try:
            query_embedding = self._get_query_embedding(query)
            
            # Convert filters dict to Qdrant Filter object if provided
            qdrant_filter = None
            if filters:
                must_conditions = []
                for key, value in filters.items():
                    must_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )
                qdrant_filter = models.Filter(must=must_conditions)
                logger.info(f"Applying filters: {filters}")

            search_results = self.qdrant_manager.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                query_filter=qdrant_filter,
                with_payload=True, # Retrieve original metadata
                with_vectors=False # No need to retrieve vectors again
            )
            logger.info(f"Retrieved {len(search_results)} raw results for query: '{query[:50]}...'")
            
            retrieved_chunks: List[RetrievedChunk] = []
            for result in search_results:
                retrieved_chunks.append(
                    RetrievedChunk(
                        text=result.payload.get('raw_text', ''), # Use 'raw_text' from payload
                        metadata=result.payload,
                        score=result.score
                    )
                )
            return retrieved_chunks
        except Exception as e:
            logger.error(f"Error during Qdrant search for query '{query[:50]}...': {e}")
            raise

if __name__ == '__main__':
    # This block is for testing the script directly
    
    try:
        logger.info("Starting Retriever test for query embedding generation and basic search.")
        retriever = Retriever()

        test_query = "What is a ROS 2 node?"
        logger.info(f"Searching for: '{test_query}'")
        search_results = retriever.search(test_query, top_k=3)
        
        if search_results:
            logger.info(f"Retrieved {len(search_results)} results:")
            for i, chunk in enumerate(search_results):
                logger.info(f"--- Result {i+1} (Score: {chunk.score:.4f}) ---")
                logger.info(f"Source URL: {chunk.metadata.get('source_url')}")
                logger.info(f"Section: {chunk.metadata.get('section')}")
                logger.info(f"Text: {chunk.text[:100]}...")
        else:
            logger.warning("No results found for the test query.")

        # Test with filters (if collection has data with these fields)
        test_query_filtered = "What is a ROS 2 node?"
        test_filters = {"section": "Understanding ROS 2 Nodes"} # Example filter
        logger.info(f"\nSearching for: '{test_query_filtered}' with filters: {test_filters}")
        filtered_results = retriever.search(test_query_filtered, top_k=2, filters=test_filters)

        if filtered_results:
            logger.info(f"Retrieved {len(filtered_results)} filtered results:")
            for i, chunk in enumerate(filtered_results):
                logger.info(f"--- Filtered Result {i+1} (Score: {chunk.score:.4f}) ---")
                logger.info(f"Source URL: {chunk.metadata.get('source_url')}")
                logger.info(f"Section: {chunk.metadata.get('section')}")
                logger.info(f"Text: {chunk.text[:100]}...")
        else:
            logger.warning("No filtered results found for the test query.")


    except ValueError as e:
        logger.error(f"Configuration Error: {e}")
        logger.error("Please ensure COHERE_API_KEY and QDRANT_URL are set in backend/.env with valid keys/URLs.")
    except Exception as e:
        logger.error(f"An unexpected error occurred during Retriever test: {e}")
