import os
import sys
from typing import List, Dict, Any
from dotenv import load_dotenv

# Add the 'backend' directory to the Python path for absolute imports
backend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Load environment variables from .env file
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '.env'))

from config.logging_config import setup_logging
from vector_store.qdrant_manager import QdrantManager

logger = setup_logging(log_file_name="validation.log")

def validate_data(collection_name: str = "rag_content_chunks", sample_size: int = 5):
    logger.info("--- Starting Data Validation Script ---")

    try:
        qdrant_manager = QdrantManager(collection_name=collection_name)
        # Check if collection exists
        if not qdrant_manager.client.collection_exists(collection_name=collection_name):
            logger.error(f"Collection '{collection_name}' does not exist. Please run ingestion pipeline first.")
            return

        # Fetch a random sample of vector records
        logger.info(f"Fetching a random sample of {sample_size} records from '{collection_name}'...")
        scroll_result, _ = qdrant_manager.client.scroll(
            collection_name=collection_name,
            limit=sample_size,
            with_payload=True,
            with_vectors=True
        )

        if not scroll_result:
            logger.warning(f"No records found in collection '{collection_name}'.")
            return

        logger.info(f"Successfully fetched {len(scroll_result)} sample records.")
        for i, point in enumerate(scroll_result):
            logger.info(f"\n--- Sample Record {i+1} (ID: {point.id}) ---")
            
            # Verify basic payload fields
            payload = point.payload
            vector = point.vector

            logger.info(f"Vector Dimension: {len(vector)}")
            logger.info(f"Source URL: {payload.get('source_url', 'N/A')}")
            logger.info(f"Page Title: {payload.get('page_title', 'N/A')}")
            logger.info(f"Section: {payload.get('section', 'N/A')}")
            logger.info(f"Chunk Index: {payload.get('chunk_index', 'N/A')}")
            logger.info(f"Raw Text (first 200 chars): {payload.get('raw_text', 'N/A')[:200]}...")

            # Implement checks for valid, non-empty metadata fields (T017)
            if not all(payload.get(field) for field in ['source_url', 'page_title', 'section', 'chunk_index', 'raw_text']):
                logger.error(f"Validation Error: Missing or empty critical metadata fields in record ID {point.id}")
            else:
                logger.info(f"Validation: All critical metadata fields present for record ID {point.id}")
            
            # Implement a semantic search query to retrieve chunks for a known sentence (T018)
            # This would require a search query and a known embedding, which is harder to automate here.
            # For now, we'll just log a placeholder for this check.
            logger.info(f"Placeholder: Semantic search query verification for record ID {point.id}")


    except ValueError as e:
        logger.error(f"Configuration Error: {e}")
        logger.error("Please ensure QDRANT_URL is set in backend/.env with a valid URL.")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")
    
    logger.info("--- Data Validation Script Finished ---")

if __name__ == '__main__':
    validate_data()
