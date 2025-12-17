import os
import hashlib
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
import sys

# Get the path to the 'backend' directory (where this script resides or a parent)
current_script_dir = os.path.dirname(os.path.abspath(__file__))
# For qdrant_manager.py, '..' goes to 'backend'
backend_root_dir = os.path.abspath(os.path.join(current_script_dir, '..'))

# Insert the backend root to sys.path if it's not already there
if backend_root_dir not in sys.path:
    sys.path.insert(0, backend_root_dir)

from config.logging_config import setup_logging

logger = setup_logging()

# Load environment variables from .env file
load_dotenv(dotenv_path=os.path.join(backend_root_dir, '.env'))

class QdrantManager:
    def __init__(self, collection_name: str = "rag_content_chunks"):
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY") # Optional, depending on Qdrant setup
        self.collection_name = collection_name

        if not self.qdrant_url:
            logger.error("QDRANT_URL not found in environment variables.")
            raise ValueError("QDRANT_URL not found in environment variables.")
        
        try:
            self.client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key,
            )
            logger.info("Qdrant client initialized successfully.")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}. Check QDRANT_URL or API key.")
            raise ValueError(f"Failed to initialize Qdrant client: {e}. Check QDRANT_URL or API key.")

        self.vector_size = 1024 # Cohere embed-english-v3.0 typically produces 1024-dim vectors
                                 # This should ideally be dynamically determined or passed in

    def ensure_collection_exists(self):
        """
        Ensures the Qdrant collection exists with the correct configuration.
        """
        try:
            if not self.client.collection_exists(collection_name=self.collection_name):
                logger.info(f"Collection '{self.collection_name}' does not exist. Creating it.")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size, 
                        distance=models.Distance.COSINE
                    ),
                    optimizers_config=models.OptimizersConfig(
                        default_segment_number=2
                    ),
                    replication_factor=1 # Adjust based on deployment needs
                )
                logger.info(f"Collection '{self.collection_name}' created.")
                
                # Create payload indexes as defined in research.md
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="source_url",
                    field_schema=models.FieldSchema(
                        type=models.FieldType.KEYWORD, 
                        is_indexed=True
                    )
                )
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section",
                    field_schema=models.FieldSchema(
                        type=models.FieldType.TEXT, # Use TEXT for free-form section names
                        is_indexed=True
                    )
                )
                logger.info("Payload indexes for 'source_url' and 'section' created.")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists.")
        except Exception as e:
            logger.error(f"Error ensuring Qdrant collection '{self.collection_name}' exists: {e}")
            raise

    def _generate_point_id(self, metadata: Dict[str, Any]) -> str:
        """
        Generates a deterministic point ID from chunk metadata.
        This ensures idempotency: same metadata always results in the same ID.
        """
        # Include raw_text in hash to ensure uniqueness even if other metadata is identical
        unique_key = f"{metadata.get('source_url', '')}-{metadata.get('section', '')}-{metadata.get('chunk_index', '')}-{metadata.get('raw_text', '')}"
        return hashlib.sha256(unique_key.encode('utf-8')).hexdigest()

    def upsert_vectors(self, chunks_with_embeddings: List[Dict[str, Any]]) -> bool:
        """
        Upserts (inserts or updates) a list of vector chunks into the Qdrant collection.

        Args:
            chunks_with_embeddings: A list of dictionaries, where each dict contains:
                                    'text', 'section', 'chunk_index', 'source_url', 'embedding'.

        Returns:
            True if upsert operation was successful, False otherwise.
        """
        points = []
        for chunk_data in chunks_with_embeddings:
            payload = {k: v for k, v in chunk_data.items() if k != 'embedding'}
            point_id = self._generate_point_id(payload)

            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=chunk_data['embedding'],
                    payload=payload
                )
            )

        try:
            operation_info = self.client.upsert(
                collection_name=self.collection_name,
                wait=True,
                points=points
            )
            if operation_info.status == models.UpdateStatus.COMPLETED:
                logger.info(f"Upsert operation completed successfully for {len(points)} points.")
                return True
            else:
                logger.warning(f"Upsert operation completed with status: {operation_info.status}")
                return False
        except Exception as e:
            logger.error(f"Error during upsert operation: {e}")
            return False

if __name__ == '__main__':
    # This block is for testing the script directly
    
    try:
        logger.info("Starting Qdrant manager test.")
        manager = QdrantManager()
        manager.ensure_collection_exists()

        # Dummy embeddings (should match self.vector_size)
        dummy_embedding_1 = [0.1] * manager.vector_size
        dummy_embedding_2 = [0.2] * manager.vector_size

        test_chunks_with_embeddings = [
            {
                "text": "This is a test chunk one.",
                "section": "Test Section 1",
                "chunk_index": 0,
                "source_url": "http://test-url.com/page1",
                "page_title": "Test Page 1",
                "embedding": dummy_embedding_1
            },
            {
                "text": "This is a test chunk two, slightly different.",
                "section": "Test Section 1",
                "chunk_index": 1,
                "source_url": "http://test-url.com/page1",
                "page_title": "Test Page 1",
                "embedding": dummy_embedding_2
            },
            {
                "text": "Another page's chunk.",
                "section": "Another Section",
                "chunk_index": 0,
                "source_url": "http://test-url.com/page2",
                "page_title": "Test Page 2",
                "embedding": dummy_embedding_1 # Same embedding for simplicity
            }
        ]

        logger.info(f"Upserting {len(test_chunks_with_embeddings)} test vectors...")
        success = manager.upsert_vectors(test_chunks_with_embeddings)

        if success:
            logger.info("Test vectors upserted successfully.")
        else:
            logger.error("Failed to upsert test vectors.")

    except ValueError as e:
        logger.error(f"Configuration Error: {e}")
        logger.error("Please ensure QDRANT_URL is set in backend/.env with a valid URL.")
    except Exception as e:
        logger.error(f"An unexpected error occurred during test: {e}")
