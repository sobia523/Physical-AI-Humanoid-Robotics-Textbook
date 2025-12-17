import os
import time
from typing import List, Dict, Optional
import cohere
from cohere import UnauthorizedError # Correct import for API errors
import requests # For network errors
from dotenv import load_dotenv
import sys

# Add the backend directory to the Python path for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from config.logging_config import setup_logging

logger = setup_logging()

# Load environment variables from .env file
# Use a more robust path for dotenv to ensure it's loaded from the backend directory
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '..', '.env'))

class EmbeddingGenerator:
    def __init__(self):
        self.api_key = os.getenv("COHERE_API_KEY")
        if not self.api_key or self.api_key == "test_api_key": # Check for placeholder key as well
            logger.error("COHERE_API_KEY not found or is a placeholder in environment variables. Please set it in backend/.env")
            raise ValueError("COHERE_API_KEY not found or is a placeholder in environment variables. Please set it in backend/.env")
        
        # Initialize Cohere client
        try:
            self.co = cohere.Client(self.api_key)
            logger.info("Cohere client initialized successfully.")
        except Exception as e:
            logger.error(f"Failed to initialize Cohere client: {e}. Check your API key.")
            raise ValueError(f"Failed to initialize Cohere client: {e}. Check your API key.")

        self.model = "embed-english-v3.0" # Or embed-multilingual-v3.0 if needed
        self.input_type = "search_document" # Or "search_query"

    def generate_embeddings(self, texts: List[str], batch_size: int = 96, max_retries: int = 5, retry_delay: int = 5) -> List[List[float]]:
        """
        Generates embeddings for a list of texts using Cohere API, with batching and retry logic.

        Args:
            texts: A list of text strings to embed.
            batch_size: The number of texts to send in each API request.
            max_retries: Maximum number of retries for API requests.
            retry_delay: Delay in seconds between retries.

        Returns:
            A list of embeddings, where each embedding is a list of floats.
        """
        embeddings: List[List[float]] = []
        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            
            for attempt in range(max_retries):
                try:
                    logger.info(f"Attempt {attempt + 1}/{max_retries}: Generating embeddings for batch of {len(batch)} texts.")
                    response = self.co.embed(
                        texts=batch,
                        model=self.model,
                        input_type=self.input_type,
                        embedding_types=['float']
                    )
                    embeddings.extend(response.embeddings)
                    logger.info(f"Successfully generated embeddings for batch starting with: '{batch[0][:50]}...'")
                    break # Exit retry loop on success
                except UnauthorizedError as e: # Catch UnauthorizedError specifically
                    logger.error(f"Cohere API Unauthorized Error (Attempt {attempt + 1}/{max_retries}): {e}. Please check your COHERE_API_KEY.")
                    raise # Re-raise immediately for invalid API key
                except cohere.error.CohereError as e: # Catch other generic Cohere API errors if needed
                    logger.error(f"Cohere API Error (Attempt {attempt + 1}/{max_retries}): {e}")
                    # Check for rate limit error specifically if possible, otherwise generic retry
                    if "rate limit" in str(e).lower() and attempt < max_retries - 1:
                        logger.warning(f"Rate limit hit. Retrying in {retry_delay} seconds...")
                        time.sleep(retry_delay)
                    elif attempt == max_retries - 1:
                        logger.error(f"Max retries reached. Failed to embed batch starting with: '{batch[0][:50]}...'")
                        raise # Re-raise after max retries
                    else: # Other API errors
                        raise
                except requests.exceptions.RequestException as e: # Catch network errors
                    logger.error(f"Network Error (Attempt {attempt + 1}/{max_retries}): {e}")
                    if attempt < max_retries - 1:
                        logger.warning(f"Retrying in {retry_delay} seconds...")
                        time.sleep(retry_delay)
                    else:
                        raise
                except Exception as e:
                    logger.error(f"An unexpected error occurred (Attempt {attempt + 1}/{max_retries}): {e}")
                    if attempt < max_retries - 1:
                        logger.warning(f"Retrying in {retry_delay} seconds...")
                        time.sleep(retry_delay)
                    else:
                        raise

        return embeddings

if __name__ == '__main__':
    # This block is for testing the script directly
    
    try:
        logger.info("Starting embedding generator test.")
        generator = EmbeddingGenerator()
        test_texts = [
            "This is the first sentence.",
            "This sentence is about artificial intelligence.",
            "Another example text for embedding.",
            "The quick brown fox jumps over the lazy dog."
        ]
        logger.info(f"Generating embeddings for {len(test_texts)} texts...")
        
        # Test with a smaller batch size to simulate batching
        test_embeddings = generator.generate_embeddings(test_texts, batch_size=2)
        
        if test_embeddings:
            logger.info(f"Successfully generated {len(test_embeddings)} embeddings.")
            logger.info(f"First embedding (first 5 dimensions): {test_embeddings[0][:5]}...")
            logger.info(f"Embedding dimension: {len(test_embeddings[0])}")
        else:
            logger.warning("No embeddings generated.")

    except ValueError as e:
        logger.error(f"Configuration Error: {e}")
        logger.error("Please ensure COHERE_API_KEY is set in backend/.env with a valid key.")
    except UnauthorizedError as e:
        logger.error(f"Cohere API Unauthorized Error during test: {e}.")
        logger.error("Check your COHERE_API_KEY. It appears to be invalid.")
    except cohere.error.CohereError as e: # Catch other generic Cohere API errors if needed
        logger.error(f"Cohere API Error during test: {e}.")
        logger.error("A generic Cohere API error occurred. Check network or API status.")
    except requests.exceptions.RequestException as e:
        logger.error(f"Network Error during test: {e}.")
        logger.error("Check your internet connection.")
    except Exception as e:
        logger.error(f"An unexpected error occurred during test: {e}")
