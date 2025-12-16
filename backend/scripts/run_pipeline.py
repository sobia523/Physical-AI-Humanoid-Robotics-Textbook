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
from ingestion.discovery import discover_urls
from ingestion.extraction import extract_content
from ingestion.chunking import chunk_content, count_tokens
from embeddings.generator import EmbeddingGenerator
from vector_store.qdrant_manager import QdrantManager

logger = setup_logging()

# --- Configuration ---
SITEMAP_URL = os.getenv("SITEMAP_URL", "https://physical-ai-humanoid-robotics-textb-beta-two.vercel.app/sitemap.xml")
# Filter out placeholder URLs from Docusaurus default sitemap if necessary
BASE_URL_TO_FILTER = os.getenv("BASE_URL_TO_FILTER", "https://your-docusaurus-site.example.com")
# Only ingest documentation pages, ignore blog posts etc.
DOCS_PATH_SEGMENT = os.getenv("DOCS_PATH_SEGMENT", "/docs/")
# Name of the Qdrant collection
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "rag_content_chunks")
# Cohere embedding model max input tokens is 512
MAX_CHUNK_TEXT_LENGTH = 500 # Slightly less than 512 to be safe with tokens


def run_pipeline():
    logger.info("--- Starting RAG Content Ingestion Pipeline ---")

    # 1. URL Discovery
    logger.info(f"Discovering URLs from sitemap: {SITEMAP_URL}...")
    all_urls = discover_urls(SITEMAP_URL)
    if not all_urls:
        logger.error("No URLs discovered. Exiting pipeline.")
        return

    # Filter out placeholder URLs and non-documentation pages
    filtered_urls = [
        url for url in all_urls 
        if BASE_URL_TO_FILTER not in url and DOCS_PATH_SEGMENT in url
    ]
    logger.info(f"Found {len(all_urls)} URLs, filtered down to {len(filtered_urls)} relevant documentation URLs.")
    
    if not filtered_urls:
        logger.warning("No relevant documentation URLs found after filtering. Exiting pipeline.")
        return

    # Initialize Embedding Generator and Qdrant Manager
    try:
        embedding_generator = EmbeddingGenerator()
        qdrant_manager = QdrantManager(collection_name=QDRANT_COLLECTION_NAME)
        qdrant_manager.ensure_collection_exists()
    except Exception as e:
        logger.error(f"Error initializing services: {e}. Exiting pipeline.")
        return

    processed_chunks_count = 0
    skipped_urls_count = 0

    for url in filtered_urls:
        logger.info(f"\nProcessing URL: {url}")
        try:
            # 2. Content Extraction
            soup_content = extract_content(url)
            if not soup_content:
                logger.warning(f"Skipping {url}: No main content extracted.")
                skipped_urls_count += 1
                continue

            # 3. Chunking Strategy
            # Get page title from the extracted content
            page_title_tag = soup_content.find('title')
            page_title = page_title_tag.get_text(strip=True) if page_title_tag else "Untitled Page"
            
            chunks_data = chunk_content(soup_content, url)
            
            if not chunks_data:
                logger.warning(f"Skipping {url}: No chunks generated.")
                skipped_urls_count += 1
                continue

            # Prepare texts for embedding and add page_title to payload
            texts_to_embed: List[str] = []
            for chunk in chunks_data:
                # Ensure chunk text is not too long for embedding model
                if count_tokens(chunk['text']) > MAX_CHUNK_TEXT_LENGTH:
                    # Simple truncation for now, can be improved
                    logger.warning(f"Chunk text for {url} (index {chunk['chunk_index']}) too long. Truncating.")
                    chunk['text'] = ' '.join(chunk['text'].split()[:MAX_CHUNK_TEXT_LENGTH])
                chunk['page_title'] = page_title # Add page title to chunk metadata
                texts_to_embed.append(chunk['text'])

            # 4. Embedding Generation
            if texts_to_embed:
                logger.info(f"Generating embeddings for {len(texts_to_embed)} chunks from {url}...")
                embeddings = embedding_generator.generate_embeddings(texts_to_embed)
                if len(embeddings) != len(texts_to_embed):
                    logger.error(f"Error: Mismatch in number of texts and embeddings for {url}. Skipping.")
                    continue

                # Combine chunks with their embeddings
                chunks_with_embeddings = []
                for i, chunk in enumerate(chunks_data):
                    chunk_with_embedding = chunk.copy()
                    chunk_with_embedding['embedding'] = embeddings[i]
                    chunks_with_embeddings.append(chunk_with_embedding)
                
                # 5. Vector Storage (Qdrant)
                logger.info(f"Upserting {len(chunks_with_embeddings)} chunks for {url} into Qdrant...")
                if qdrant_manager.upsert_vectors(chunks_with_embeddings):
                    processed_chunks_count += len(chunks_with_embeddings)
                else:
                    logger.error(f"Failed to upsert chunks for {url}.")
            else:
                logger.info(f"No texts to embed for {url}.")


        except Exception as e:
            logger.error(f"An unexpected error occurred while processing {url}: {e}")
            skipped_urls_count += 1
            continue

    logger.info("\n--- Pipeline Summary ---")
    logger.info(f"Total relevant URLs considered: {len(filtered_urls)}")
    logger.info(f"URLs skipped due to errors or no content: {skipped_urls_count}")
    logger.info(f"Total chunks processed and upserted to Qdrant: {processed_chunks_count}")
    logger.info("--- Pipeline Finished ---")

if __name__ == '__main__':
    run_pipeline()
