import os
import sys
import time # For latency measurement
from typing import List, Dict, Any

# Add the 'backend' directory to the Python path for absolute imports
backend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from retrieval.retriever import Retriever
from config.logging_config import setup_logging
from retrieval.models import RetrievedChunk

logger = setup_logging(log_file_name="retrieval_test.log")

def print_results(results: List[RetrievedChunk], title: str = "Results"):
    logger.info(f"\n--- {title} ---")
    if results:
        logger.info(f"Retrieved {len(results)} results:")
        for i, chunk in enumerate(results):
            logger.info(f"  --- Result {i+1} (Score: {chunk.score:.4f}) ---")
            logger.info(f"    Source URL: {chunk.metadata.get('source_url')}")
            logger.info(f"    Page Title: {chunk.metadata.get('page_title')}")
            logger.info(f"    Section: {chunk.metadata.get('section')}")
            logger.info(f"    Chunk Index: {chunk.metadata.get('chunk_index')}")
            logger.info(f"    Text: {chunk.text[:150]}...")
            if len(chunk.text) > 150:
                logger.info("    ...")
    else:
        logger.warning("  No results found.")

def run_retrieval_tests():
    logger.info("--- Starting Retrieval Test Script ---")

    try:
        retriever = Retriever()

        # --- Test Case 1: General Semantic Query ---
        query1 = "What are ROS 2 nodes?"
        start_time = time.perf_counter()
        results1 = retriever.search(query1, top_k=5)
        end_time = time.perf_counter()
        print_results(results1, f"Test Case 1: '{query1}' (unfiltered, Latency: {(end_time - start_time)*1000:.2f}ms)")
        
        # Metadata Integrity Check for Test Case 1
        if results1:
            for i, chunk in enumerate(results1):
                if not all(chunk.metadata.get(field) for field in ['source_url', 'page_title', 'section', 'chunk_index', 'raw_text']):
                    logger.error(f"Metadata Validation Failed for Result {i+1} in Test Case 1: Missing critical fields.")
                if not chunk.text == chunk.metadata.get('raw_text'):
                    logger.error(f"Metadata Validation Failed for Result {i+1} in Test Case 1: 'text' and 'raw_text' mismatch.")
            logger.info("Metadata integrity checks passed for Test Case 1 results.")


        # --- Test Case 2: Specific Technical Query ---
        query2 = "How do I simulate physics in Gazebo?"
        start_time = time.perf_counter()
        results2 = retriever.search(query2, top_k=3)
        end_time = time.perf_counter()
        print_results(results2, f"Test Case 2: '{query2}' (unfiltered, Latency: {(end_time - start_time)*1000:.2f}ms)")

        # --- Test Case 3: Edge Case - Short Query ---
        query3 = "ROS 2"
        start_time = time.perf_counter()
        results3 = retriever.search(query3, top_k=2)
        end_time = time.perf_counter()
        print_results(results3, f"Test Case 3: '{query3}' (short query, Latency: {(end_time - start_time)*1000:.2f}ms)")

        # --- Test Case 4: Filtered Query by Source URL ---
        query4 = "humanoid robot control"
        # This filter URL should ideally come from the ingested content
        filter_url = "https://your-docusaurus-site.example.com/docs/module1-ros2-humanoid-control/chapter1"
        filters4 = {"source_url": filter_url}
        start_time = time.perf_counter()
        results4 = retriever.search(query4, top_k=5, filters=filters4)
        end_time = time.perf_counter()
        print_results(results4, f"Test Case 4: '{query4}' filtered by source_url='{filter_url}' (Latency: {(end_time - start_time)*1000:.2f}ms)")
        
        # Verify filter for Test Case 4
        if results4 and not all(chunk.metadata.get('source_url') == filter_url for chunk in results4):
            logger.error(f"Filter Validation Failed for Test Case 4: Not all results match source_url filter.")
        else:
            logger.info("Filter validation passed for Test Case 4 results.")

        # --- Test Case 5: Filtered Query by Section ---
        query5 = "Isaac Sim features"
        # This filter section should ideally come from the ingested content
        filter_section = "NVIDIA Isaac Sim: photorealistic simulation and synthetic data generation"
        filters5 = {"section": filter_section}
        start_time = time.perf_counter()
        results5 = retriever.search(query5, top_k=5, filters=filters5)
        end_time = time.perf_counter()
        print_results(results5, f"Test Case 5: '{query5}' filtered by section='{filter_section}' (Latency: {(end_time - start_time)*1000:.2f}ms)")

        # Verify filter for Test Case 5
        if results5 and not all(chunk.metadata.get('section') == filter_section for chunk in results5):
            logger.error(f"Filter Validation Failed for Test Case 5: Not all results match section filter.")
        else:
            logger.info("Filter validation passed for Test Case 5 results.")


    except ValueError as e:
        logger.error(f"Configuration Error: {e}")
        logger.error("Please ensure COHERE_API_KEY and QDRANT_URL are set in backend/.env with valid keys/URLs.")
    except Exception as e:
        logger.error(f"An unexpected error occurred during retrieval tests: {e}")
    
    logger.info("--- Retrieval Test Script Finished ---")

if __name__ == '__main__':
    run_retrieval_tests()