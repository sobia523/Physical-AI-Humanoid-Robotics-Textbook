
import sys
import os

# Add local directory to path so we can import agent modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from agent.rag_pipeline import ingest_docs

if __name__ == "__main__":
    print("--- Starting Professional Data Ingestion ---")
    print("This process will read your docs and upload them to Qdrant.")
    print("It respects API rate limits, so it might take a few minutes.")
    print("--------------------------------------------")
    
    ingest_docs()
    
    print("--------------------------------------------")
    print("Ingestion Process Finished.")
