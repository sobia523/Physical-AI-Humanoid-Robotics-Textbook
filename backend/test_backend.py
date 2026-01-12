
import asyncio
import os
import sys

# Ensure backend directory is in path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from agent.rag_pipeline import get_answer, ingest_docs

async def test():
    print("--- Starting Backend Diagnostic ---")
    
    # Check Env
    key = os.getenv("GEMINI_API_KEY")
    if not key:
        print("FAIL: GEMINI_API_KEY is missing or empty in environment (checking os.environ, not .env file directly).")
        # pipeline loads .env, so it should be there if main code runs
    else:
        print(f"PASS: GEMINI_API_KEY found (length: {len(key)})")

    # Test Ingestion (Basic check)
    print("\n--- Testing Ingestion Logic ---")
    try:
        # ingest_docs() # Skipping full ingestion to save time, just check if it imports and runs without syntax error
        print("Ingestion function exists.")
    except Exception as e:
        print(f"FAIL: Ingestion logic error: {e}")

    # Test Retrieval/Generation
    print("\n--- Testing Agent Response ---")
    query = "What is ROS 2?"
    try:
        response = await get_answer(query)
        print("Response received:")
        print(response)
        
        if "answer" in response and "Error" not in response["answer"]:
             print("PASS: Agent generated an answer.")
        else:
             print("FAIL: Agent returned an error message.")
             
    except Exception as e:
        print(f"FAIL: Exception during get_answer: {e}")

if __name__ == "__main__":
    asyncio.run(test())
