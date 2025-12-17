import os
import glob
import time
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../.env"))

# Configure Gemini
GEN_API_KEY = os.getenv("GEMINI_API_KEY")
if GEN_API_KEY:
    genai.configure(api_key=GEN_API_KEY)

# Configure Qdrant
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "rag_content_chunks"

q_client = None
if QDRANT_URL and QDRANT_API_KEY:
    try:
        q_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    except Exception as e:
        print(f"Failed to init Qdrant: {e}")

def get_embedding(text):
    if not GEN_API_KEY: return None
    try:
        result = genai.embed_content(
            model="models/embedding-001",
            content=text,
            task_type="retrieval_document",
            title="Embedding of single chunk"
        )
        return result['embedding']
    except Exception as e:
        print(f"Embedding error: {e}")
        return None

def ingest_docs():
    """Reads all markdown files from website/docs and ingests into Qdrant."""
    if not q_client or not GEN_API_KEY:
        print("Skipping ingestion: Client or Key missing")
        return

    # Check if collection exists
    collections = q_client.get_collections().collections
    exists = any(c.name == COLLECTION_NAME for c in collections)
    
    if not exists:
        print(f"Creating collection {COLLECTION_NAME}...")
        q_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
        )
    
    # Check if empty
    count_result = q_client.count(collection_name=COLLECTION_NAME)
    if count_result.count > 0:
        print("Collection already has data. Skipping ingestion.")
        return

    print("Ingesting documents...")
    # Base path for docs
    base_docs_path = os.path.join(os.path.dirname(__file__), "../../website/docs")
    docs_path = os.path.join(base_docs_path, "**/*.md")
    files = glob.glob(docs_path, recursive=True)
    
    points = []
    idx = 0
    for f_path in files:
        try:
            with open(f_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Use relative path for citation source (e.g. module1/chapter1.md)
            rel_path = os.path.relpath(f_path, base_docs_path).replace("\\", "/")
            
            # Simple chunking by paragraphs for this repair
            chunks = content.split('\n\n')
            for chunk in chunks:
                if len(chunk.strip()) < 50: continue
                
                embedding = get_embedding(chunk)
                if embedding:
                    points.append(models.PointStruct(
                        id=idx,
                        vector=embedding,
                        payload={"text": chunk, "source": rel_path}
                    ))
                    idx += 1
                    
                    if len(points) >= 50:
                        q_client.upload_points(collection_name=COLLECTION_NAME, points=points)
                        print(f"Uploaded {len(points)} chunks...")
                        points = []
                        time.sleep(1) # Rate limit protection
        except Exception as e:
            print(f"Error reading {f_path}: {e}")

    if points:
        q_client.upload_points(collection_name=COLLECTION_NAME, points=points)
    print("Ingestion complete.")

# Trigger ingestion on import (simplified for "fix it" request)
try:
    ingest_docs()
except Exception as e:
    print(f"Ingestion failed: {e}")

async def get_answer(query: str, selected_text: str = None):
    if not GEN_API_KEY:
        return {"answer": "Error: GEMINI_API_KEY missing.", "citations": []}

    context_text = ""
    citations = []

    # 1. Retrieval
    if selected_text:
        context_text = selected_text
        citations.append({"source_url": "User Selection", "section": "Selection", "raw_text_snippet": selected_text})
    elif q_client:
        try:
            # Embed query
            q_emb = genai.embed_content(
                model="models/embedding-001",
                content=query,
                task_type="retrieval_query"
            )['embedding']
            
            search_result = q_client.search(
                collection_name=COLLECTION_NAME,
                query_vector=q_emb,
                limit=3
            )
            
            for hit in search_result:
                context_text += hit.payload.get('text', '') + "\n---\n"
                citations.append({
                    "source_url": hit.payload.get('source', 'Unknown'), 
                    "section": "Retrieved Context", 
                    "raw_text_snippet": hit.payload.get('text', '')[:100]
                })
        except Exception as e:
            print(f"Search failed: {e}")
            context_text = "Search failed, answering from general knowledge."

    # 2. Generation
    prompt = (
        "You are an expert AI Tutor for the textbook 'Physical AI & Humanoid Robotics'. "
        "Use the following retrieved context to answer the student's question clearly, academically, and accurately. "
        "If the context doesn't contain the answer, say so, but try to help based on the general domain knowledge of Robotics.\n\n"
    )
    
    if context_text:
        prompt += f"--- CONTEXT ---\n{context_text}\n---------------\n"
    
    prompt += f"Question: {query}\n"
    prompt += "Answer:"
    
    try:
        model = genai.GenerativeModel('gemini-pro')
        response = model.generate_content(prompt)
        return {
            "answer": response.text,
            "citations": citations,
            "message": "Generated by Gemini"
        }
    except Exception as e:
        return {"answer": f"Error: {str(e)}", "citations": []}

