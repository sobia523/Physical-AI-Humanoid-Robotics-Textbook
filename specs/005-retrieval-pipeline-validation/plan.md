# Implementation Plan: RAG Retrieval Pipeline Validation and Semantic Query Testing

**Feature Branch**: `005-retrieval-pipeline-validation`
**Feature Spec**: [spec.md](./spec.md)
**Status**: In Progress

## 1. Architecture Context

This feature (Spec-2) provides the crucial semantic retrieval layer, bridging the gap between:
- **Spec-1 (Ingestion & Vector Storage)**: Which populates the Qdrant vector database with embedded book content.
- **Spec-3 (Agent + FastAPI)**: Which will consume the retrieval logic to formulate responses.

Qdrant serves as the single source of truth for all retrievable knowledge. The retrieval logic developed here will be a standalone, testable module, ensuring its correctness and performance before integration into any higher-level agent system.

## 2. Retrieval Pipeline Design

### 2.1. Query Input Handling
-   **Input**: Accept a raw natural language query string (e.g., "What is a ROS 2 node?").
-   **Normalization/Preprocessing**: Basic cleanup (trim whitespace, lowercase if appropriate, though Cohere models are often robust to casing). No complex NLP preprocessing is initially planned.
-   **Embedding Generation**:
    -   Utilize the existing `EmbeddingGenerator` from Spec-1 (`backend/embeddings/generator.py`).
    -   Generate query embeddings using the same Cohere model (`embed-english-v3.0`, `input_type="search_query"`) that was used for document embedding (`input_type="search_document"`). This ensures embedding space consistency.

### 2.2. Vector Search Execution
-   **Target**: Qdrant collection (`rag_content_chunks`) populated by Spec-1.
-   **Similarity Search**: Perform vector similarity search using the query embedding against the stored document embeddings in Qdrant.
-   **Top-k Retrieval**: Retrieve a configurable number `k` of the most semantically similar chunks.
-   **Distance Metric**: Use `Cosine` similarity as configured in the Qdrant collection (from Spec-1).
-   **Relevance Scoring**: Qdrant's similarity score will be used directly to rank results.

### 2.3. Metadata-Based Filtering
-   **Optional Filters**: The retrieval function will accept optional parameters for filtering results:
    -   `source_url` (string, exact match)
    -   `page_title` (string, keyword match)
    -   `section` (string, keyword match)
-   **Filter Implementation**: Qdrant's payload filtering capabilities will be leveraged, using the indexes created on `source_url` (keyword) and `section` (text) in Spec-1.
-   **Support for Future Constraints**:
    -   **"answer using only selected text"**: Filters by `source_url` and `section` directly support this by narrowing down the context to specific documents or parts of documents.
    -   **Section-level or page-level retrieval**: Achieved by combining semantic search with appropriate `section` or `source_url` filters.

## 3. Test Scenario Design

Structured retrieval test cases will be defined and implemented in a dedicated test script (`backend/scripts/test_retrieval.py`).
-   **General Semantic Queries**:
    -   Example: "Explain ROS 2 nodes."
    -   Expected: Chunks from ROS 2 module covering node concepts.
-   **Specific Technical Queries**:
    -   Example: "How do I simulate physics in Gazebo?"
    -   Expected: Chunks from Module 2 (Digital Twin) specifically about Gazebo physics simulation.
-   **Edge Cases**:
    -   **Short Queries**: "ROS 2" -> Expected: Broad results related to ROS 2.
    -   **Ambiguous Terms**: "AI brain" -> Expected: Chunks from Module 3 (AI-Robot Brain).
    -   **Non-existent Phrases**: "quantum robot teleportation" -> Expected: Few or no relevant results, or very low scores.
-   **Filtered Queries**:
    -   Query: "ROS 2 nodes", Filter: `source_url` matching Module 1.
    -   Expected: Only ROS 2 node related chunks from Module 1.
    -   Query: "Gazebo simulation", Filter: `section` matching "Sensor simulation".
    -   Expected: Chunks about sensor simulation in Gazebo.

For each test case, the expected type of retrieved content and acceptable relevance threshold will be defined within the test code.

## 4. Evaluation & Quality Metrics

-   **Precision Indicators**: Percentage of retrieved top-k chunks that are contextually relevant to the query.
-   **Recall Indicators**: Ability to retrieve all expected relevant chunks from the database for a given query (qualitative assessment for specific test cases).
-   **Latency Benchmarks**: Measure the time taken for query embedding generation and Qdrant search.
-   **Failure Patterns**: Track instances of irrelevant or empty results for valid queries, and low-confidence matches.

Evaluation results will be crucial to determine readiness for Spec-3 agent integration.

## 5. Decisions Needing Documentation

All research and decisions will be documented in [research.md](./research.md). Key areas to investigate and justify:
-   **R-01**: Optimal `top-k` value for retrieval (number of chunks to retrieve).
-   **R-02**: Choice of similarity threshold (if any) to filter out low-relevance results.
-   **R-03**: Query embedding batching strategy for performance, considering potential caching for frequently asked questions.

## 6. Error Handling & Edge Cases

Plan for the retrieval module:
-   **Empty Retrieval Results**: If no matches found, return an empty list gracefully.
-   **Low-Confidence Matches**: Results below a certain (configurable) similarity threshold might be flagged or filtered out.
-   **Missing/Malformed Metadata**: Handle gracefully, perhaps logging a warning, but should not stop retrieval.
-   **Qdrant Connection Failures**: Implement retry mechanisms and robust error logging.

Errors will be logged to the existing `backend/logs/retrieval.log` (or similar) and surfaced to the calling function.

## 7. Testing & Validation Strategy

Validation checks will be aligned with the acceptance criteria from `spec.md`:
-   **Semantic Relevance**: Automated tests will verify if queries consistently return semantically relevant chunks (SC-001).
-   **Metadata Integrity**: Tests will ensure all retrieved chunks map correctly to source URLs and contain complete/usable metadata fields (SC-002).
-   **Performance**: Benchmarks will measure retrieval latency to ensure it meets SC-003 and SC-004.
-   **Reproducibility**: Tests will confirm retrieval results are stable across repeated runs for identical inputs (SC-005).

## 8. Quality & Governance

-   All architectural and quality rules from `sp.constitution` will be followed.
-   The plan is organized by logical phases:
    -   Query Design (embedding generation for query)
    -   Retrieval Execution (Qdrant search)
    -   Evaluation (metrics and test scenarios)
    -   Validation (acceptance criteria checks)

### Exclusions:
-   No agent orchestration.
-   No prompt engineering for LLMs.
-   No LLM response synthesis.
-   No frontend integration.