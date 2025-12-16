# PHR: Create Implementation Plan for Content Ingestion

- **ID**: 2
- **TITLE**: Create Implementation Plan for Content Ingestion
- **STAGE**: plan
- **DATE_ISO**: 2025-12-16
- **SURFACE**: agent
- **MODEL**: gemini-pro
- **FEATURE**: 001-content-ingestion-pipeline
- **BRANCH**: 001-content-ingestion-pipeline
- **USER**: TLS
- **COMMAND**: `/sp.plan`
- **LABELS**: ["planning", "architecture", "rag", "ingestion"]
- **LINKS**:
  - **SPEC**: `specs/001-content-ingestion-pipeline/spec.md`
  - **PLAN**: `specs/001-content-ingestion-pipeline/plan.md`
  - **TICKET**: null
  - **ADR**: null
  - **PR**: null
- **FILES_YAML**:
  - `specs/001-content-ingestion-pipeline/plan.md`
  - `specs/001-content-ingestion-pipeline/research.md`
  - `specs/001-content-ingestion-pipeline/data-model.md`
  - `specs/001-content-ingestion-pipeline/quickstart.md`
- **TESTS_YAML**:
  - "none"
- **PROMPT_TEXT**:
  ```
  /sp.plan

  Objective:
  Design and document a complete ingestion, embedding, and vector-storage pipeline
  for a RAG chatbot using deployed book content, starting from initial backend
  project setup through validated vector retrieval readiness.

  --------------------------------------------------
  INITIAL PROJECT SETUP (FOUNDATION PHASE)
  --------------------------------------------------

  Create:
  - A dedicated backend workspace for the RAG system
  - A reproducible Python environment using UV

  Steps to plan and document:
  1. Create a `backend/` directory at the project root
  2. Initialize a UV-managed Python project inside `backend/`
  3. Define dependency boundaries for:
    - Web scraping and parsing
    - Embedding generation
    - Vector database communication
  4. Establish environment configuration patterns:
    - API keys (Cohere, Qdrant)
    - Deployment-safe `.env` handling
  5. Define folder structure for long-term scalability:
    - ingestion/
    - embeddings/
    - vector_store/
    - config/
    - scripts/
    - logs/

  Document rationale for:
  - Choosing UV over traditional virtualenv/poetry
  - Separation of ingestion logic from later agent/runtime logic

  --------------------------------------------------
  ARCHITECTURE SKETCH
  --------------------------------------------------

  Create:
  - High-level system architecture diagram showing:
    - Deployed Docusaurus website
    - Sitemap-driven URL discovery
    - Content extraction layer
    - Chunking and preprocessing
    - Cohere embedding generation
    - Qdrant vector storage
  - Clear data flow arrows from source → vector DB

  Explicit inputs:
  - Base site URL:
    https://physical-ai-humanoid-robotics-textb-beta-two.vercel.app/
  - Sitemap source:
    https://physical-ai-humanoid-robotics-textb-beta-two.vercel.app/sitemap.xml

  --------------------------------------------------
  EXECUTION FLOW (PIPELINE DESIGN)
  --------------------------------------------------

  Plan and explain each phase in detail:

  Phase 1: URL Discovery
  - Fetch and parse sitemap.xml
  - Filter valid documentation/book URLs
  - Handle pagination or nested sitemap entries if present

  Phase 2: Content Extraction
  - Fetch HTML content for each URL
  - Strip navigation, footer, and non-content elements
  - Preserve semantic structure (headings, sections)

  Phase 3: Chunking Strategy
  - Define chunk size and overlap
  - Explain tradeoffs between:
    - Fixed-size chunks
    - Heading-based chunks
  - Ensure chunking supports:
    - Precise citation
    - “answer from selected text only” in future specs

  Phase 4: Embedding Generation
  - Use Cohere embedding models
  - Batch requests safely to avoid rate limits
  - Ensure deterministic input ordering
  - Validate embedding dimensional consistency

  Phase 5: Vector Storage (Qdrant)
  - Define Qdrant collection configuration:
    - Distance metric
    - Payload indexing
  - Store embeddings with full metadata:
    - Source URL
    - Page title
    - Section/heading
    - Chunk index
    - Raw text
  - Ensure idempotent insertion (safe re-runs)

  --------------------------------------------------
  DATA MODEL & METADATA DESIGN
  --------------------------------------------------

  Document:
  - Vector schema
  - Payload structure
  - Naming conventions for chunk IDs
  - How metadata enables:
    - Source traceability
    - Section-level answers
    - Filtered retrieval in later specs

  --------------------------------------------------
  DECISIONS NEEDING DOCUMENTATION
  --------------------------------------------------

  Explicitly list and justify:
  - Sitemap-based discovery vs manual URL lists
  - HTML parsing approach and libraries
  - Chunk size and overlap values
  - Embedding model choice and tradeoffs
  - Qdrant configuration decisions

  --------------------------------------------------
  TESTING & VALIDATION STRATEGY
  --------------------------------------------------

  Define validation checks aligned with acceptance criteria:

  - Sitemap successfully parsed and URLs discovered
  - All URLs ingested without fatal pipeline failure
  - Embeddings generated for every chunk
  - Vectors stored and queryable in Qdrant
  - Sample semantic queries return relevant chunks
  - Metadata correctly maps results back to source URLs
  - Re-running pipeline does not create duplicates

  --------------------------------------------------
  QUALITY & GOVERNANCE
  --------------------------------------------------

  Follow:
  - All global architectural, formatting, and quality rules defined in sp.constitution

  Organize the plan by phases:
  - Foundation → Discovery → Extraction → Embedding → Storage → Validation

  Explicitly exclude:
  - Agent logic
  - Retrieval orchestration
  - Frontend integration
  - User-facing APIs
  ```
- **RESPONSE_TEXT**:
  ```
  The implementation plan for the 'RAG Content Ingestion Pipeline' has been created. All planning artifacts have been generated and are located in the `specs/001-content-ingestion-pipeline/` directory. The feature is ready for the task breakdown phase.
  ```
