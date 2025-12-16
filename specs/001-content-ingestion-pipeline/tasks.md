# Tasks: RAG Content Ingestion Pipeline

**Input**: Design documents from `/specs/001-content-ingestion-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Tests**: A validation script is included as part of User Story 2 to meet the acceptance criteria. No unit or integration tests are included in this task list.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the backend project and create the necessary directory and file structure.

- [ ] T001 Create the root directory for the pipeline at `backend/`
- [ ] T002 [P] Inside `backend/`, initialize a Python project using `uv init`
- [ ] T003 [P] Create the directory structure: `backend/config/`, `backend/ingestion/`, `backend/embeddings/`, `backend/vector_store/`, `backend/scripts/`, `backend/logs/`
- [ ] T004 [P] Create empty Python files: `backend/ingestion/discovery.py`, `backend/ingestion/extraction.py`, `backend/ingestion/chunking.py`, `backend/embeddings/generator.py`, `backend/vector_store/qdrant_manager.py`, `backend/scripts/run_pipeline.py`
- [ ] T005 [P] Create the dependencies file `backend/requirements.txt` with initial packages: `requests`, `beautifulsoup4`, `cohere`, `qdrant-client`, `python-dotenv`
- [ ] T006 [P] Create the example environment file `backend/.env.example` for required API keys (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Ensure the environment is configured before implementation begins.

- [ ] T007 Configure local environment by creating a `backend/.env` file and populating it with valid API keys.

---

## Phase 3: User Story 1 - Content Pipeline Execution (Priority: P1) 🎯 MVP

**Goal**: Implement the end-to-end ingestion pipeline that discovers, chunks, embeds, and stores content from the Docusaurus website.

**Independent Test**: Run `python backend/scripts/run_pipeline.py` and verify that the Qdrant database is populated with vector records containing the expected metadata and text.

### Implementation for User Story 1

- [ ] T008 [US1] Implement URL discovery logic in `backend/ingestion/discovery.py` to fetch and parse the sitemap.xml.
- [ ] T009 [US1] Implement HTML content extraction in `backend/ingestion/extraction.py` using `BeautifulSoup`.
- [ ] T010 [US1] Implement semantic chunking strategy in `backend/ingestion/chunking.py` based on HTML headings.
- [ ] T011 [US1] Implement embedding generation in `backend/embeddings/generator.py` to batch calls to the Cohere API.
- [ ] T012 [US1] Implement vector storage logic in `backend/vector_store/qdrant_manager.py` to connect to Qdrant and handle idempotent insertions.
- [ ] T013 [US1] Implement the main pipeline orchestration script `backend/scripts/run_pipeline.py` to connect and execute all modules in sequence.
- [ ] T014 [US1] Add logging throughout all modules to write progress and errors to files in the `backend/logs/` directory.

**Checkpoint**: At this point, the entire ingestion pipeline should be functional and capable of populating the vector database from the source website.

---

## Phase 4: User Story 2 - Data Validation (Priority: P2)

**Goal**: Create a script to validate the integrity and correctness of the data stored in the vector database.

**Independent Test**: Run the validation script and confirm that its output shows correct metadata and text for a sample of ingested documents.

### Implementation for User Story 2

- [ ] T015 [US2] Create a new validation script at `backend/scripts/validate_data.py`.
- [ ] T016 [US2] In `validate_data.py`, implement logic to connect to Qdrant and fetch a random sample of vector records.
- [ ] T017 [US2] In `validate_data.py`, implement checks to verify that the sampled records contain valid, non-empty metadata fields (`source_url`, `page_title`, `section`, `chunk_index`, `raw_text`).
- [ ] T018 [US2] In `validate_data.py`, implement a semantic search query to retrieve chunks for a known sentence and verify the correct content is returned.

**Checkpoint**: The validation script should be able to confirm that the pipeline from User Story 1 is producing high-quality, correct data.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize documentation and perform cleanup.

- [ ] T019 Update the `README.md` in the `backend/` directory with detailed instructions on setup and execution, based on the `quickstart.md`.
- [ ] T020 Code cleanup and refactoring across all modules for clarity and efficiency.
- [ ] T021 Final review of logging to ensure all critical operations and errors are captured.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)** is the starting point.
- **Foundational (Phase 2)** depends on Setup.
- **User Story 1 (Phase 3)** depends on Foundational.
- **User Story 2 (Phase 4)** depends on User Story 1.
- **Polish (Phase 5)** depends on all other phases being complete.

### User Story Dependencies
- **User Story 1 (P1)**: Must be completed first.
- **User Story 2 (P2)**: Directly depends on the successful completion and execution of User Story 1.

### Implementation Strategy
The strategy is sequential and incremental:
1.  Complete **Setup** and **Foundational** phases.
2.  Implement all tasks for **User Story 1** to build the core MVP pipeline.
3.  Implement all tasks for **User Story 2** to validate the output of the MVP.
4.  Complete the **Polish** phase.

This ensures a functional and validated pipeline is built before any final polish is applied.
