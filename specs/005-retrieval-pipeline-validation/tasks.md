# Tasks: RAG Retrieval Pipeline Validation and Semantic Query Testing

**Input**: Design documents from `/specs/005-retrieval-pipeline-validation/`
**Prerequisites**: plan.md, spec.md, research.md, retrieved-chunk-model.md

**Tests**: A dedicated `backend/scripts/test_retrieval.py` script is included to meet validation acceptance criteria.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Minimal setup as the `backend/` project and core dependencies are already established from Spec-1.

- [x] T001 Create the `backend/retrieval/` directory.
- [x] T002 Create an empty Python file: `backend/retrieval/retriever.py`.
- [x] T003 Update `backend/requirements.txt` if any new packages are required (e.g., for specialized query preprocessing if identified later).

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: No new foundational tasks beyond what's already configured for the `backend/` project.

- [x] T004 Ensure `backend/.env` is properly configured with `COHERE_API_KEY` and `QDRANT_URL`. (Manual check for user)

---

## Phase 3: User Story 1 - Semantic Search and Retrieval (Priority: P1) 🎯 MVP

**Goal**: Implement the core semantic search functionality to retrieve relevant chunks from Qdrant based on a natural language query.

**Independent Test**: Run `backend/scripts/test_retrieval.py` with basic semantic queries and verify that it returns relevant chunks with correct metadata.

### Implementation for User Story 1

- [x] T005 [US1] Implement `query_embeddings` generation logic in `backend/retrieval/retriever.py` using `backend/embeddings/generator.py`.
- [x] T006 [US1] Implement core semantic search logic in `backend/retrieval/retriever.py` to query Qdrant for `top-k` similar vectors.
- [x] T007 [US1] Implement result formatting in `backend/retrieval/retriever.py` to return `RetrievedChunk` objects, including `text`, `metadata`, and `score`.
- [x] T008 [US1] Create the test script `backend/scripts/test_retrieval.py` and implement initial semantic query test cases.
- [x] T009 [US1] Integrate logging into `backend/retrieval/retriever.py` and `backend/scripts/test_retrieval.py`.

**Checkpoint**: At this point, basic semantic search should be functional and testable.

---

## Phase 4: User Story 2 - Filtered Retrieval and Metadata Validation (Priority: P2)

**Goal**: Enhance retrieval with metadata filtering and validate the integrity and traceability of retrieved information.

**Independent Test**: Run `backend/scripts/test_retrieval.py` with filtered queries and verify that results adhere to filter criteria and metadata is accurate.

### Implementation for User Story 2

- [x] T010 [US2] Enhance semantic search logic in `backend/retrieval/retriever.py` to accept and apply `source_url` filters.
- [x] T011 [US2] Enhance semantic search logic in `backend/retrieval/retriever.py` to accept and apply `section` filters.
- [x] T012 [US2] Expand `backend/scripts/test_retrieval.py` to include test cases for filtered queries (by `source_url` and `section`).
- [x] T013 [US2] Expand `backend/scripts/test_retrieval.py` to include checks for metadata integrity and traceability in retrieved chunks.
- [x] T014 [US2] Add latency measurement and reporting to `backend/scripts/test_retrieval.py` for both unfiltered and filtered queries.

**Checkpoint**: All core retrieval logic, including filtering and validation, should be functional.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize documentation and perform cleanup.

- [x] T015 Update `backend/README.md` with instructions on running retrieval tests and configuring any new settings.
- [x] T016 Code cleanup and refactoring across all new modules for clarity and efficiency.
- [x] T017 Final review of logging to ensure all critical operations and errors are captured in the retrieval modules.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)** is the starting point.
- **Foundational (Phase 2)** depends on Setup.
- **User Story 1 (Phase 3)** depends on Foundational.
- **User Story 2 (Phase 4)** depends on User Story 1.
- **Polish (Phase 5)** depends on all other phases being complete.

### User Story Dependencies
- **User Story 1 (P1)**: Must be completed first as it establishes the core retrieval mechanism.
- **User Story 2 (P2)**: Directly depends on User Story 1's retrieval functionality.

### Implementation Strategy
The strategy is sequential and incremental:
1.  Complete **Setup** and **Foundational** phases.
2.  Implement all tasks for **User Story 1** to build the core semantic search MVP.
3.  Implement all tasks for **User Story 2** to add filtering and comprehensive validation.
4.  Complete the **Polish** phase.

This ensures a robust and validated retrieval system is built step-by-step.
