# Tasks: RAG Agent Construction with FastAPI

**Input**: Design documents from `/specs/006-rag-agent-construction/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md

**Tests**: A dedicated `backend/scripts/test_agent_api.py` script is included to meet validation acceptance criteria.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create necessary directories and add new core dependencies.

- [ ] T001 Create the `backend/agent/` directory.
- [ ] T002 Create an empty Python file: `backend/agent/tools.py`.
- [ ] T003 Create an empty Python file: `backend/agent/agent_orchestrator.py`.
- [ ] T004 Update `backend/requirements.txt` to include `fastapi`, `uvicorn`, `openai`, `pydantic-settings`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Initialize the FastAPI application and ensure environment variables are configured.

- [ ] T005 Create the main FastAPI application file: `backend/main.py`.
- [ ] T006 Add a basic FastAPI endpoint (e.g., `/health`) in `backend/main.py` for testing.
- [ ] T007 Ensure `backend/.env` is properly configured with `OPENAI_API_KEY`. (Manual check for user)

---

## Phase 3: User Story 1 - Question Answering with Retrieval (Priority: P1) 🎯 MVP

**Goal**: Implement an agent that answers user questions using retrieved book content and exposes this via a FastAPI endpoint.

**Independent Test**: Send a basic question to the `/agent/ask` endpoint and verify an accurate, grounded answer with citations.

### Implementation for User Story 1

- [ ] T008 [US1] Implement `RetrieverTool` in `backend/agent/tools.py` that encapsulates the `Retriever.search` logic from Spec-005.
- [ ] T009 [US1] Implement the agent's core logic in `backend/agent/agent_orchestrator.py` using `openai.lib.Assistant` and integrating `RetrieverTool`.
- [ ] T010 [US1] Implement the `/agent/ask` FastAPI endpoint in `backend/main.py` to accept `AgentRequest` and return `AgentResponse`.
- [ ] T011 [US1] Integrate `backend/agent/agent_orchestrator.py` into the `/agent/ask` endpoint handler in `backend/main.py`.
- [ ] T012 [US1] Create the test script `backend/scripts/test_agent_api.py` and implement initial test cases for basic question answering.
- [ ] T013 [US1] Implement citation parsing from LLM response in `backend/agent/agent_orchestrator.py`.
- [ ] T014 [US1] Integrate logging into `backend/agent/tools.py`, `backend/agent/agent_orchestrator.py`, `backend/main.py`, and `backend/scripts/test_agent_api.py`.

**Checkpoint**: At this point, basic question answering through the FastAPI endpoint should be functional and testable.

---

## Phase 4: User Story 2 - Constrained Answering (Priority: P2)

**Goal**: Enhance the agent to handle query constraints (source URL, section) and enforce grounded answering within these constraints.

**Independent Test**: Send questions with `source_url` and `section` constraints and verify that answers adhere to the constraints and citations reflect the constrained context.

### Implementation for User Story 2

- [ ] T015 [US2] Enhance `RetrieverTool` in `backend/agent/tools.py` to accept and pass `source_url_constraint` and `section_constraint` to the underlying `Retriever.search` method.
- [ ] T016 [US2] Enhance agent prompting in `backend/agent/agent_orchestrator.py` to instruct the LLM to strictly adhere to provided constraints and cite sources from the constrained context.
- [ ] T017 [US2] Expand `backend/scripts/test_agent_api.py` to include test cases for constrained queries (by `source_url` and `section`).
- [ ] T018 [US2] Expand `backend/scripts/test_agent_api.py` to include checks for agent's refusal to answer when outside constrained knowledge.
- [ ] T019 [US2] Add latency measurement and reporting to `backend/scripts/test_agent_api.py` for FastAPI endpoint responses.

**Checkpoint**: The agent should now handle constrained queries correctly and validate adherence.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize documentation and perform cleanup.

- [ ] T020 Update `backend/README.md` with instructions on running the FastAPI server and agent tests.
- [ ] T021 Code cleanup and refactoring across all new modules for clarity and efficiency.
- [ ] T022 Final review of logging to ensure all critical operations and errors are captured.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)** is the starting point.
- **Foundational (Phase 2)** depends on Setup.
- **User Story 1 (Phase 3)** depends on Foundational.
- **User Story 2 (Phase 4)** depends on User Story 1.
- **Polish (Phase 5)** depends on all other phases being complete.

### User Story Dependencies
- **User Story 1 (P1)**: Must be completed first as it establishes the core agent functionality.
- **User Story 2 (P2)**: Directly depends on User Story 1's basic question-answering capabilities.

### Implementation Strategy
The strategy is sequential and incremental:
1.  Complete **Setup** and **Foundational** phases.
2.  Implement all tasks for **User Story 1** to build the core RAG agent MVP.
3.  Implement all tasks for **User Story 2** to add constraint handling and expanded validation.
4.  Complete the **Polish** phase.

This ensures a robust and validated agent is built step-by-step.
