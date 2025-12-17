# Tasks: RAG Frontend Integration

**Input**: Design documents from `/specs/007-rag-frontend-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: UI tests for core functionality and robustness.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare the Docusaurus frontend for integration.

- [ ] T001 Navigate to the `website/` directory and install Docusaurus dependencies (`npm install` or `yarn install`).
- [ ] T002 Create `website/src/components/` directory for custom React components.
- [ ] T003 Create `website/.env` file with `REACT_APP_RAG_AGENT_API_URL` pointing to the backend.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Set up the Docusaurus theme for custom UI components.

- [ ] T004 Swizzle the Docusaurus theme `Layout` component: `npm run swizzle @docusaurus/theme-classic Layout -- --eject`.
- [ ] T005 Create an empty React component file: `website/src/components/ChatWidget.js`.

---

## Phase 3: User Story 1 - Basic Question Answering from UI (Priority: P1) 🎯 MVP

**Goal**: Implement a functional chat UI for submitting questions and displaying basic responses.

**Independent Test**: Manually submit a question via the chat UI and verify that a response is displayed, along with any basic error handling.

### Implementation for User Story 1

- [ ] T006 [US1] Implement the basic floating button UI for `ChatWidget` in `website/src/components/ChatWidget.js`.
- [ ] T007 [US1] Implement the expandable chat panel UI for `ChatWidget` including an input field and display area.
- [ ] T008 [US1] Inject `ChatWidget` into `website/src/theme/Layout/index.js` (or `index.tsx`).
- [ ] T009 [US1] Implement HTTP POST request logic from `ChatWidget` to the backend `/agent/ask` endpoint, handling `FrontendRequest`.
- [ ] T010 [US1] Implement `FrontendResponse` parsing and display `answer` text in `ChatWidget`.
- [ ] T011 [US1] Implement loading indicators in `ChatWidget` while awaiting backend response.
- [ ] T012 [US1] Implement basic error handling and display for API communication failures in `ChatWidget`.
- [ ] T013 [US1] Display citations in `ChatWidget` with `source_url` and `section` information.

**Checkpoint**: Basic free-form question answering from the UI should be functional.

---

## Phase 4: User Story 2 - Selected Text Query (Priority: P2)

**Goal**: Enable users to ask questions about selected text, with appropriate constraints.

**Independent Test**: Select text on a page, submit a query about it, and verify that the response is correctly scoped to the selected text.

### Implementation for User Story 2

- [ ] T014 [US2] Implement text selection capture using `window.getSelection()` in `ChatWidget` or a separate utility.
- [ ] T015 [US2] Implement UI to trigger a selected text query (e.g., a tooltip with "Ask about this" button).
- [ ] T016 [US2] Pass `selected_text_constraint`, `source_url_constraint`, and `section_constraint` (from current page context) to the backend `FrontendRequest`.
- [ ] T017 [US2] Enhance `ChatWidget` to display responses for constrained queries, clearly indicating any applied constraints.
- [ ] T018 [US2] Make citations clickable links in `ChatWidget` that navigate to `source_url` and scroll to the `section`.
- [ ] T019 [US2] Implement client-side logic to dynamically extract the `source_url` and nearest `section` from the current page.

**Checkpoint**: Selected text queries should be fully functional.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize frontend integration, ensure robustness, and update documentation.

- [ ] T020 Implement draggable functionality for the floating chat button.
- [ ] T021 Implement state management for the chat widget (e.g., preserving chat history across page navigations).
- [ ] T022 Implement a mechanism to prevent multiple simultaneous queries.
- [ ] T023 Code cleanup and refactoring of frontend components for clarity and performance.
- [ ] T024 Update `website/docusaurus.config.js` with any necessary paths or configurations for the chat UI.
- [ ] T025 Manual verification that the integration does not break existing Docusaurus content or layout.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)** is the starting point.
- **Foundational (Phase 2)** depends on Setup.
- **User Story 1 (Phase 3)** depends on Foundational.
- **User Story 2 (Phase 4)** depends on User Story 1.
- **Polish (Phase 5)** depends on all other phases being complete.

### User Story Dependencies
- **User Story 1 (P1)**: Must be completed first as it establishes the core chat UI and backend communication.
- **User Story 2 (P2)**: Directly depends on User Story 1's basic question-answering capabilities.

### Implementation Strategy
The strategy is sequential and incremental:
1.  Complete **Setup** and **Foundational** phases.
2.  Implement all tasks for **User Story 1** to build the core chat UI with basic Q&A.
3.  Implement all tasks for **User Story 2** to add selected text querying and enhanced response display.
4.  Complete the **Polish** phase.

This ensures a robust and well-integrated frontend is built step-by-step.
