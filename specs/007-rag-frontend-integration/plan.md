# Implementation Plan: Frontend and Backend Integration for Book-Embedded RAG Chatbot

**Feature Branch**: `007-rag-frontend-integration`
**Feature Spec**: [spec.md](./spec.md)
**Status**: In Progress

## 1. Architecture Overview

### 1.1. High-Level Integration Architecture
```mermaid
graph TD
    A[User (Browser)] -- UI Interaction --> B(Docusaurus Frontend);
    B -- HTTP/REST (FrontendRequest) --> C(FastAPI Agent Backend);
    C -- Retrieval + LLM Reasoning --> D(Qdrant DB + OpenAI Agents SDK);
    D -- Retrieved Chunks + Answer --> C;
    C -- AgentResponse (HTTP/REST) --> B;
    B -- Rendered Answer + Citations --> A;
```
This architecture shows the full lifecycle of a user query:
1.  **UI Interaction**: User types a question or selects text and submits.
2.  **Docusaurus Frontend**: Captures user input, constructs `FrontendRequest`.
3.  **FastAPI Agent Backend**: Receives request, invokes retrieval and LLM reasoning, constructs `AgentResponse`.
4.  **Qdrant DB + OpenAI Agents SDK**: Provides knowledge base and reasoning capabilities.
5.  **Rendered Answer**: Frontend parses `AgentResponse` and displays it with citations.

### 1.2. Clear Request/Response Lifecycle
-   User initiates query from Docusaurus UI.
-   Frontend constructs JSON payload mirroring `AgentRequest` structure from Spec-006.
-   Frontend sends POST request to `/agent/ask` endpoint on the FastAPI backend.
-   Backend processes request and returns `AgentResponse` (JSON).
-   Frontend receives, parses, and renders `AgentResponse` data.

## 2. Frontend Integration Design

### 2.1. Chat UI Placement
-   **Decision**: Implement the chat UI as a **floating widget** (e.g., a small icon in the corner that expands into a chat panel).
-   **Rationale**: A floating widget is non-intrusive, allows users to access the chatbot from any page without disrupting their reading, and can be easily toggled. This aligns with the constraint of not redesigning the core layout.
-   **Alternative**: Sidebar panel or inline component would require more significant layout changes.

### 2.2. User Interaction Modes
-   **Free-form Question Input**: A standard text input field within the chat panel for users to type questions.
-   **Selected-Text Question Flow**:
    1.  User highlights text on any page.
    2.  A small, ephemeral UI element (e.g., a tooltip with a "Ask about this" button) appears near the selection.
    3.  Clicking the button opens the chat panel (if closed) and pre-populates the query with the selected text as a `selected_text_constraint`. The user can then type their question.

### 2.3. Request Construction
The frontend will construct a JSON object matching the `AgentRequest` schema from Spec-006.
-   `query`: User's typed question.
-   `source_url_constraint`: Dynamically captured from the current Docusaurus page URL.
-   `section_constraint`: Dynamically captured from the nearest heading above the selected text/current scroll position (if selected text isn't used).
-   `selected_text_constraint`: Captured from user's text selection.

## 3. Backend Connection & API Contract

-   **API Endpoint**: `POST /agent/ask` from the FastAPI Agent Backend (Spec-006).
-   **Request/Response Schemas**: Strictly adhere to `AgentRequest` and `AgentResponse` Pydantic models defined in Spec-006.
-   **Error Response Handling**: Frontend will parse backend error responses (e.g., 4xx, 5xx status codes, or `AgentResponse` with `message` field) and display user-friendly messages.
-   **Timeout/Retry Behavior**: Frontend will implement a reasonable timeout for API requests (e.g., 10-15 seconds). No automatic retries will be implemented in the frontend; user will be prompted to try again.

## 4. Selected-Text-Only Constraint Flow

-   **Capture Selected Text**: JavaScript DOM APIs (`window.getSelection()`) will be used to accurately capture the user's highlighted text.
-   **Transmit to Backend**: The captured selected text will be sent as the `selected_text_constraint` field in the `AgentRequest`.
-   **Agent Enforcement**: The backend agent (from Spec-006) is responsible for interpreting this constraint and limiting its knowledge source accordingly.
-   **Failure Communication**: If the agent cannot find relevant information within the selected text, it will communicate this via the `AgentResponse.message` field, which the frontend will display.

## 5. State, Loading & Error Handling (Frontend)

-   **Loading Indicators**: Display a clear visual loading state (e.g., spinner, "Agent is thinking...") after query submission until a response is received.
-   **Partial/Empty Responses**:
    -   If `AgentResponse.answer` is empty but `AgentResponse.message` is present, display the message.
    -   If both are empty, display a generic "No information found" message.
-   **Backend Errors**: Display a user-friendly error message if API call fails (e.g., network error, backend server error).
-   **User Feedback**: Provide immediate feedback for actions (e.g., "Query submitted!").

## 6. Security & Deployment Considerations

-   **CORS Configuration**: The FastAPI backend (Spec-006) MUST be configured to allow Cross-Origin Resource Sharing (CORS) for the deployed Docusaurus frontend domain.
-   **Environment-Specific API URLs**: The frontend will use environment variables (e.g., `.env` file for Docusaurus build, or dynamic injection) to configure the backend API URL, allowing for different environments (dev, staging, prod).
-   **Protection against Malformed Requests**: The FastAPI backend's Pydantic validation (from Spec-006) handles this. Frontend will ensure valid request construction.
-   **Safe Exposure**: Frontend will only send necessary user input (query, constraints) to the backend. No sensitive user data.

## 7. Decisions Needing Documentation

All research and decisions will be documented in [research.md](./research.md). Key areas to investigate and justify:
-   **R-01**: Specific Docusaurus integration points for custom UI components (e.g., theme swizzling, plugin architecture).
-   **R-02**: Cross-browser compatibility for selected text capture.
-   **R-03**: Detailed design of the floating chat widget (UI/UX, expandable/collapsible behavior).
-   **R-04**: Implementation approach for making citations clickable and scrolling to sections within Docusaurus.

## 8. Testing & Validation Strategy

Validation checks will be aligned with the acceptance criteria from `spec.md`:
-   **End-to-end Flow**: Automated browser tests (e.g., Playwright) for submitting questions and validating responses (SC-001).
-   **Grounded Answers/Citations**: Manual verification and integration tests with backend response structure (SC-002).
-   **Selected Text Scope**: Dedicated UI tests for selected text queries to ensure correct constraint passing and agent adherence (SC-003).
-   **UI Robustness**: UI tests for loading states, error displays, and empty responses (SC-004).
-   **No Regression**: Automated visual regression tests for existing Docusaurus pages.
-   **Performance**: Frontend performance metrics (e.g., Lighthouse scores) will be monitored to ensure no degradation (SC-005).

## 9. Quality & Governance

-   All architectural, formatting, and quality rules from `sp.constitution` will be followed.
-   The plan is organized by logical phases:
    -   UI Design (component structure, interaction)
    -   API Integration (connecting frontend to backend)
    -   Constraint Enforcement (handling selected text, context)
    -   Validation (testing strategies)

### Exclusions:
-   Streaming responses (current implementation is request/response).
-   Authentication flows.
-   Multi-turn conversational memory.