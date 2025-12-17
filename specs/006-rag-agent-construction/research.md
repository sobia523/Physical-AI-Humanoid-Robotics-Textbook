# Research & Decisions: RAG Agent Construction

This document records key technical decisions made during the planning phase for the RAG Agent Construction.

## R-01: Specific OpenAI Agents SDK components to use

-   **Decision**: Utilize `openai.lib.Assistant` (or equivalent high-level agent orchestration within the SDK) in conjunction with custom tool definitions. The retrieval logic (from Spec-005) will be encapsulated as a custom tool that the agent can invoke.
-   **Rationale**: The OpenAI Agents SDK is designed to streamline the creation of agents capable of using tools. Leveraging its built-in constructs (like `Assistant` and `Tool` definitions) aligns with the framework constraint and simplifies the agent's reasoning loop by abstracting away complex LLM interaction patterns.
-   **Alternatives considered**:
    -   Manual LLM prompting: While flexible, this approach requires significant boilerplate for tool invocation, response parsing, and state management, which the SDK aims to automate.
    -   Other agent frameworks (e.g., LangChain, LlamaIndex): Explicitly excluded by the "OpenAI Agents SDK" constraint.

## R-02: Exact LLM prompting strategy for grounding and citation generation

-   **Decision**: The agent's system message will be carefully crafted to include explicit instructions for the LLM to:
    1.  Answer questions *solely* based on the provided retrieved context.
    2.  Refuse to answer if the context does not contain sufficient information.
    3.  Include citations for *every* piece of information directly extracted from the context, using a predefined structured format (e.g., `[Source: {URL}, Section: {Heading}]`).
The retrieved chunks will be clearly delineated within the user message as part of the context.
-   **Rationale**: Strong system-level instructions are paramount for achieving grounded answers and accurate citation generation in RAG systems, mitigating hallucination and improving trustworthiness. Clear formatting of context helps the LLM differentiate between query and source material.
-   **Alternatives considered**:
    -   Less explicit prompting: Risks increased hallucination and less consistent citation formatting.
    -   Post-processing to add citations: More complex and less reliable than having the LLM generate them directly as part of its response.

## R-03: Context construction and truncation strategy

-   **Decision**:
    1.  Retrieved `RetrievedChunk` objects will be ordered by their semantic relevance score (highest first) as provided by Qdrant.
    2.  Context will be constructed by concatenating the `raw_text` of these ordered chunks, along with their metadata (URL, section) formatted for display (e.g., `"[Document N (Source: URL, Section: Heading)]\nText content\n"`).
    3.  If the combined context (retrieved chunks + user query + system prompt) exceeds the LLM's maximum context window, truncation will occur by removing the least relevant chunks (those with the lowest similarity scores) until the context fits.
-   **Rationale**: Prioritizing higher-scoring chunks ensures that the most semantically relevant information is always presented to the LLM. Ordering helps maintain a logical flow for the LLM. Truncation is a necessary evil for managing LLM context window limitations, and removing the least relevant data first is the most logical approach.
-   **Alternatives considered**:
    -   Truncating individual chunks: Risks losing critical information from within a chunk.
    -   Random truncation: Reduces determinism and may remove highly relevant information.

## R-04: Citation parsing and formatting from the LLM's response

-   **Decision**: A post-processing step will be implemented to parse the LLM's raw answer string using regular expressions (or a similar pattern-matching technique) to extract citations that adhere to the predefined format specified in R-02. These extracted citations will then be structured into the `AgentResponse` format.
-   **Rationale**: While prompting the LLM to generate citations in a specific format is effective, a robust parsing layer is necessary to handle slight variations or errors in the LLM's output and to ensure the `AgentResponse` consistently provides structured citations.
-   **Alternatives considered**:
    -   Direct LLM structured output (e.g., JSON response): While ideal, this can sometimes reduce LLM flexibility in natural language generation and may still require robust parsing for edge cases. It also might be more complex to integrate with the OpenAI Agents SDK's standard output.
    -   No parsing: Risks presenting unformatted or inconsistent citations to the end-user.
