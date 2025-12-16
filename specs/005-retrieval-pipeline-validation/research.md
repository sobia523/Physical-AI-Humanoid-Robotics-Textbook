# Research & Decisions: RAG Retrieval Pipeline

This document records key technical decisions made during the planning phase for the RAG retrieval pipeline.

## R-01: Optimal `top-k` value for retrieval

-   **Decision**: Initial `top-k` value of 5-10 chunks.
-   **Rationale**: This range offers a pragmatic balance for providing sufficient context to a downstream LLM without excessive noise or exceeding typical context window limitations. The exact optimal value will be determined through empirical testing during the evaluation phase, where relevance metrics will guide fine-tuning. This initial range is a common starting point in RAG systems.
-   **Alternatives considered**:
    -   Smaller `top-k` (e.g., 1-3): Risks missing crucial context, especially for complex queries.
    -   Larger `top-k` (e.g., 15-20+): May introduce too much irrelevant information, increasing noise and potentially exceeding LLM context window limits, thus increasing latency and cost.

## R-02: Choice of similarity threshold

-   **Decision**: Initially, no explicit similarity threshold will be applied to filter results; the system will always return the `top-k` most similar chunks.
-   **Rationale**: Applying a strict similarity threshold too early can prematurely filter out potentially useful, albeit less similar, chunks, leading to empty result sets for some queries. By consistently returning `top-k` chunks, we ensure a baseline context is always provided. A threshold can be introduced and fine-tuned in later iterations after empirical analysis of score distributions and relevance judgments, allowing for a more informed decision on what constitutes a "good enough" score.
-   **Alternatives considered**:
    -   Fixed similarity threshold: Risks returning too few or too many results, making the system brittle and less robust to varying query types.
    -   Dynamic similarity threshold: More complex to implement and typically requires a significant amount of labeled data for training or a sophisticated adaptive mechanism, which is out of scope for this initial phase.

## R-03: Query embedding batching or caching

-   **Decision**: Initially, query embeddings will be generated on-demand for each incoming natural language query. Batching will not be implemented for single queries, and caching for frequently asked questions will not be included in this iteration.
-   **Rationale**: For the validation and testing phase, query volume is expected to be low, making the overhead of implementing and maintaining a caching or batching layer for queries an unnecessary complexity. The primary focus is on validating the core retrieval logic and accuracy. If future performance benchmarks (e.g., for a high-traffic chatbot) reveal query embedding generation as a bottleneck, batching or caching can be introduced as a targeted optimization.
-   **Alternatives considered**:
    -   Query embedding caching: Improves latency for repeated identical queries but adds state management complexity and cache invalidation challenges.
    -   Query embedding batching: Useful for scenarios with high volumes of simultaneous, distinct queries but adds latency for individual requests unless efficiently managed. Not applicable for single, real-time query processing.
