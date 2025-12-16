# Data Model: Retrieved Chunk

This document defines the data structure for a `RetrievedChunk` entity, which represents a single chunk of text returned by the semantic retrieval system. This entity combines the original chunk metadata with the retrieval score.

## Entity: RetrievedChunk

A `RetrievedChunk` is the output format from the retrieval pipeline, providing the necessary information for downstream processing (e.g., by an LLM for response generation).

### Schema

| Field Name     | Data Type             | Description                                                                                             | Required |
|----------------|-----------------------|---------------------------------------------------------------------------------------------------------|----------|
| `text`         | String                | The raw text content of the retrieved chunk.                                                            | Yes      |
| `metadata`     | Object (JSON)         | A JSON object containing all associated metadata from the original ingested `VectorRecord` payload.     | Yes      |
| `score`        | Float                 | The similarity score between the query embedding and the chunk's embedding. Higher is more similar.     | Yes      |

### Metadata Structure (within `metadata` field)

The `metadata` field of a `RetrievedChunk` directly corresponds to the `payload` structure of the `VectorRecord` stored in Qdrant.

| Metadata Field | Data Type      | Description                                                                                             | Example                                                  |
|----------------|----------------|---------------------------------------------------------------------------------------------------------|----------------------------------------------------------|
| `source_url`   | String         | The full URL of the page from which the text was extracted.                                             | `https://[...]/module1-ros2-humanoid-control/chapter1`   |
| `page_title`   | String         | The title of the source HTML page.                                                                      | `Chapter 1: Introduction to ROS 2`                       |
| `section`      | String         | The title of the immediate parent section or heading (e.g., from an `<h2>` or `<h3>` tag).               | `Understanding ROS 2 Nodes`                              |
| `chunk_index`  | Integer        | The 0-based sequential position of this chunk within its source page.                                   | `3`                                                      |
| `raw_text`     | String         | The original, unmodified text content of the chunk. (Note: This is redundant with top-level `text` but kept for full payload traceability) | "A ROS 2 node is a fundamental component..."           |

### Example `RetrievedChunk` Object

```json
{
  "text": "ROS 2 nodes are fundamental components. They are executables that communicate via topics.",
  "metadata": {
    "source_url": "https://physical-ai-humanoid-robotics-textb-beta-two.vercel.app/docs/module1/chapter1",
    "page_title": "Chapter 1: ROS 2 Basics",
    "section": "What are ROS 2 Nodes?",
    "chunk_index": 5,
    "raw_text": "ROS 2 nodes are fundamental components. They are executables that communicate via topics."
  },
  "score": 0.875
}
```
