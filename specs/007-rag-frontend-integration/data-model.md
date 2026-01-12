# Data Model: Frontend-Backend Communication

This document defines the data structures for requests sent from the Docusaurus frontend to the FastAPI agent backend, and responses received by the frontend. These models ensure a clear and consistent contract for the UI interaction with the RAG agent.

## Entity: FrontendRequest

`FrontendRequest` represents the input sent from the Docusaurus frontend to the FastAPI backend. It mirrors the `AgentRequest` model from Spec-006, with the addition of `selected_text_constraint`.

### Schema

| Field Name                  | Data Type        | Description                                                                   | Required |
|-----------------------------|------------------|-------------------------------------------------------------------------------|----------|
| `query`                     | String           | The natural language question from the user.                                  | Yes      |
| `source_url_constraint`     | Optional[String] | Limits retrieval to chunks from a specific URL (e.g., current page URL).      | No       |
| `section_constraint`        | Optional[String] | Limits retrieval to chunks from a specific section/heading.                   | No       |
| `selected_text_constraint`  | Optional[String] | The text selected by the user, used for very narrow contextual answers.       | No       |

### Example `FrontendRequest` Object

```json
{
  "query": "What are the key principles?",
  "source_url_constraint": "https://example.com/docs/module1/chapter1",
  "section_constraint": null,
  "selected_text_constraint": "ROS 2 is a flexible framework for robot development."
}
```

## Entity: FrontendResponse

`FrontendResponse` represents the output received by the Docusaurus frontend from the FastAPI backend. It directly mirrors the `AgentResponse` model from Spec-006.

### Schema

| Field Name       | Data Type      | Description                                                                                                   | Required |
|------------------|----------------|---------------------------------------------------------------------------------------------------------------|----------|
| `answer`         | String         | The agent's synthesized answer to the user's query.                                                           | Yes      |
| `citations`      | List[Citation] | A list of `Citation` objects referencing the source material used for the answer.                             | Yes      |
| `message`        | Optional[String] | An informational message (e.g., if the agent cannot answer the question or encounters issues).          | No       |

### Entity: Citation (nested within `FrontendResponse.citations`)

A `Citation` object provides details about a specific source used in the agent's answer.

### Schema (Citation)

| Field Name           | Data Type | Description                                                                                 | Required |
|----------------------|-----------|---------------------------------------------------------------------------------------------|----------|
| `source_url`         | String    | The full URL of the original document.                                                      | Yes      |
| `section`            | String    | The specific section or heading within the document.                                        | Yes      |
| `raw_text_snippet`   | String    | A short snippet of the raw text from the cited chunk that was most relevant to the answer. | Yes      |

### Example `FrontendResponse` Object

```json
{
  "answer": "ROS 2 nodes are fundamental components in the ROS 2 ecosystem that communicate via topics. [Source: example.com/chapter1, Section: ROS 2 Nodes]",
  "citations": [
    {
      "source_url": "https://example.com/docs/module1/chapter1",
      "section": "ROS 2 Nodes, Topics, and Services",
      "raw_text_snippet": "ROS 2 nodes are fundamental components..."
    }
  ],
  "message": null
}
```
