# Data Model: RAG Agent API

This document defines the data structures for the request and response objects used by the FastAPI endpoint exposing the RAG agent. These models ensure clear contracts for interaction with the agent.

## Entity: AgentRequest

`AgentRequest` represents the input received by the FastAPI endpoint from a client.

### Schema

| Field Name              | Data Type      | Description                                                    | Required |
|-------------------------|----------------|----------------------------------------------------------------|----------|
| `query`                 | String         | The natural language question from the user.                   | Yes      |
| `source_url_constraint` | Optional[String] | Limits retrieval to chunks from a specific URL.              | No       |
| `section_constraint`    | Optional[String] | Limits retrieval to chunks from a specific section/heading.    | No       |

### Example `AgentRequest` Object

```json
{
  "query": "What are the core principles of ROS 2?",
  "source_url_constraint": "https://example.com/docs/module1/chapter1",
  "section_constraint": "ROS 2 Nodes, Topics, and Services"
}
```

## Entity: AgentResponse

`AgentResponse` represents the output returned by the FastAPI endpoint to the client.

### Schema

| Field Name       | Data Type      | Description                                                                                                   | Required |
|------------------|----------------|---------------------------------------------------------------------------------------------------------------|----------|
| `answer`         | String         | The agent's synthesized answer to the user's query.                                                           | Yes      |
| `citations`      | List[Citation] | A list of `Citation` objects referencing the source material used for the answer.                             | Yes      |
| `message`        | Optional[String] | An informational message, especially if the agent cannot answer the question or encounters issues.          | No       |

### Entity: Citation (nested within `AgentResponse.citations`)

A `Citation` object provides details about a specific source used in the agent's answer.

### Schema (Citation)

| Field Name           | Data Type | Description                                                                                 | Required |
|----------------------|-----------|---------------------------------------------------------------------------------------------|----------|
| `source_url`         | String    | The full URL of the original document.                                                      | Yes      |
| `section`            | String    | The specific section or heading within the document.                                        | Yes      |
| `raw_text_snippet`   | String    | A short snippet of the raw text from the cited chunk that was most relevant to the answer. | Yes      |

### Example `AgentResponse` Object

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
