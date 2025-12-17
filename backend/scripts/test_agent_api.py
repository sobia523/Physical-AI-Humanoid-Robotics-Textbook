import os
import sys
import time # For latency measurement
from fastapi.testclient import TestClient
import json

# Get the path to the project root, assuming it's the directory containing 'backend'
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from backend.main import app
from backend.agent.models import AgentRequest, AgentResponse
from backend.config.logging_config import setup_logging

logger = setup_logging(log_file_name="agent_api_test.log")

client = TestClient(app)

def test_ask_agent_basic_question():
    logger.info("--- Running Test: Basic Question Answering ---")
    
    test_query = "What is a URDF model?"
    request_payload = AgentRequest(query=test_query).dict()

    start_time = time.perf_counter()
    response = client.post("/agent/ask", json=request_payload)
    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000
    
    assert response.status_code == 200
    response_data = AgentResponse.parse_obj(response.json())
    
    logger.info(f"Response (Latency: {latency_ms:.2f}ms): {response_data.dict()}")
    assert response_data.answer is not None and len(response_data.answer) > 0
    assert len(response_data.citations) > 0 # Expect at least one citation
    
    logger.info("Basic Question Answering Test PASSED.")

def test_ask_agent_unanswerable_question():
    logger.info("--- Running Test: Unanswerable Question ---")
    
    test_query = "What is the capital of Mars?"
    request_payload = AgentRequest(query=test_query).dict()

    start_time = time.perf_counter()
    response = client.post("/agent/ask", json=request_payload)
    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000
    
    assert response.status_code == 200
    response_data = AgentResponse.parse_obj(response.json())
    
    logger.info(f"Response (Latency: {latency_ms:.2f}ms): {response_data.dict()}")
    # Expecting the agent to indicate it cannot answer
    assert "cannot answer" in response_data.answer.lower() or "not found" in response_data.answer.lower()
    assert response_data.citations == []
    
    logger.info("Unanswerable Question Test PASSED.")

def test_ask_agent_constrained_by_source_url():
    logger.info("--- Running Test: Constrained by Source URL ---")
    
    # Placeholder values for an actual URL and a question that should be answerable from it
    constrained_query = "What is a ROS 2 node?"
    constrained_source_url = "http://your-docusaurus-site.example.com/docs/module1-ros2-humanoid-control/chapter1" # Replace with actual URL from your Qdrant
    
    request_payload = AgentRequest(query=constrained_query, source_url_constraint=constrained_source_url).dict()

    start_time = time.perf_counter()
    response = client.post("/agent/ask", json=request_payload)
    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000
    
    assert response.status_code == 200
    response_data = AgentResponse.parse_obj(response.json())
    
    logger.info(f"Response (Latency: {latency_ms:.2f}ms): {response_data.dict()}")
    assert response_data.answer is not None and len(response_data.answer) > 0
    assert len(response_data.citations) > 0
    # Verify all citations come from the constrained URL
    assert all(c.source_url == constrained_source_url for c in response_data.citations)
    
    logger.info("Constrained by Source URL Test PASSED.")

def test_ask_agent_constrained_by_section():
    logger.info("--- Running Test: Constrained by Section ---")
    
    constrained_query = "What is physics simulation?"
    constrained_section = "Simulate physics, gravity, and collisions in Gazebo" # Replace with actual section from your Qdrant
    
    request_payload = AgentRequest(query=constrained_query, section_constraint=constrained_section).dict()

    start_time = time.perf_counter()
    response = client.post("/agent/ask", json=request_payload)
    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000
    
    assert response.status_code == 200
    response_data = AgentResponse.parse_obj(response.json())
    
    logger.info(f"Response (Latency: {latency_ms:.2f}ms): {response_data.dict()}")
    assert response_data.answer is not None and len(response_data.answer) > 0
    assert len(response_data.citations) > 0
    # Verify all citations come from the constrained section
    assert all(c.section == constrained_section for c in response_data.citations)
    
    logger.info("Constrained by Section Test PASSED.")

def test_ask_agent_refuse_to_answer_due_to_constraint():
    logger.info("--- Running Test: Refuse to Answer Due to Constraint ---")
    
    # Query for something not in the constrained section
    query = "What is a URDF model?"
    # A section that should NOT contain information about URDF models
    non_relevant_section = "NVIDIA Isaac Sim: photorealistic simulation and synthetic data generation" 
    
    request_payload = AgentRequest(query=query, section_constraint=non_relevant_section).dict()

    start_time = time.perf_counter()
    response = client.post("/agent/ask", json=request_payload)
    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000
    
    assert response.status_code == 200
    response_data = AgentResponse.parse_obj(response.json())
    
    logger.info(f"Response (Latency: {latency_ms:.2f}ms): {response_data.dict()}")
    # Agent should indicate it cannot answer within the given constraint
    assert "cannot answer" in response_data.answer.lower() or "not found" in response_data.answer.lower()
    
    logger.info("Refuse to Answer Due to Constraint Test PASSED.")


if __name__ == "__main__":
    logger.info("--- Starting Agent API Tests ---")
    try:
        test_ask_agent_basic_question()
        test_ask_agent_unanswerable_question()
        test_ask_agent_constrained_by_source_url()
        test_ask_agent_constrained_by_section()
        test_ask_agent_refuse_to_answer_due_to_constraint()
    except Exception as e:
        logger.error(f"An error occurred during API tests: {e}")
    logger.info("--- Agent API Tests Finished ---")