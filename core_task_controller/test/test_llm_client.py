import json
from unittest.mock import Mock

import pytest

from core_task_controller.llm_client import LlmClient


def converse_reply(text):
    """Shape a Bedrock Converse response carrying `text` as the reply body."""
    return {'output': {'message': {'content': [{'text': text}]}}}


class TestLlmClient:
    @pytest.fixture
    def mock_client(self):
        """Mock boto3 bedrock-runtime client."""
        return Mock()

    @pytest.fixture
    def llm(self, mock_client):
        """LlmClient instance with a mocked Bedrock client."""
        return LlmClient(api_client=mock_client)

    def test_generate_mission_valid_navigation(self, llm, mock_client):
        """Valid navigation JSON from the model is extracted and returned."""
        expected = {"mode": "navigation", "map_name": "warehouse", "loops": 3}
        mock_client.converse.return_value = converse_reply(
            '```json\n%s\n```' % json.dumps(expected))

        result = llm.generate_mission("Patrol the perimeter 3 times")
        assert result["success"] is True
        assert result["mission"] == expected

    def test_generate_mission_mapping(self, llm, mock_client):
        """Mapping mode mission is extracted."""
        expected = {"mode": "mapping", "map_name": "warehouse"}
        mock_client.converse.return_value = converse_reply(
            '```json\n%s\n```' % json.dumps(expected))

        result = llm.generate_mission("Start mapping the warehouse")
        assert result["success"] is True
        assert result["mission"] == expected

    def test_generate_mission_schema_validation_failure(self, llm, mock_client):
        """Valid JSON but missing required fields is rejected."""
        mock_client.converse.return_value = converse_reply(
            '```json\n{"mode": "navigation"}\n```')  # missing map_name

        result = llm.generate_mission("Do something")
        assert result["success"] is False
        assert result["mission"] is None

    def test_generate_mission_invalid_json_syntax(self, llm, mock_client):
        """Syntactically invalid JSON in the response is rejected."""
        mock_client.converse.return_value = converse_reply(
            '```json\n{invalid}\n```')

        result = llm.generate_mission("Do something")
        assert result["success"] is False
        assert result["mission"] is None
        assert "json" in result["reasoning"].lower()

    def test_generate_mission_no_json_code_block(self, llm, mock_client):
        """A response without a ```json block is rejected."""
        mock_client.converse.return_value = converse_reply('just plain text')

        result = llm.generate_mission("Do something")
        assert result["success"] is False

    def test_generate_mission_api_error(self, llm, mock_client):
        """A Bedrock call error is caught and returned as failure."""
        mock_client.converse.side_effect = Exception("API timeout")

        result = llm.generate_mission("Do something")
        assert result["success"] is False
        reasoning = result["reasoning"].lower()
        assert "error" in reasoning or "timeout" in reasoning

    def test_generate_mission_includes_reasoning(self, llm, mock_client):
        """A success response carries the model's reasoning text."""
        expected = {"mode": "navigation", "map_name": "warehouse", "loops": 2}
        mock_client.converse.return_value = converse_reply(
            'The user wants: navigate twice.\n```json\n%s\n```'
            % json.dumps(expected))

        result = llm.generate_mission("Navigate twice")
        assert result["success"] is True
        assert result["reasoning"] is not None
        assert len(result["reasoning"]) > 0
