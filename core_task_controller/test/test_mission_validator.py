from core_task_controller.mission_validator import validate_and_explain


class TestMissionValidator:
    def test_valid_navigation_mission(self):
        """Valid navigation mission passes."""
        mission = {"mode": "navigation", "map_name": "warehouse", "loops": 2}
        result = validate_and_explain(mission)
        assert result["valid"] is True
        assert result["mission"] == mission

    def test_valid_mapping_mission_no_loops(self):
        """Mapping mode ignores loops parameter."""
        mission = {"mode": "mapping", "map_name": "warehouse"}
        result = validate_and_explain(mission)
        assert result["valid"] is True
        assert result["mission"] == mission

    def test_invalid_missing_mode(self):
        """Missing mode rejects."""
        mission = {"map_name": "warehouse"}
        result = validate_and_explain(mission)
        assert result["valid"] is False
        assert "mode" in result["reason"].lower()

    def test_invalid_missing_map_name(self):
        """Missing map_name rejects."""
        mission = {"mode": "navigation"}
        result = validate_and_explain(mission)
        assert result["valid"] is False
        assert "map_name" in result["reason"].lower()

    def test_invalid_loops_zero(self):
        """loops=0 rejects."""
        mission = {"mode": "navigation", "map_name": "warehouse", "loops": 0}
        result = validate_and_explain(mission)
        assert result["valid"] is False
        assert "loops" in result["reason"].lower()

    def test_invalid_loops_negative(self):
        """Negative loops reject."""
        mission = {"mode": "navigation", "map_name": "warehouse", "loops": -1}
        result = validate_and_explain(mission)
        assert result["valid"] is False
        assert "loops" in result["reason"].lower()

    def test_invalid_mode(self):
        """Unknown mode rejects."""
        mission = {"mode": "inspect", "map_name": "warehouse"}
        result = validate_and_explain(mission)
        assert result["valid"] is False
        assert "mapping" in result["reason"].lower()

    def test_not_dict(self):
        """Non-dict input rejects."""
        result = validate_and_explain("not a dict")
        assert result["valid"] is False

    def test_valid_find_person_mission_no_loops(self):
        """find_person needs only mode and map_name, like mapping."""
        mission = {"mode": "find_person", "map_name": "warehouse"}
        result = validate_and_explain(mission)
        assert result["valid"] is True
        assert result["mission"] == mission

    def test_mission_field_preserved_on_valid(self):
        """Valid mission is returned unchanged."""
        mission = {"mode": "collect_goals", "map_name": "office"}
        result = validate_and_explain(mission)
        assert result["mission"] is mission  # Same object
