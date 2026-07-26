"""Multi-provider LLM client: Bedrock, Claude API, OpenAI → mission JSON conversion (ROS-free)."""
import json
import os
import re


SYSTEM_PROMPT = """You are a mission planner for a warehouse robot.

The robot can execute one of four mission types:
1. "mapping" - Drive the robot while building a map with SLAM
2. "navigation" - Navigate an existing map, following a perimeter loop
3. "collect_goals" - Collect waypoint markers for the perimeter
4. "squad_navigation" - Split the perimeter loop across TWO robots (one drives
   the front half, the other the back half)

When a user gives a command that is genuinely one of those three missions,
respond with ONLY a JSON object (inside ```json blocks) describing it. Never
include explanations outside the JSON block.

The JSON must have this exact structure:
{
  "mode": "mapping" | "navigation" | "collect_goals" | "squad_navigation",
  "map_name": "<EXPLICIT map name — REQUIRED. User MUST specify it. No defaults.>",
  "loops": <integer >= 1, ONLY for navigation and squad_navigation modes>
}

Examples:
- User: "Patrol the perimeter 3 times"
  Response: ```json
{"mode": "navigation", "map_name": "warehouse", "loops": 3}
```

- User: "Start building a map"
  Response: Please specify a map name, e.g., "start building a map called floor2"

- User: "Start building a map called floor2"
  Response: ```json
{"mode": "mapping", "map_name": "floor2"}
```

- User: "Capture the perimeter waypoints"
  Response: ```json
{"mode": "collect_goals", "map_name": "warehouse"}
```

- User: "Both robots patrol the perimeter, split it between them"
  Response: ```json
{"mode": "squad_navigation", "map_name": "warehouse", "loops": 1}
```

This drives a real robot, so a wrong mission is worse than no mission.

CRITICAL: For mapping/navigation/collect_goals missions, the user MUST explicitly
specify the map_name. If they say "start building a map" without a name, REJECT it
with a message like "Please specify a map name (e.g., 'start building a map called
floor2')". Never guess a default map name.

If the command is NOT one of the four missions above - a question, a greeting,
gibberish, anything this robot cannot do, OR a mapping/navigation command missing
the required map_name - do NOT output a JSON block. Reply with one short sentence
saying you cannot map it to a mission. Never guess or produce JSON without complete
information.

- User: "What is the capital of France?"
  Response: That is a question, not a robot mission.

- User: "Make me a sandwich"
  Response: This robot only maps, navigates, and collects waypoints.

When the command IS a mission, output the JSON block only, with no reasoning or
explanation outside it.
"""


class LlmClient:
    """Multi-provider LLM client: AWS Bedrock, Claude API, or OpenAI."""

    def __init__(self, api_client=None,
                 model=None,
                 temperature=0.3,
                 provider=None):
        """Initialize with a provider (auto-detect if not specified).

        Args:
            api_client: Provider-specific client object
            model: Model ID (auto-detected if None)
            temperature: Sampling temperature (0.0-1.0)
            provider: "bedrock", "claude", "openai" (auto-detected if None)
        """
        self.temperature = temperature
        self.provider = provider or self._detect_provider()

        if self.provider == "bedrock":
            self.client = api_client or self._create_bedrock_client()
            self.model = model or "us.amazon.nova-micro-v1:0"
        elif self.provider == "claude":
            self.client = api_client or self._create_claude_client()
            self.model = model or "claude-3-5-sonnet-20241022"
        elif self.provider == "openai":
            self.client = api_client or self._create_openai_client()
            self.model = model or "gpt-4o-mini"
        else:
            raise ValueError(f"Unknown provider: {self.provider}")

    def _detect_provider(self):
        """Auto-detect provider from environment variables."""
        if os.getenv("OPENAI_API_KEY"):
            return "openai"
        if os.getenv("ANTHROPIC_API_KEY"):
            return "claude"
        if os.getenv("AWS_BEARER_TOKEN_BEDROCK") or os.getenv("AWS_ACCESS_KEY_ID"):
            return "bedrock"
        raise RuntimeError(
            "No LLM credentials found. Set one of:\n"
            "  AWS_BEARER_TOKEN_BEDROCK + AWS_REGION (for Bedrock)\n"
            "  ANTHROPIC_API_KEY (for Claude API)\n"
            "  OPENAI_API_KEY (for OpenAI)"
        )

    def _create_bedrock_client(self):
        """Create AWS Bedrock client."""
        import boto3
        region = os.getenv("AWS_REGION", "us-east-1")
        return boto3.client("bedrock-runtime", region_name=region)

    def _create_claude_client(self):
        """Create Anthropic Claude client."""
        try:
            import anthropic
            return anthropic.Anthropic()
        except ImportError:
            raise RuntimeError("anthropic package not installed. Install with: pip install anthropic")

    def _create_openai_client(self):
        """Create OpenAI client."""
        try:
            import openai
            return openai.OpenAI()
        except ImportError:
            raise RuntimeError("openai package not installed. Install with: pip install openai")

    def generate_mission(self, nl_command: str) -> dict:
        """Convert natural language command to mission JSON.

        Args:
            nl_command: Natural language instruction (e.g., "Patrol twice")

        Returns:
            {
                "success": bool,
                "mission": dict | None,  # {mode, map_name, loops?} if success
                "reasoning": str         # LLM response or error message
            }
        """
        try:
            if self.provider == "bedrock":
                return self._call_bedrock(nl_command)
            elif self.provider == "claude":
                return self._call_claude(nl_command)
            elif self.provider == "openai":
                return self._call_openai(nl_command)
        except Exception as e:
            return {
                "success": False,
                "mission": None,
                "reasoning": f"{self.provider} API error: {str(e)}"
            }

    def _call_bedrock(self, nl_command: str) -> dict:
        """Call AWS Bedrock."""
        response = self.client.converse(
            modelId=self.model,
            system=[{"text": SYSTEM_PROMPT}],
            messages=[{"role": "user", "content": [{"text": nl_command}]}],
            inferenceConfig={"maxTokens": 256, "temperature": self.temperature},
        )
        response_text = response["output"]["message"]["content"][0]["text"]
        return self._extract_mission(response_text)

    def _call_claude(self, nl_command: str) -> dict:
        """Call Claude API (Anthropic)."""
        response = self.client.messages.create(
            model=self.model,
            max_tokens=256,
            temperature=self.temperature,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": nl_command}],
        )
        response_text = response.content[0].text
        return self._extract_mission(response_text)

    def _call_openai(self, nl_command: str) -> dict:
        """Call OpenAI API."""
        response = self.client.chat.completions.create(
            model=self.model,
            max_tokens=256,
            temperature=self.temperature,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": nl_command}],
        )
        response_text = response.choices[0].message.content
        return self._extract_mission(response_text)

    def _extract_mission(self, response_text: str) -> dict:
        """Extract and validate mission JSON from response."""
        # Extract JSON from ```json ... ``` block
        json_match = re.search(r'```json\s*(.*?)\s*```', response_text, re.DOTALL)
        if not json_match:
            return {
                "success": False,
                "mission": None,
                "reasoning": "LLM response did not include ```json code block"
            }

        json_str = json_match.group(1)
        mission = json.loads(json_str)

        # Minimal validation (full validation happens in validator)
        if not isinstance(mission, dict):
            return {
                "success": False,
                "mission": None,
                "reasoning": "Extracted JSON is not a dict"
            }

        # Validate required fields
        if "mode" not in mission or "map_name" not in mission:
            return {
                "success": False,
                "mission": None,
                "reasoning": "Mission missing required fields: 'mode' and/or 'map_name'"
            }

        return {
            "success": True,
            "mission": mission,
            "reasoning": response_text[:200]
        }
