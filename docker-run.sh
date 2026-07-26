#!/usr/bin/env bash
# Run the Omokai container with GUI (X11) and AWS credentials passed through.
#
#   ./docker-run.sh                 # two-robot squad (default), opens Gazebo+RViz
#   ./docker-run.sh squad:=false    # single robot
#   ./docker-run.sh nlm:=false      # no LLM front-end
#   CMD=bash ./docker-run.sh        # just a shell inside the container
#
# Build first:  docker build -t omokai:latest .
# GUI needs an X server on the host; AWS_* creds drive the LLM stage.
# Supports: AWS_ACCESS_KEY_ID + AWS_SECRET_ACCESS_KEY, or AWS_BEARER_TOKEN_BEDROCK.
set -euo pipefail

IMAGE="${IMAGE:-omokai:latest}"

# Let the container talk to the host X server (best-effort; harmless if no xhost).
xhost +local:root >/dev/null 2>&1 || true

ARGS=(
  --rm -it
  --net=host
  --env DISPLAY="${DISPLAY:-:0}"
  --env QT_X11_NO_MITSHM=1
  --env AWS_REGION="${AWS_REGION:-us-east-1}"
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
)

# Pass LLM provider credentials: AWS Bedrock, Claude API, or OpenAI.
for v in AWS_REGION AWS_ACCESS_KEY_ID AWS_SECRET_ACCESS_KEY AWS_SESSION_TOKEN AWS_BEARER_TOKEN_BEDROCK ANTHROPIC_API_KEY OPENAI_API_KEY; do
  [ -n "${!v:-}" ] && ARGS+=(--env "$v")
done
[ -d "${HOME}/.aws" ] && ARGS+=(--volume "${HOME}/.aws:/root/.aws:ro")

# Opt-in NVIDIA GPU for faster YOLO / rendering: GPU=1 ./docker-run.sh
[ "${GPU:-0}" = "1" ] && ARGS+=(--gpus all)

if [ "${CMD:-}" = "bash" ]; then
  exec docker run "${ARGS[@]}" "$IMAGE" bash
fi

# Source the workspace and hand off to run.sh with whatever args were given.
exec docker run "${ARGS[@]}" "$IMAGE" \
  bash -lc "source install/setup.bash && ./run.sh $*"
