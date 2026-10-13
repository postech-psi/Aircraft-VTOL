#!/usr/bin/env bash
# Run all simulation tools in the same Linux environment, with persistent host results.
set -euo pipefail
PSI_WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PSI_IMAGE_NAME="${PSI_SITL_IMAGE:-psi-vtol-sitl:v1.17.0-harmonic}"
PSI_IMAGE_ID="$(docker image inspect --format '{{.Id}}' "$PSI_IMAGE_NAME")"
if [[ $# -eq 0 ]]; then
  set -- python3 -m psi_vtol.sim.run_case
fi
PSI_DOCKER_ARGS=(--rm --init --shm-size=1g
  --mount "type=bind,source=${PSI_WORKSPACE_DIR},target=/workspace"
  --env PYTHONPATH=/workspace
  --env "PSI_RUNTIME_IMAGE=${PSI_IMAGE_NAME}"
  --env "PSI_RUNTIME_IMAGE_ID=${PSI_IMAGE_ID}")
if [[ -n "${PSI_PX4_CHECKOUT:-}" ]]; then
  PSI_DOCKER_ARGS+=(--mount "type=bind,source=${PSI_PX4_CHECKOUT},target=/opt/px4")
fi
docker run "${PSI_DOCKER_ARGS[@]}" "${PSI_IMAGE_NAME}" "$@"
