#!/usr/bin/env bash

GZ_DIR=~/PX4-Autopilot/Tools/simulation/gz

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

WORLDS=$(realpath "${SCRIPT_DIR}/../simulation/worlds")
GZ_WORLDS=$(realpath "${GZ_DIR}/worlds")
MODELS=$(realpath "${SCRIPT_DIR}/../simulation/models")
GZ_MODELS=$(realpath "${GZ_DIR}/models")

echo "Linking all ${WORLDS} to ${GZ_WORLDS}"
ln -sf $(realpath "${WORLDS}/*") ${GZ_WORLDS}
echo "Linking all ${MODELS} to ${GZ_MODELS}"
ln -sf $(realpath "${MODELS}/*") ${GZ_MODELS}