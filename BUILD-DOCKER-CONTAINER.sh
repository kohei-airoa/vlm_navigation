#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
if [ -z "$DEEP_PROJECT_NAME" ]; then
  echo "Set DEEP_PROJECT_NAME (e.g. 'export DEEP_PROJECT_NAME=mytest')"
  exit 1
fi
PROJECT=$DEEP_PROJECT_NAME
CONTAINER="${PROJECT}_deep_1"
export HOSTNAME=$(hostname)
export CONTAINER=$CONTAINER  # Export the container name for docker compose to coherently set container name.

# モード選択: train | deploy （第1引数で指定。未指定は train）
MODE=${1:-train}
case "$MODE" in
  train)
    export DEEP_DOCKERFILE=./docker/Dockerfile.train
    : "${DEEP_IMAGE_TAG:=train}"
    ;;
  deploy)
    export DEEP_DOCKERFILE=./docker/Dockerfile.deploy
    : "${DEEP_IMAGE_TAG:=deploy}"
    ;;
  *)
    echo "Unknown mode: $MODE (expected 'train' or 'deploy')"
    exit 1
    ;;
esac
export DEEP_IMAGE_TAG=$DEEP_IMAGE_TAG

# データセットのパスを指定する
if [ -z "$DEEP_DATASET_PATH" ]; then
  export DEEP_DATASET_PATH=$(pwd)/datasets
  echo "DEEP_DATASET_PATH is not set. Set DEEP_DATASET_PATH to $DEEP_DATASET_PATH"
fi

# キャッシュのパスを指定する（ホスト側に永続化）
if [ -z "$DEEP_CACHE_ROOT_HOST" ]; then
  export DEEP_CACHE_ROOT_HOST=$(pwd)/.docker_cache
  echo "DEEP_CACHE_ROOT_HOST is not set. Using $DEEP_CACHE_ROOT_HOST"
fi

echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"
echo "$0: Using image tag: ${DEEP_IMAGE_TAG}"
echo "$0: Using dockerfile: ${DEEP_DOCKERFILE}"
echo "$0: DEEP_DATASET_PATH=${DEEP_DATASET_PATH}"
echo "$0: CACHE_HOST_DIR=${DEEP_CACHE_ROOT_HOST}"

# Stop and remove the Docker container.
EXISTING_CONTAINER_ID=$(docker ps -aq -f name=${CONTAINER})
if [ ! -z "${EXISTING_CONTAINER_ID}" ]; then
  echo "The container name ${CONTAINER} is already in use" 1>&2
  echo ${EXISTING_CONTAINER_ID}
  exit 1
fi

################################################################################

# Build the Docker image with the Nvidia GL library.
echo "starting build"
docker compose -p ${PROJECT} -f ./docker/docker-compose.yml build
