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
    : "${DEEP_IMAGE_TAG:=train}"  # 既存のデフォルトを維持
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

# HSR関連の設定（deployのみ必須）
if [ "$MODE" = "deploy" ]; then
  if [ -n "${HSR_IP}" ]; then
    echo "No argument provided. Using defined HSR_IP: ${HSR_IP}"
  elif [ -n "${ROBOT_NAME}" ]; then
    HSR_NAME="${ROBOT_NAME}"
    echo "No argument provided. Now resolving global host name '${HSR_NAME}'..."
    HSR_IP=$(getent hosts ${HSR_NAME} | awk '{ print $1 }')
    if [ "$?" != "0" ] || [ -z "${HSR_IP}" ]; then
      echo "Failed to resolve global host name '${HSR_NAME}'. Trying local host name..."
      HSR_NAME=${HSR_NAME}.local
      echo "Now resolving local host name '${HSR_NAME}'..."
      HSR_IP=$(avahi-resolve -4 --name ${HSR_NAME} | cut -f 2)
      if [ "$?" != "0" ]; then
        echo "Failed to execute 'avahi-resolve'. You may need to install 'avahi-utils'."
        exit 1
      elif [ -z "${HSR_IP}" ]; then
        echo "Failed to resolve local host name '${HSR_NAME}'."
        exit 1
      else
        echo "Successfully resolved local host name '${HSR_NAME}' as '${HSR_IP}'."
      fi
    else
      echo "Successfully resolved global host name '${HSR_NAME}' as '${HSR_IP}'."
    fi
  else
    echo "[ERROR] Please provide one of the following:"
    echo "    - pass the robot name as an argument"
    echo "    - set the 'HSR_IP' environment variable"
    echo "    - set the 'ROBOT_NAME' environment variable"
    exit 1
  fi
  if [ -n "${ROS_IP:-}" ]; then
    echo "ROS_IP is already set to ${ROS_IP}. Using it."
  else
    if command -v ifconfig >/dev/null 2>&1; then
      HOST_IP_LIST=$(ifconfig | grep 'inet ' | awk '{print $2}')
    else
      HOST_IP_LIST=$(ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1)
    fi
    echo "Please choose IP from the following list:"
    select ip in $HOST_IP_LIST; do
      if [ -n "$ip" ]; then
        export ROS_IP=$ip
        echo "ROS_IP is not set. Setting it to ${ROS_IP}."
        break
      fi
    done
  fi
else
  echo "[INFO] Skipping HSR network setup in train mode."
fi

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
# 必要なディレクトリを作成
mkdir -p "${DEEP_CACHE_ROOT_HOST}"/{uv-cache,uv-data,uv-python,openpi_cache,huggingface_home,tmp,cache}

echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"
echo "$0: Using image tag: ${DEEP_IMAGE_TAG}"
echo "$0: Using dockerfile: ${DEEP_DOCKERFILE}"
echo "$0: HSR_IP=${HSR_IP}"
echo "$0: ROS_IP=${ROS_IP}"
echo "$0: DEEP_DATASET_PATH=${DEEP_DATASET_PATH}"
echo "$0: CACHE_HOST_DIR=${DEEP_CACHE_ROOT_HOST}"

# Run the Docker container in the background.
# Any changes made to './docker/docker-compose.yml' will recreate and overwrite the container.
docker compose -p ${PROJECT} -f ./docker/docker-compose.yml up -d

# Display GUI through X Server by granting full access to any external client.
xhost +

# Enter the Docker container with a Bash shell (with or without a custom 'roslaunch' command).
case "$2" in
  ( "" )
    if [ "$MODE" = "deploy" ]; then
      INIT_CMD="sed -i 's/TMP_IP/${ROS_IP}/' ~/.bashrc; sed -i 's/localhost:11311/${HSR_IP}:11311/' ~/.bashrc;"
    else
      INIT_CMD=":"
    fi
    docker exec -i -t ${CONTAINER} bash -c "${INIT_CMD} source ~/.bashrc; bash"
    ;;
  ( * )
    echo "Failed to enter the Docker container '${CONTAINER}': '$2' is not a valid argument value."
    ;;
esac
