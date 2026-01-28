#!/bin/bash

################################################################################

# Start the D-Bus daemon (if available)
if command -v service >/dev/null 2>&1 && [ -x /etc/init.d/dbus ]; then
	echo "[INIT] Starting dbus service (if not already running)..."
	service dbus start || true
fi

################################################################################

# Start the avahi daemon (for resolving DNS name) if available
if command -v service >/dev/null 2>&1 && [ -x /etc/init.d/avahi-daemon ]; then
	echo "[INIT] Starting avahi-daemon (if not already running)..."
	service avahi-daemon start || true
fi

# # dockerを実行する人のユーザー名のディレクトリを作成して、その中に.bashrcをコピーすることで、bashの設定を反映
# mkdir -p $HOME
# cp /home/.bashrc $HOME/.bashrc

################################################################################

# Apply ROS logging fix (idempotent). Only if ROS Noetic path exists.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "/opt/ros/noetic" ]; then
	echo "[INIT] Applying rosgraph/roslogging.py fix with mode 0644..."
	bash "$SCRIPT_DIR/fix_roslogging.sh" || echo "[INIT] Warning: fix_roslogging.sh failed; continuing."
else
	echo "[INIT] Skipping roslogging fix: /opt/ros/noetic not found."
fi

################################################################################

# Prepare cache directories based on env vars (if provided via compose)
mkdir -p "${UV_CACHE_DIR}" "${UV_DATA_DIR}" "${UV_PYTHON_DIR}" \
				 "${OPENPI_DATA_HOME}" "${HF_HOME}" "${TMPDIR}" "${XDG_CACHE_HOME}" || true

# Optionally perform a one-time uv sync if no venv/lock is present
if [ -d "/home/openpi" ]; then
	cd /home/openpi || true
	if [ -f "uv.lock" ] && [ ! -f "/home/cache/.uv_synced" ]; then
		echo "[INIT] Performing initial 'uv sync' to warm caches..."
		GIT_LFS_SKIP_SMUDGE=1 uv sync && touch /home/cache/.uv_synced || true
	fi
fi

################################################################################

# Source ROS env and build catkin via external script, and persist into bashrc
ROS_INIT_SCRIPT="/home/docker_scripts/initialize-ros-env.sh"

if [ -f "$ROS_INIT_SCRIPT" ]; then
	echo "[INIT] Setting ROS environment from $ROS_INIT_SCRIPT"
	# shellcheck disable=SC1090
	echo "$ROS_INIT_SCRIPT" >> /root/.bashrc
else
	echo "[INIT] Warning: ROS init script not found at $ROS_INIT_SCRIPT. Skipping ROS initialization."
fi

# Mark initialization as complete for host-side waiters

################################################################################

# Keep the Docker container running in the background.
# https://stackoverflow.com/questions/30209776/docker-container-will-automatically-stop-after-docker-run-d
tail -f /dev/null