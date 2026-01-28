FROM nvidia/cuda:12.2.2-devel-ubuntu22.04
SHELL ["/bin/bash", "-c"]

# Configure image
ARG PYTHON_VERSION=3.10
ARG DEBIAN_FRONTEND=noninteractive



# Install apt dependencies (基本ツール、Python、その他必要なライブラリ)
RUN apt update && apt install -y --no-install-recommends \
    build-essential cmake \
    git git-lfs openssh-client \
    nano vim less util-linux tree \
    htop atop nvtop \
    sed gawk grep curl wget zip unzip \
    tcpdump sysstat screen tmux \
    libusb-1.0-0-dev libglib2.0-0 libgl1-mesa-glx libegl1-mesa \
    speech-dispatcher portaudio19-dev libgeos-dev \
    python${PYTHON_VERSION} python${PYTHON_VERSION}-venv python3-pip\
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# Install ffmpeg build dependencies
RUN apt update && apt install -y --no-install-recommends \
    autoconf automake yasm \
    libass-dev \
    libfreetype6-dev \
    libgnutls28-dev \
    libunistring-dev \
    libmp3lame-dev \
    libtool \
    libvorbis-dev \
    meson \
    ninja-build \
    pkg-config \
    texinfo \
    yasm \
    zlib1g-dev \
    nasm \
    libx264-dev \
    libx265-dev libnuma-dev \
    libvpx-dev \
    libfdk-aac-dev \
    libopus-dev \
    libsvtav1-dev libsvtav1enc-dev libsvtav1dec-dev \
    libdav1d-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install gh CLI tool
RUN (type -p wget >/dev/null || (apt update && apt-get install wget -y)) \
    && mkdir -p -m 755 /etc/apt/keyrings \
    && wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
    && chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
    && apt update \
    && apt install gh -y \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Setup python: シンボリックリンクの作成
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install poetry
RUN curl -sSL https://install.python-poetry.org | python - --version 1.8.5
ENV PATH="/root/.local/bin:$PATH"
RUN echo 'if [ "$HOME" != "/root" ]; then ln -sf /root/.local/bin/poetry $HOME/.local/bin/poetry; fi' >> /root/.bashrc
RUN poetry config virtualenvs.create false
RUN poetry config virtualenvs.in-project true

# Install uv
RUN wget -qO- https://astral.sh/uv/install.sh | sh

# Install qt
RUN apt update && apt install -y --no-install-recommends \
 	qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools

# Set EGL as the rendering backend for MuJoCo
ENV MUJOCO_GL="egl"


# Copy helper scripts and user alignment files
COPY ./docker/scripts/. /root/scripts/
RUN chmod -R +x /root/scripts/*
COPY ./docker/scripts/.bashrc /home/
RUN chown 1000:1000 /home/.bashrc
RUN chown -R 1000:1000 /home

# # Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT []

CMD ["/bin/bash"]

