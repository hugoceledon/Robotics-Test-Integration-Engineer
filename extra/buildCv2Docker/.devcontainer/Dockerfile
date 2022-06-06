# -----------------------------------------------------------------------------
# Hugo Celedón
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
FROM ubuntu:20.04

ARG USERNAME=koda
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# -----------------------------------------------------------------------------
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # ------------------------------
    # Free memory
    && apt autoremove -y && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Install essentials
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # ------------------------------
    # Basics
    apt-utils \
    bash-completion \
    build-essential \
    htop \
    lsb-release \
    openssh-client \
    # ------------------------------
    # Download repositories and files
    apt-transport-https \
    ca-certificates \
    curl \
    git \
    gnupg-agent \
    gnupg2 \
    iproute2 \
    wget \
    # ------------------------------
    # Network stuff
    lsb-release \
    network-manager \
    wireless-tools \
    # ------------------------------
    # Install clang formater
    clang-format \
    # ------------------------------
    # Parsing yaml in c++
    libyaml-cpp-dev \
    # ------------------------------
    # Python
    python3 \
    python3-argcomplete \
    python3-distutils \
    python3-pip \
    python3-tk \
    # ------------------------------
    # Audio tools
    alsa-base \
    alsa-tools \
    alsa-utils \
    libasound2-dev \ 
    pulseaudio \
    # ------------------------------
    # Debug tools
    valgrind \
    # ---------------------------------
    && apt autoremove -y && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Install cmake from source, because latest version not available in apt repos
ENV CMAKE_VERSION=3.22
RUN wget https://cmake.org/files/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.0-linux-x86_64.sh \
    && bash cmake-${CMAKE_VERSION}.0-linux-x86_64.sh --skip-license --prefix=/usr/local

# -----------------------------------------------------------------------------
# Python Libraries
RUN pip3 install -U pip \
    && python3 -m pip install \
    # Add EXTRA DEV Python dependencies in here
    # ------------------------------
    # Tools to check code compliance
    typed-ast \
    black \
    click \
    pylint~=2.11.1 \
    # ------------------------------
    # Installing basics
    setuptools \
    # ------------------------------
    # Plotter Tools
    matplotlib\
    && rm -rf /home/koda/.cache/pip


RUN /bin/bash /workspace/installCv2.sh

CMD ["/bin/bash"]