####################################################
#
#   AutoDRIVE Simulator Dockerfile
#
####################################################

# Set base image and environment variables
FROM nvidia/vulkan:1.1.121-cuda-10.1--ubuntu18.04

# Add CUDA repository key
RUN rm /etc/apt/sources.list.d/cuda.list
RUN rm /etc/apt/sources.list.d/nvidia-ml.list
RUN apt-key del 7fa2af80
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

# Install Debian packages
RUN apt update \
    && apt install -y --no-install-recommends \
        sudo \
        wget \
        gedit \
        nano \
        vim \
        curl \
        unzip \
        net-tools \
        libvulkan1 \
        libc++1 \
        libc++abi1 \
        vulkan-utils \
    && rm -rf /var/lib/apt/lists/*

# Install tools for display
RUN apt update --fix-missing \
    && apt install -y xvfb ffmpeg libgdal-dev libsm6 libxext6

# Copy AutoDRIVE Simulator executable
COPY autodrive_simulator /home/autodrive_simulator

# Set work directory and register executable
WORKDIR /home/autodrive_simulator
RUN chmod +x /home/autodrive_simulator/AutoDRIVE\ Simulator.x86_64

# Set entrypoint
COPY autodrive_simulator.sh /home
ENTRYPOINT ["/home/autodrive_simulator.sh"]