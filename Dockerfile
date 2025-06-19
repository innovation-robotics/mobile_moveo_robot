# Use the official Ubuntu 22.04 base image
FROM ubuntu:22.04

# Set a maintainer label (optional but good practice)
LABEL maintainer="info@innovation-robotics.com"

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Example of copying a file
COPY config/ /site_config/

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

  # You can add more instructions here as needed, for example:
# COPY . /app
# RUN make
# EXPOSE 8080
# CMD ["./your-application"]

# Default command to run when the container starts
# This will keep the container running indefinitely for inspection
CMD ["bash"]

