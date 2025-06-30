FROM ghcr.io/esphome/esphome:2025.6.2
ARG USERNAME=esphome
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USERNAME

USER $USERNAME
