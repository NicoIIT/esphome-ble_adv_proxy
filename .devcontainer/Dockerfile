FROM ghcr.io/esphome/esphome:2025.12.3
ARG USERNAME=esphome
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USERNAME

ADD https://raw.githubusercontent.com/esphome/esphome/refs/heads/dev/.clang-format /esphome/.clang-format
RUN chmod +r /esphome/.clang-format

USER $USERNAME
