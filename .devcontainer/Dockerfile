FROM ghcr.io/esphome/esphome:2026.3.3
ARG USERNAME=esphome
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USERNAME

ADD https://raw.githubusercontent.com/esphome/esphome/refs/heads/dev/.clang-format /esphome/.clang-format
RUN chmod +r /esphome/.clang-format

RUN apt update && apt install -y gpg;

RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | gpg --dearmor -o /usr/share/keyrings/githubcli-archive-keyring.gpg;
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null;
RUN apt update && apt install -y gh;

USER $USERNAME
