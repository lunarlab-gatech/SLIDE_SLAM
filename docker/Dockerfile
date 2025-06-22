FROM xurobotics/slide-slam:latest

# Set user arguments (being username, echo $UID, and id -g)
ARG USERNAME=dbutterfield3
ARG USER_UID=3332128
ARG USER_GID=2626

# Create a non-root user with matching UID/GID
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add a custom .bashrc with warm colors and fixed hostname
RUN echo 'export PS1="\\[\\e[1;31m\\]\\u@slideslam_baseline\\[\\e[0m\\]:\\[\\e[2;37m\\]\\w\\[\\e[0m\\]\\$ "' > /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

# Set default user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Install tmuxp
RUN sudo apt install tmuxp lsof -y