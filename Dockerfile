FROM ros:melodic-ros-base

RUN apt-get update && apt-get install --no-install-recommends -y \
    python \
    python-pip python-setuptools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup entrypoint
COPY . /car
WORKDIR /car/call_a_robot
RUN pip install -r requirements.txt

# add devcontainer user with sudo rights and a home directory
RUN useradd -ms /bin/bash devcontainer
RUN chown -R devcontainer /car

# allow sudo without password
RUN echo "devcontainer ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER devcontainer

CMD ["python", "callarobot.py"]
