# This is an auto generated Dockerfile for ros:indigo-ros-core
# generated from templates/docker_images/create_ros_core_image.Dockerfile.em
# generated on 2017-05-23 21:41:45 +0000
FROM ubuntu:bionic

# install bootstrap tools
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

CMD ["python", "callarobot.py"]
