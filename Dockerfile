FROM ubuntu:bionic

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
