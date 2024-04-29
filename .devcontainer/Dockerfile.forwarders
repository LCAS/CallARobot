FROM python:3.9

RUN pip install paho-mqtt==1.6.1 websocket-client==1.3.3 rel==0.4.7
COPY . /CAR-Forwarders
WORKDIR /CAR-Forwarders

CMD ["python", "MtoWENV.py"]
