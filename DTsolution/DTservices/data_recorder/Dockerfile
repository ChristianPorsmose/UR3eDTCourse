FROM python:3.11-slim

WORKDIR /app

RUN  pip install pika influxdb-client pyyaml

COPY communication/__init__.py communication/
COPY communication/rabbitmq.py communication/
COPY communication/protocol.py communication/


COPY DTsolution/DTservices/utils utils
COPY DTsolution/DTservices/data_recorder/data_recorder.py .

CMD ["python","-u", "data_recorder.py"]
