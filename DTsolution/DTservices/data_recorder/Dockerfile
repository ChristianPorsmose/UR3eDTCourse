FROM python:3.11-slim

WORKDIR /app

RUN  pip install pika influxdb-client pyyaml numpy scipy

COPY communication/__init__.py communication/
COPY communication/rabbitmq.py communication/
COPY communication/protocol.py communication/
COPY communication/typed_protocol.py communication/
COPY communication/typed_protocol_client.py communication/

COPY DTsolution/utils utils
COPY DTsolution/DTservices/data_recorder/data_recorder.py .

CMD ["python","-u", "data_recorder.py"]
