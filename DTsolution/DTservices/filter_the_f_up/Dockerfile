FROM python:3.11-slim

WORKDIR /app

RUN  pip install pika influxdb-client pyyaml

COPY communication/__init__.py communication/
COPY communication/rabbitmq.py communication/
COPY communication/protocol.py communication/

COPY DTsolution/utils utils
COPY DTsolution/DTservices/filter_the_f_up/filter_loop.py .

CMD ["python","-u", "filter_loop.py"]
