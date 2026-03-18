FROM python:3.11-slim

WORKDIR /app

RUN  pip install pika pyyaml numpy scipy

COPY communication/__init__.py communication/
COPY communication/rabbitmq.py communication/
COPY communication/protocol.py communication/

COPY DTsolution/DTservices/utils utils
COPY DTsolution/DTservices/abnormal_movement/abnormal_movement.py .

CMD ["python","-u", "abnormal_movement.py"]