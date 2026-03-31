FROM python:3.11-slim

WORKDIR /app

RUN  pip install pika pyyaml numpy

COPY communication/__init__.py communication/
COPY communication/rabbitmq.py communication/
COPY communication/protocol.py communication/

COPY DTsolution/utils utils
COPY DTsolution/DTservices/stuck_joint_detection/stuck_joint.py .

CMD ["python","-u", "stuck_joint.py"]