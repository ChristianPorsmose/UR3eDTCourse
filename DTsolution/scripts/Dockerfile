FROM python:3.11-slim

WORKDIR /app

RUN  pip install pika pyyaml numpy

COPY communication/__init__.py communication/
COPY communication/rabbitmq.py communication/
COPY communication/protocol.py communication/

COPY DTsolution/utils utils
COPY DTsolution/scripts/self_cleaning_move_generator.py .

CMD ["python","-u", "self_cleaning_move_generator.py"]