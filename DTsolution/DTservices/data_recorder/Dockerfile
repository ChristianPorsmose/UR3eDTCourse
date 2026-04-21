FROM communication-base
RUN  pip install influxdb-client
COPY DTsolution/DTservices/data_recorder/data_recorder.py .
CMD ["python","-u", "data_recorder.py"]
