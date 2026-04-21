FROM communication-base

RUN apt-get update && apt-get install -y \
    build-essential gcc g++ python3-dev

ENV POETRY_VIRTUALENVS_CREATE=false

RUN pip install poetry
COPY pyproject.toml .

COPY startup/ startup/

RUN touch README.md

RUN poetry install

COPY DTsolution/models DTsolution/models
COPY DTsolution/DTservices/RT_robot_prediction .


CMD ["python", "-u", "RT_robot_prediction.py"]
