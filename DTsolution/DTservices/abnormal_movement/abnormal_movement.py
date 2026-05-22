from collections import deque
import threading
import numpy as np
from pathlib import Path
from sklearn.ensemble import IsolationForest
from communication.typed_protocol import PhysicalTwinState, KinematicModelState, Deviation
from communication.typed_protocol_client import TypedRabbitMQClient
from utils.utils import interpolate, load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
import queue

latest_mock_queue: queue.Queue[PhysicalTwinState] = queue.Queue(maxsize=1)
kinematic_queue: deque[KinematicModelState] = deque(maxlen=20)
publish_queue: queue.Queue[Deviation] = queue.Queue()

N_JOINTS = 6
WARMUP_SAMPLES = 100  # samples before first model fit
REFIT_INTERVAL = 50   # retrain every N new samples
HISTORY_SIZE = 500    # rolling window per joint
CONTAMINATION = 0.05  # expected fraction of anomalies in training data

joint_history: list[deque[float]] = [deque(maxlen=HISTORY_SIZE) for _ in range(N_JOINTS)]
models: list[IsolationForest | None] = [None] * N_JOINTS
samples_since_refit: list[int] = [0] * N_JOINTS
model_locks: list[threading.Lock] = [threading.Lock() for _ in range(N_JOINTS)]


def refit_model(joint_idx: int) -> None:
    data = np.array(list(joint_history[joint_idx])).reshape(-1, 1)
    new_model = IsolationForest(contamination=CONTAMINATION, n_estimators=100, random_state=42)
    new_model.fit(data)
    with model_locks[joint_idx]:
        models[joint_idx] = new_model


def deviation_loop() -> None:
    while True:
        latest_mock = latest_mock_queue.get()
        mock_time = latest_mock.timestamp
        kin_value = interpolate(mock_time, list(kinematic_queue), lambda x: x.q_actual)
        if kin_value is None:
            continue

        deviation = (np.array(latest_mock.q_actual) - np.array(kin_value)).tolist()

        anomaly_detected = False
        for i in range(N_JOINTS):
            joint_history[i].append(deviation[i])
            samples_since_refit[i] += 1

            if len(joint_history[i]) >= WARMUP_SAMPLES and samples_since_refit[i] >= REFIT_INTERVAL:
                threading.Thread(target=refit_model, args=(i,), daemon=True).start()
                samples_since_refit[i] = 0

            with model_locks[i]:
                current_model = models[i]

            if current_model is not None and current_model.predict([[deviation[i]]])[0] == -1:
                anomaly_detected = True

        if anomaly_detected:
            publish_queue.put(Deviation(joint_deviations=deviation))


def main():
    config = load_config(Path("connect.yml"))
    print("STARTING ABNORMAL MOVEMENT")
    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:

        typed_client.subscribe(
            PhysicalTwinState, lambda x: latest_mock_queue.put(x), "abnormal_queue_1"
        )
        typed_client.subscribe(
            KinematicModelState, lambda x: kinematic_queue.append(x), "abnormal_queue_2"
        )

        threading.Thread(target=deviation_loop, daemon=True).start()
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
