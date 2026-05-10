import time
import threading
from collections import deque
from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d

from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import PhysicalTwinState, KinematicModelState, WearStatus
from communication.typed_protocol_client import TypedRabbitMQClient
import queue

CHECK_INTERVAL = 60     
ANALYSIS_WINDOW = 300   
MIN_SAMPLES = 50        
WEAR_RATIO = 2.0        # late MAD / early MAD ratio to flag wear
MIN_BASELINE_DEV = 1e-4 # rad — skip ratio check if early baseline is pure noise
WEAR_DELTA = 0.02       # rad — absolute MAD increase also flags wear
N_JOINTS = 6

publish_queue: queue.Queue[WearStatus] = queue.Queue()

pt_deque: deque[PhysicalTwinState] = deque(maxlen=5000)
kin_deque: deque[KinematicModelState] = deque(maxlen=10000)


def check_wear() -> None:
    cutoff = time.time() - ANALYSIS_WINDOW
    pt_snap = [s for s in list(pt_deque) if s.timestamp >= cutoff]
    kin_snap = [s for s in list(kin_deque) if s.timestamp >= cutoff]

    if len(pt_snap) < MIN_SAMPLES:
        print(f"Wear check: only {len(pt_snap)} PT samples (need {MIN_SAMPLES})")
        return
    if len(kin_snap) < 2:
        print(f"Wear check: insufficient KIN samples ({len(kin_snap)})")
        return

    pt_times = np.array([s.timestamp for s in pt_snap])
    pt_pos = np.array([s.q_actual for s in pt_snap])      

    kin_times_raw = np.array([s.timestamp for s in kin_snap])
    kin_pos_raw = np.array([s.q_actual for s in kin_snap])

    sort_idx = np.argsort(kin_times_raw)
    kin_times = kin_times_raw[sort_idx]
    kin_pos = kin_pos_raw[sort_idx]

    valid = (pt_times >= kin_times[0]) & (pt_times <= kin_times[-1])
    if valid.sum() < MIN_SAMPLES:
        print(f"Wear check: only {valid.sum()} overlapping samples (need {MIN_SAMPLES})")
        return

    f = interp1d(kin_times, kin_pos, axis=0, kind="linear")
    kin_interp = f(pt_times[valid])                         
    dev = np.abs(pt_pos[valid] - kin_interp)                

    mid = len(dev) // 2
    affected_joints = []

    for i in range(N_JOINTS):
        early_mad = dev[:mid, i].mean()
        late_mad = dev[mid:, i].mean()
        ratio_ok = early_mad >= MIN_BASELINE_DEV and late_mad / early_mad > WEAR_RATIO
        delta_ok = late_mad - early_mad > WEAR_DELTA
        if ratio_ok or delta_ok:
            affected_joints.append(i)

    wear_detected = len(affected_joints) > 0
    early_late = [f"{dev[:mid,i].mean():.4f}/{dev[mid:,i].mean():.4f}" for i in range(N_JOINTS)]
    print(f"Wear check: wear_detected={wear_detected}, joints={affected_joints}, early/late={early_late}")
    publish_queue.put(WearStatus(wear_detected=wear_detected, affected_joints=affected_joints))


def wear_loop() -> None:
    while True:
        try:
            check_wear()
        except Exception as e:
            print(f"Wear check error: {e}")
        time.sleep(CHECK_INTERVAL)


def main():
    connect_config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**connect_config)) as typed_client:
        typed_client.subscribe(PhysicalTwinState, pt_deque.append, "wear_det_pt")
        typed_client.subscribe(KinematicModelState, kin_deque.append, "wear_det_kin")

        threading.Thread(target=wear_loop, daemon=True).start()
        threading.Thread(
            target=lambda: typed_publisher_loop(typed_client, publish_queue),
            daemon=True
        ).start()

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
