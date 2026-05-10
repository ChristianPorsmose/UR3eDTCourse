import time
import threading
from pathlib import Path

import numpy as np
import pandas as pd
from influxdb_client import InfluxDBClient

from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import PhysicalTwinState, WearStatus
from communication.typed_protocol_client import TypedRabbitMQClient
import queue

CHECK_INTERVAL = 300    # seconds between wear checks
ANALYSIS_WINDOW = 1800  # seconds of history to analyse
MIN_SAMPLES = 50        # minimum aligned data points required
WEAR_RATIO = 2.0        # late MAD must be this many times early MAD to flag wear
MIN_BASELINE_DEV = 1e-4 # rad — below this early MAD is treated as noise
N_JOINTS = 6

publish_queue: queue.Queue[WearStatus] = queue.Queue()
JOINT_COLS = [f"q_actual_{i}" for i in range(N_JOINTS)]


def fetch_joint_df(query_api, bucket: str, measurement: str) -> pd.DataFrame | None:
    field_filter = " or ".join(f'r["_field"] == "q_actual_{i}"' for i in range(N_JOINTS))
    query = f"""
from(bucket: "{bucket}")
  |> range(start: -{ANALYSIS_WINDOW}s)
  |> filter(fn: (r) => r["_measurement"] == "{measurement}")
  |> filter(fn: (r) => {field_filter})
  |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
  |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
"""
    result = query_api.query_data_frame(query)
    if isinstance(result, list):
        if not result:
            return None
        result = pd.concat(result, ignore_index=True)
    if result.empty or not all(c in result.columns for c in JOINT_COLS):
        return None
    df = result.set_index("_time")[JOINT_COLS].dropna()
    df.index = pd.to_datetime(df.index, utc=True)
    return df.sort_index()


def check_wear(query_api, bucket: str) -> None:
    pt = fetch_joint_df(query_api, bucket, "robotarm.pt.state")
    kin = fetch_joint_df(query_api, bucket, "rt_model.dt.state")

    if pt is None or kin is None:
        print("Wear check: insufficient data in InfluxDB")
        return

    merged = pt.join(kin, how="inner", lsuffix="_pt", rsuffix="_kin")
    if len(merged) < MIN_SAMPLES:
        print(f"Wear check: only {len(merged)} aligned samples (need {MIN_SAMPLES})")
        return

    mid = len(merged) // 2
    affected_joints = []

    for i in range(N_JOINTS):
        dev = (merged[f"q_actual_{i}_pt"] - merged[f"q_actual_{i}_kin"]).abs()
        early_mad = dev.iloc[:mid].mean()
        late_mad = dev.iloc[mid:].mean()
        if early_mad >= MIN_BASELINE_DEV and late_mad / early_mad > WEAR_RATIO:
            affected_joints.append(i)

    wear_detected = len(affected_joints) > 0
    print(f"Wear check: wear_detected={wear_detected}, affected_joints={affected_joints}")
    publish_queue.put(WearStatus(wear_detected=wear_detected, affected_joints=affected_joints))


def wear_loop(query_api, bucket: str) -> None:
    while True:
        try:
            check_wear(query_api, bucket)
        except Exception as e:
            print(f"Wear check error: {e}")
        time.sleep(CHECK_INTERVAL)


def main():
    connect_config = load_config(Path("connect.yml"))
    influx_config = load_config(Path("influxdb.yml"))

    with (
        InfluxDBClient(**influx_config) as influx_client,
        TypedRabbitMQClient(Rabbitmq(**connect_config)) as typed_client,
    ):
        query_api = influx_client.query_api()
        bucket = influx_config["bucket"]

        # no-op subscription to keep the RabbitMQ IO loop alive for publishing
        typed_client.subscribe(PhysicalTwinState, lambda _: None, "wear_detection_heartbeat")

        threading.Thread(target=wear_loop, args=(query_api, bucket), daemon=True).start()
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
