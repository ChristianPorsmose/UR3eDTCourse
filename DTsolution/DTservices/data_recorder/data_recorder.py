from dataclasses import fields
from datetime import datetime, timezone
from functools import partial
from typing import Callable, Any, TypeAlias

from pathlib import Path

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import ASYNCHRONOUS, WriteApi

from utils.utils import load_config
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import (
    PhysicalTwinState,
    LoadProgram,
    InjectWear,
    KinematicModelState,
    StuckJointStatus,
    FilteredState,
    WearStatus,
    Deviation,
    MsgProtocol
)
from communication.typed_protocol_client import TypedRabbitMQClient


WriterFn : TypeAlias = Callable[[Point], Any]


def add_field(point: Point, key: str, value: Any) -> Point:
    if value is None:
        return point

    if isinstance(value, list):
        for i, v in enumerate(value):
            point = point.field(f"{key}_{i}", v)
        return point

    if isinstance(value, (int, float, str, bool)):
        return point.field(key, value)

    return point.field(key, str(value))


def write_dataclass(writer: WriterFn, dp: MsgProtocol) -> None:
    point = (
        Point(dp.routing_key())
        .time(datetime.now(timezone.utc))
        .tag("msg_type", type(dp).__name__)
    )

    for f in fields(dp):
        value = getattr(dp, f.name)
        point = add_field(point, f.name, value)

    writer(record=point)

def ensure_bucket_exists(client: InfluxDBClient, bucket_name: str, org_name: str) -> None:
    """Checks if a bucket exists, and creates it on the fly if it doesn't."""
    buckets_api = client.buckets_api()
    
    # Check if bucket already exists
    if buckets_api.find_bucket_by_name(bucket_name):
        print(f"Bucket '{bucket_name}' already exists.")
        return

    print(f"Bucket '{bucket_name}' not found. Creating it...")
    
    # We need the org_id to create a bucket
    orgs_api = client.organizations_api()
    orgs = orgs_api.find_organizations(org=org_name)
    
    if not orgs:
        raise ValueError(f"Organization '{org_name}' not found.")
    
    org_id = orgs[0].id
    
    # Create the bucket
    buckets_api.create_bucket(bucket_name=bucket_name, org_id=org_id)
    print(f"Successfully created bucket: '{bucket_name}'")

def main():
    connect_config = load_config(Path("connect.yml"))
    influx_config = load_config(Path("influxdb.yml"))

    with (
        TypedRabbitMQClient(Rabbitmq(**connect_config)) as typed_client,
        InfluxDBClient(**influx_config) as client
    ):
        # Ensure the bucket exists before we start writing
        ensure_bucket_exists(client, influx_config["bucket"], influx_config["org"])

        write_api: WriteApi = client.write_api(
            write_options=ASYNCHRONOUS
        )

        writer = partial(
            write_api.write,
            bucket=influx_config["bucket"],
            org=influx_config["org"]
        )

        subscriptions = {
            PhysicalTwinState: write_dataclass,
            KinematicModelState: write_dataclass,
            StuckJointStatus: write_dataclass,
            LoadProgram: write_dataclass,
            InjectWear: write_dataclass,
            FilteredState: write_dataclass,
            WearStatus: write_dataclass,
            Deviation: write_dataclass,
        }

        for msg_type, handler in subscriptions.items():
            typed_client.subscribe(
                msg_type,
                lambda msg, h=handler: h(writer, msg),
                queue_name=msg_type.__name__
            )

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()