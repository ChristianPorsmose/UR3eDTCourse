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
    KinematicModelState,
    StuckJointStatus,
    FilteredState,
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


def main():
    connect_config = load_config(Path("connect.yml"))
    influx_config = load_config(Path("influxdb.yml"))

    with (
        TypedRabbitMQClient(Rabbitmq(**connect_config)) as typed_client,
        InfluxDBClient(**influx_config) as client
    ):
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
            FilteredState: write_dataclass,
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