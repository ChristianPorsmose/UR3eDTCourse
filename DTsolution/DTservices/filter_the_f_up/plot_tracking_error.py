"""
Fetch q_actual/q_target from PhysicalTwinState and q_actual from FilteredState,
then plot raw vs filtered tracking error against q_target.

Run from this directory:
    python plot_tracking_error.py [--minutes 2]
"""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from influxdb_client import InfluxDBClient

INFLUX_URL    = "http://localhost:8086"
INFLUX_TOKEN  = "SECRET_AF"
INFLUX_ORG    = "ur3e"
INFLUX_BUCKET = "ur3e"
RAW_MEASUREMENT      = "robotarm.pt.state"
FILTERED_MEASUREMENT = "filtered.state"
NUM_JOINTS = 6


def fetch_measurement(measurement: str, fields: list[str], minutes: int) -> pd.DataFrame:
    fields_filter = " or ".join(f'r["_field"] == "{f}"' for f in fields)
    query = f"""
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -{minutes}m)
  |> filter(fn: (r) => r["_measurement"] == "{measurement}")
  |> filter(fn: (r) => {fields_filter})
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
  |> sort(columns: ["_time"])
"""
    with InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG) as client:
        df = client.query_api().query_data_frame(query)

    if df is None or (isinstance(df, list) and len(df) == 0):
        raise RuntimeError(f"No data for '{measurement}'.")
    if isinstance(df, list):
        df = pd.concat(df, ignore_index=True)
    return df.sort_values("_time").dropna(subset=fields).reset_index(drop=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--minutes", type=int, default=2)
    args = parser.parse_args()

    actual_cols = [f"q_actual_{i}" for i in range(NUM_JOINTS)]
    target_cols = [f"q_target_{i}" for i in range(NUM_JOINTS)]

    print(f"Fetching last {args.minutes} minute(s) from {INFLUX_URL} …")
    try:
        raw      = fetch_measurement(RAW_MEASUREMENT, actual_cols + target_cols, args.minutes)
        filtered = fetch_measurement(FILTERED_MEASUREMENT, actual_cols, args.minutes)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    # Align filtered onto raw timestamps (nearest, 500 ms tolerance)
    merged = pd.merge_asof(
        raw[["_time"] + actual_cols + target_cols],
        filtered[["_time"] + actual_cols].rename(columns={c: f"filt_{c}" for c in actual_cols}),
        on="_time", direction="nearest", tolerance=pd.Timedelta("500ms"),
    ).dropna()

    t = merged["_time"].astype("int64").to_numpy() / 1e9
    t -= t[0]

    q_actual   = merged[actual_cols].to_numpy()
    q_target   = merged[target_cols].to_numpy()
    q_filtered = merged[[f"filt_q_actual_{i}" for i in range(NUM_JOINTS)]].to_numpy()

    raw_err      = q_actual   - q_target
    filtered_err = q_filtered - q_target
    raw_norm      = np.linalg.norm(raw_err,      axis=1)
    filtered_norm = np.linalg.norm(filtered_err, axis=1)

    print(f"\nSamples: {len(t)}  |  Duration: {t[-1]:.1f}s")
    print(f"{'Joint':<8} {'Raw max':>12} {'Raw mean':>12} {'Filt max':>12} {'Filt mean':>12}")
    for j in range(NUM_JOINTS):
        print(f"  J{j:<5} {np.abs(raw_err[:,j]).max():>12.2e} {np.abs(raw_err[:,j]).mean():>12.2e}"
              f" {np.abs(filtered_err[:,j]).max():>12.2e} {np.abs(filtered_err[:,j]).mean():>12.2e}")

    colors = plt.cm.tab10(np.linspace(0, 0.6, NUM_JOINTS))
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle("Tracking error: q_actual vs q_target  (raw vs filtered)", fontsize=13, fontweight="bold")

    # Per-joint raw error
    for j in range(NUM_JOINTS):
        axes[0].plot(t, raw_err[:, j], color=colors[j], lw=0.9, alpha=0.7, label=f"J{j}")
    axes[0].axhline(0, color="white", lw=0.5, alpha=0.3)
    axes[0].set_ylabel("Error (rad)")
    axes[0].set_title("Raw error  (q_actual − q_target)")
    axes[0].legend(ncol=6, fontsize=8)

    # Per-joint filtered error
    for j in range(NUM_JOINTS):
        axes[1].plot(t, filtered_err[:, j], color=colors[j], lw=0.9, alpha=0.7, label=f"J{j}")
    axes[1].axhline(0, color="white", lw=0.5, alpha=0.3)
    axes[1].set_ylabel("Error (rad)")
    axes[1].set_title("Filtered error  (q_filtered − q_target)")
    axes[1].legend(ncol=6, fontsize=8)

    # Error norm comparison
    axes[2].fill_between(t, raw_norm,      alpha=0.2, color="tab:orange", label=f"Raw  (mean {raw_norm.mean():.2e})")
    axes[2].fill_between(t, filtered_norm, alpha=0.3, color="tab:blue",   label=f"Filtered  (mean {filtered_norm.mean():.2e})")
    axes[2].plot(t, raw_norm,      color="tab:orange", lw=1.2)
    axes[2].plot(t, filtered_norm, color="tab:blue",   lw=1.2)
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("‖error‖ (rad)")
    axes[2].set_title("Error norm — raw vs filtered")
    axes[2].legend(fontsize=9)

    plt.tight_layout()
    plt.savefig("tracking_error.png", dpi=150, bbox_inches="tight")
    print("\nSaved tracking_error.png")
    plt.show()


if __name__ == "__main__":
    main()
