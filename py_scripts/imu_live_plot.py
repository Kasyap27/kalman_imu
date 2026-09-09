#!/usr/bin/env python3
"""Live plot of the ESP32-C3 attitude EKF's UART telemetry.

Reads the binary telemetry packets emitted by main/uart_debug.c off the
board's console UART (shared with idf.py monitor / flashing) and plots
orientation, debiased angular rate, gyro bias, and raw accelerometer
readings in real time. Useful for sanity-checking and tuning the EKF
(see docs/attitude_ekf_design.md) against the sensor readings driving it.

Packet layout (must match main/uart_debug.c's telemetry_packet_t --
update both together if the firmware's layout changes):
    uint8_t  sync[2]         0xAA, 0x55
    uint32_t seq
    uint32_t timestamp_us
    float    q[4]            orientation quaternion, w,x,y,z
    float    angular_rate[3] rad/s, debiased
    float    gyro_bias[3]    rad/s
    float    accel_body[3]   g
    uint16_t crc             CRC-16/XMODEM (poly 0x1021, init 0x0000) over
                              the bytes from seq..accel_body[2]

Since the board's console UART also carries ESP_LOG text output, this
script resyncs on the 0xAA 0x55 marker and validates the CRC, so log
lines interleaved on the wire are simply skipped over rather than
misparsed. As a second line of defense (a corrupted packet can still
coincidentally pass a 16-bit CRC, just far less often than the 8-bit
checksum this used to use), parsed samples are also rejected if any
field is outside physically plausible bounds -- see PLAUSIBLE_* below.

Usage:
    Edit SERIAL_PORT below to match your board, then just run the file
    (e.g. click "Run" in your editor) -- no command-line arguments needed.
    CLI flags are still available if you'd rather use those instead.
"""

import argparse
import queue
import struct
import sys
import threading
from collections import deque

import serial

# Edit these to match your setup, then run the file directly.
SERIAL_PORT = "/dev/ttyACM1"  # e.g. "COM5" on Windows
BAUD_RATE = 921600            # must match CONFIG_ESP_CONSOLE_UART_BAUDRATE
WINDOW_SECONDS = 10.0         # seconds of history shown on screen

PACKET_FORMAT = "<2sII4f3f3f3fH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SYNC_BYTES = b"\xaa\x55"

# Reject a parsed sample if any field falls outside these bounds -- catches
# the rare packet that coincidentally passes the CRC despite being garbled
# (bit errors on the wire), without needing the CRC to be perfect.
PLAUSIBLE_ACCEL_G = 4.0        # sensor is configured well within this FS range
PLAUSIBLE_RATE_RAD_S = 35.0    # ~2000 dps, the ICM-20948's max gyro FS range
PLAUSIBLE_QUAT_NORM = (0.9, 1.1)


def crc16_xmodem(data: bytes) -> int:
    """Must match main/uart_debug.c's crc16_xmodem exactly (poly 0x1021, init 0x0000)."""
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def is_plausible(qw, qx, qy, qz, gx, gy, gz, ax, ay, az) -> bool:
    qnorm = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
    if not (PLAUSIBLE_QUAT_NORM[0] <= qnorm <= PLAUSIBLE_QUAT_NORM[1]):
        return False
    if max(abs(gx), abs(gy), abs(gz)) > PLAUSIBLE_RATE_RAD_S:
        return False
    if max(abs(ax), abs(ay), abs(az)) > PLAUSIBLE_ACCEL_G:
        return False
    return True


def read_packets(ser: serial.Serial):
    """Generator yielding parsed telemetry dicts, resyncing on bad frames."""
    buf = bytearray()
    while True:
        buf.extend(ser.read(max(1, ser.in_waiting)))

        while True:
            sync_idx = buf.find(SYNC_BYTES)
            if sync_idx < 0:
                # No sync marker at all yet; keep only a short tail in case
                # a sync byte is split across reads.
                if len(buf) > 1:
                    del buf[:-1]
                break
            if sync_idx > 0:
                del buf[:sync_idx]
            if len(buf) < PACKET_SIZE:
                break

            frame = bytes(buf[:PACKET_SIZE])
            payload = frame[len(SYNC_BYTES) : -2]  # seq..accel_body[2], i.e. everything but sync+crc
            (frame_crc,) = struct.unpack("<H", frame[-2:])
            if crc16_xmodem(payload) != frame_crc:
                # Not a real packet (likely log text that happened to
                # contain 0xAA 0x55) -- drop the sync bytes and rescan.
                del buf[:2]
                continue

            del buf[:PACKET_SIZE]
            (
                _sync,
                seq,
                timestamp_us,
                qw, qx, qy, qz,
                gx, gy, gz,
                bx, by, bz,
                ax, ay, az,
                _crc,
            ) = struct.unpack(PACKET_FORMAT, frame)

            if not is_plausible(qw, qx, qy, qz, gx, gy, gz, ax, ay, az):
                # Passed the CRC by coincidence despite being garbled --
                # drop this one sample rather than let it wreck autoscale.
                continue

            yield {
                "seq": seq,
                "t": timestamp_us / 1e6,
                "q": (qw, qx, qy, qz),
                "angular_rate": (gx, gy, gz),
                "gyro_bias": (bx, by, bz),
                "accel": (ax, ay, az),
            }


def reader_loop(ser, out_queue, stop_event):
    """Runs in a background thread: the only thing that touches `ser`.

    Keeping serial I/O off the GUI thread means matplotlib's event loop
    never blocks waiting on the board (e.g. while it's still calibrating
    and hasn't sent a packet yet) -- a stalled read here can't freeze the
    plot window into looking hung.
    """
    for pkt in read_packets(ser):
        if stop_event.is_set():
            return
        try:
            out_queue.put_nowait(pkt)
        except queue.Full:
            pass  # GUI thread is behind; drop the sample rather than block or grow unbounded


def quat_to_euler_deg(q):
    import math

    qw, qx, qy, qz = q
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT, help="Serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--window", type=float, default=WINDOW_SECONDS, help="Seconds of history to show")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading telemetry from {args.port} @ {args.baud} baud")

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    max_samples = int(args.window * 200)  # generous upper bound on sample rate
    t_hist = deque(maxlen=max_samples)
    euler_hist = [deque(maxlen=max_samples) for _ in range(3)]
    rate_hist = [deque(maxlen=max_samples) for _ in range(3)]
    bias_hist = [deque(maxlen=max_samples) for _ in range(3)]
    accel_hist = [deque(maxlen=max_samples) for _ in range(3)]

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 10))
    ax_euler, ax_rate, ax_bias, ax_accel = axes
    euler_labels = ["roll", "pitch", "yaw"]
    axis_labels = ["x", "y", "z"]

    euler_lines = [ax_euler.plot([], [], label=euler_labels[i])[0] for i in range(3)]
    ax_euler.set_ylabel("orientation (deg)")
    ax_euler.legend(loc="upper left")
    # Headroom so roll/pitch/yaw don't sit flush against the plot edges when
    # autoscale fits all three lines' full range each frame.
    ax_euler.margins(y=0.15)

    rate_lines = [ax_rate.plot([], [], label=axis_labels[i])[0] for i in range(3)]
    ax_rate.set_ylabel("angular rate (deg/s)")
    ax_rate.legend(loc="upper left")

    bias_lines = [ax_bias.plot([], [], label=axis_labels[i])[0] for i in range(3)]
    ax_bias.set_ylabel("gyro bias (deg/s)")
    ax_bias.legend(loc="upper left")

    accel_lines = [ax_accel.plot([], [], label=axis_labels[i])[0] for i in range(3)]
    ax_accel.set_ylabel("accel (g)")
    ax_accel.set_xlabel("time (s)")
    ax_accel.legend(loc="upper left")

    data_queue = queue.Queue(maxsize=2000)
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_loop, args=(ser, data_queue, stop_event), daemon=True)
    reader.start()

    def update(_frame):
        # Drain whatever the reader thread has queued since the last
        # redraw; never touches the serial port directly, so this can't
        # block the GUI event loop.
        while True:
            try:
                pkt = data_queue.get_nowait()
            except queue.Empty:
                break
            t_hist.append(pkt["t"])
            for i, v in enumerate(quat_to_euler_deg(pkt["q"])):
                euler_hist[i].append(v)
            for i, v in enumerate(pkt["angular_rate"]):
                rate_hist[i].append(v * 57.29578)  # rad/s -> deg/s
            for i, v in enumerate(pkt["gyro_bias"]):
                bias_hist[i].append(v * 57.29578)
            for i, v in enumerate(pkt["accel"]):
                accel_hist[i].append(v)

        if not t_hist:
            return euler_lines + rate_lines + bias_lines + accel_lines

        t0 = t_hist[0]
        xs = [t - t0 for t in t_hist]

        for i in range(3):
            euler_lines[i].set_data(xs, euler_hist[i])
            rate_lines[i].set_data(xs, rate_hist[i])
            bias_lines[i].set_data(xs, bias_hist[i])
            accel_lines[i].set_data(xs, accel_hist[i])

        # Minimum y-span per subplot (deg, deg/s, deg/s, g) so that when a
        # signal is nearly flat (e.g. holding still), autoscale doesn't zoom
        # in on noise-floor jitter and make it look like wild oscillation.
        min_spans = (2.0, 5.0, 1.0, 0.1)
        for ax, min_span in zip(axes, min_spans):
            # Re-enable autoscale each frame: the min-span clamp below calls
            # set_ylim(), which matplotlib treats as an implicit "user locked
            # this axis" and disables autoscale_on -- without resetting it
            # here, the axis would get stuck at whatever range it was
            # clamped to the first time the signal went flat.
            ax.set_autoscaley_on(True)
            ax.relim()
            ax.autoscale_view()
            lo, hi = ax.get_ylim()
            span = hi - lo
            if span < min_span:
                mid = (hi + lo) / 2.0
                ax.set_ylim(mid - min_span / 2.0, mid + min_span / 2.0)

        return euler_lines + rate_lines + bias_lines + accel_lines

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
