"""Shared JSON-lines telemetry model for both USB connection modes."""

from __future__ import annotations

import json
import math
import time
from typing import Any


class TelemetryStore:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.source: str | None = None
        self.imu: dict[str, Any] = {}
        self.baro: dict[str, Any] = {}
        self.gps: dict[str, Any] = {}
        self.rc: dict[str, Any] = {}
        self.radio: dict[str, Any] = {}
        self.last_packet: dict[str, Any] = {}
        self.last_line = ""
        self.updated_at = 0.0
        self.last_rf_packet_at = 0.0
        self.last_rc_packet_at = 0.0

    def ingest(self, line: str) -> dict[str, Any]:
        packet = json.loads(line)
        if not isinstance(packet, dict):
            raise ValueError("telemetry line must be a JSON object")

        self.last_line = line.strip()
        self.last_packet = packet
        self.source = packet.get("source")
        self.updated_at = time.monotonic()

        if self.source == "aircraft_usb":
            self.imu = dict(packet.get("imu", {}))
            self.imu["i2c_addresses"] = packet.get("i2c_addresses", [])
            self.baro = dict(packet.get("baro", {}))
            self.gps = dict(packet.get("gps", {}))
            self.rc = dict(packet.get("rc", {}))
            self.radio = dict(packet.get("nrf", {}))
            return packet

        if self.source != "ground_station":
            raise ValueError(f"unknown telemetry source: {self.source!r}")

        kind = packet.get("kind")
        if kind in {"sensor", "control", "gps"}:
            self.last_rf_packet_at = self.updated_at
            self.radio.update({
                "rx_packets": packet.get("rx_packets", self.radio.get("rx_packets", 0)),
                "lost_packets": packet.get("lost_packets", self.radio.get("lost_packets", 0)),
                "auto_ack_count": packet.get("auto_ack_count", self.radio.get("auto_ack_count", 0)),
            })
        if kind == "link":
            self.radio.update(packet)
        elif kind == "sensor":
            self.imu = {
                "ok": packet.get("imu_ok", False),
                "mag_ok": packet.get("mag_ok", False),
                "accel_mg": packet.get("accel_mg", [0, 0, 0]),
                "gyro_cdeg_s": packet.get("gyro_cdeg_s", [0, 0, 0]),
                "mag_counts": packet.get("mag_counts", [0, 0, 0]),
            }
            self.baro = {
                "ok": packet.get("baro_ok", False),
                "pressure_pa": packet.get("pressure_pa", 0),
                "temperature_centi_c": packet.get("temperature_centi_c", 0),
            }
            self.radio["last_seq"] = packet.get("seq")
        elif kind == "control":
            self.last_rc_packet_at = self.updated_at
            self.rc = {
                "valid": packet.get("rc_valid", False),
                "armed": packet.get("armed", False),
                "throttle_us": packet.get("throttle_us", 0),
                "aileron_us": packet.get("aileron_us", 0),
                "elevator_us": packet.get("elevator_us", 0),
                "aux1": packet.get("aux1", 0),
                "trim_flags": packet.get("trim_flags", 0),
                "throttle_boost": packet.get("throttle_boost", 0),
                "throttle_mode": packet.get("throttle_mode", 0),
                "rc_age_ms": packet.get("rc_age_ms", 65535),
                "rc_packets": packet.get("rc_packets", 0),
            }
            self.radio.update({
                "air_tx_ok": packet.get("air_tx_ok", 0),
                "air_tx_failed": packet.get("air_tx_failed", 0),
                "last_control_seq": packet.get("seq"),
            })
            self.gps.update({
                "stream": packet.get("gps_stream", False),
                "fix": packet.get("gps_fix", False),
            })
        elif kind == "gps":
            self.gps = {
                "stream": packet.get("stream_ok", False),
                "fix": packet.get("fix_ok", False),
                "fix_type": packet.get("fix_type", 0),
                "satellites": packet.get("satellites", 0),
                "hdop_centi": packet.get("hdop_centi", 0),
                "lat_e7": packet.get("lat_e7", 0),
                "lon_e7": packet.get("lon_e7", 0),
                "alt_mm": packet.get("alt_mm", 0),
                "speed_cms": packet.get("speed_cms", 0),
                "course_centi_deg": packet.get("course_centi_deg", 0),
            }
            self.radio.update({"last_gps_seq": packet.get("seq")})
        else:
            raise ValueError(f"unknown ground-station packet kind: {kind!r}")
        return packet


class AttitudeEstimator:
    """Small display-only complementary filter; not flight-control software."""

    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.ready = False
        self.last_time = 0.0

    def update(self, imu: dict[str, Any]) -> tuple[float, float, float, bool]:
        now = time.monotonic()
        accel = imu.get("accel_mg", [0, 0, 0])
        gyro = imu.get("gyro_cdeg_s", [0, 0, 0])
        mag = imu.get("mag_counts", [0, 0, 0])
        if not imu.get("ok") or len(accel) < 3 or len(gyro) < 3:
            return self.roll, self.pitch, self.heading, False

        ax, ay, az = (float(value) / 1000.0 for value in accel[:3])
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if not self.ready:
            self.roll, self.pitch = roll_acc, pitch_acc
            self.ready = True
        else:
            dt = min(max(now - self.last_time, 0.0), 0.25)
            gx, gy = float(gyro[0]) / 100.0, float(gyro[1]) / 100.0
            self.roll = 0.98 * (self.roll + gx * dt) + 0.02 * roll_acc
            self.pitch = 0.98 * (self.pitch + gy * dt) + 0.02 * pitch_acc

        if imu.get("mag_ok") and len(mag) >= 2:
            mx, my = float(mag[0]), float(mag[1])
            self.heading = math.degrees(math.atan2(-my, mx)) % 360.0
        self.last_time = now
        return self.roll, self.pitch, self.heading, True
