"""Shared JSON-lines telemetry model for both USB connection modes."""

from __future__ import annotations

import json
import math
import time
from typing import Any


class BaroTracker:
    """Display-only barometric altitude with a calibratable baseline pressure."""

    STANDARD_SEA_LEVEL_PA = 101325.0
    EXPONENT = 1.0 / 5.255

    def __init__(self) -> None:
        self.baseline_pa: float = self.STANDARD_SEA_LEVEL_PA
        self.calibrated: bool = False

    def calibrate(self, pressure_pa: float) -> None:
        if pressure_pa and pressure_pa > 30000.0:
            self.baseline_pa = float(pressure_pa)
            self.calibrated = True

    def altitude_m(self, pressure_pa: float) -> float:
        if not pressure_pa or pressure_pa <= 0.0:
            return 0.0
        ratio = float(pressure_pa) / self.baseline_pa
        return 44330.0 * (1.0 - ratio ** self.EXPONENT)


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
        self.command: dict[str, Any] = {}
        self.rc_sniff: dict[str, Any] = {}
        self.radio_test: dict[str, Any] = {}
        self.last_sniff_at: float = 0.0
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
        elif kind == "rc_sniff":
            pot1 = int(packet.get("pot1", 0) or 0)
            self.rc_sniff = {
                "valid": True,
                "armed": bool(packet.get("armed", False)),
                "throttle_us": packet.get("throttle_us", 0),
                "aileron_us": packet.get("aileron_us", 0),
                "elevator_us": packet.get("elevator_us", 0),
                "aux1": pot1 & 0x03FF,
                "trim_flags": (pot1 >> 12) & 0x0F,
                "throttle_boost": packet.get("pot2", 0),
                "throttle_mode": packet.get("switch2", 0),
                "packets": packet.get("seq", 0),
            }
            self.last_sniff_at = self.updated_at
        elif kind == "radio_test":
            self.radio_test = dict(packet)
        elif kind == "command":
            self.command.update({
                "op": packet.get("op"),
                "seq": packet.get("cmd_seq"),
                "status": packet.get("status"),
            })
        elif kind == "command_ack":
            self.command.update({
                "ack_op": packet.get("op"),
                "ack_seq": packet.get("cmd_seq"),
                "ack_result": packet.get("result", 0),
                "mission_state": packet.get("mission_state", 0),
                "ack_at": self.updated_at,
            })
        else:
            raise ValueError(f"unknown ground-station packet kind: {kind!r}")
        return packet


class AttitudeEstimator:
    """Display-only complementary filter with a stationary gyro/level calibration."""

    CALIBRATION_SAMPLES = 25

    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.ready = False
        self.last_time = 0.0
        self.start_calibration()

    def start_calibration(self) -> None:
        self.ready = False
        self.calibration_count = 0
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.gyro_sum = [0.0, 0.0, 0.0]
        self.roll_sin_sum = 0.0
        self.roll_cos_sum = 0.0
        self.pitch_sin_sum = 0.0
        self.pitch_cos_sum = 0.0
        self.roll_zero = 0.0
        self.pitch_zero = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = 0.0

    @property
    def calibration_status(self) -> str:
        if self.ready:
            return "CALIBRATED · DISPLAY ONLY"
        return f"CALIBRATING {self.calibration_count}/{self.CALIBRATION_SAMPLES} · KEEP LEVEL/STILL"

    @staticmethod
    def _wrap_degrees(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0

    def update(self, imu: dict[str, Any]) -> tuple[float, float, float, bool]:
        now = time.monotonic()
        accel = imu.get("accel_mg", [0, 0, 0])
        gyro = imu.get("gyro_cdeg_s", [0, 0, 0])
        mag = imu.get("mag_counts", [0, 0, 0])
        if not imu.get("ok") or len(accel) < 3 or len(gyro) < 3:
            return self.roll, self.pitch, self.heading, False

        ax, ay, az = (float(value) / 1000.0 for value in accel[:3])
        gyro_dps = [float(value) / 100.0 for value in gyro[:3]]
        accel_magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_magnitude = math.sqrt(sum(value * value for value in gyro_dps))
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        if not self.ready:
            # Only use samples that look stationary: roughly 1 g and low gyro rate.
            if 0.75 <= accel_magnitude <= 1.25 and gyro_magnitude <= 8.0:
                self.calibration_count += 1
                for index in range(3):
                    self.gyro_sum[index] += gyro_dps[index]
                self.roll_sin_sum += math.sin(math.radians(roll_acc))
                self.roll_cos_sum += math.cos(math.radians(roll_acc))
                self.pitch_sin_sum += math.sin(math.radians(pitch_acc))
                self.pitch_cos_sum += math.cos(math.radians(pitch_acc))

            if self.calibration_count >= self.CALIBRATION_SAMPLES:
                count = float(self.calibration_count)
                self.gyro_bias = [value / count for value in self.gyro_sum]
                self.roll_zero = math.degrees(math.atan2(self.roll_sin_sum, self.roll_cos_sum))
                self.pitch_zero = math.degrees(math.atan2(self.pitch_sin_sum, self.pitch_cos_sum))
                self.roll = 0.0
                self.pitch = 0.0
                self.ready = True
                self.last_time = now
            return self.roll, self.pitch, self.heading, self.ready

        dt = min(max(now - self.last_time, 0.0), 0.25)
        corrected_roll = gyro_dps[0] - self.gyro_bias[0]
        corrected_pitch = gyro_dps[1] - self.gyro_bias[1]
        roll_reference = self._wrap_degrees(roll_acc - self.roll_zero)
        pitch_reference = self._wrap_degrees(pitch_acc - self.pitch_zero)
        self.roll = 0.98 * (self.roll + corrected_roll * dt) + 0.02 * roll_reference
        self.pitch = 0.98 * (self.pitch + corrected_pitch * dt) + 0.02 * pitch_reference

        if imu.get("mag_ok") and len(mag) >= 2:
            mx, my = float(mag[0]), float(mag[1])
            self.heading = math.degrees(math.atan2(-my, mx)) % 360.0
        self.last_time = now
        return self.roll, self.pitch, self.heading, True
