from __future__ import annotations

import json
import math
import sys
import time
from collections import deque

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtGui import QColor, QDesktopServices, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from telemetry import AttitudeEstimator, TelemetryStore


class StatusTile(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setObjectName("statusTile")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        self.title = QLabel(title.upper())
        self.title.setObjectName("tileTitle")
        self.value = QLabel("WAITING")
        self.value.setObjectName("tileValue")
        self.detail = QLabel("No data")
        self.detail.setObjectName("tileDetail")
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.detail)
        self.set_state("WAITING", "No data", "#65758b")

    def set_state(self, value: str, detail: str, color: str) -> None:
        self.value.setText(value)
        self.detail.setText(detail)
        self.value.setStyleSheet(f"color: {color};")


class ArtificialHorizon(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.attitude_ok = False
        self.setMinimumSize(330, 260)

    def set_attitude(self, roll: float, pitch: float, heading: float, valid: bool) -> None:
        self.roll, self.pitch, self.heading, self.attitude_ok = roll, pitch, heading, valid
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#111a27"))
        center = self.rect().center()
        radius = min(self.width() * 0.36, self.height() * 0.36)
        painter.save()
        painter.translate(center)
        painter.setClipRect(-radius, -radius, radius * 2, radius * 2)
        painter.rotate(-self.roll)
        pitch_shift = max(-35.0, min(35.0, self.pitch)) * 2.0
        painter.translate(0, pitch_shift)
        painter.fillRect(-radius * 2, -radius * 2, radius * 4, radius * 2, QColor("#2879ae"))
        painter.fillRect(-radius * 2, 0, radius * 4, radius * 2, QColor("#8b6242"))
        painter.setPen(QPen(QColor("#f5f5f5"), 2))
        painter.drawLine(int(-radius * 2), 0, int(radius * 2), 0)
        for mark in (-20, -10, 10, 20):
            y = int(-mark * 2)
            painter.drawLine(-24, y, 24, y)
        painter.restore()

        painter.setPen(QPen(QColor("#f5c451"), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(center.x() - 45, center.y(), center.x() - 12, center.y())
        painter.drawLine(center.x() + 12, center.y(), center.x() + 45, center.y())
        painter.drawEllipse(center, 5, 5)
        painter.setPen(QColor("#e7edf5"))
        painter.setFont(QFont("Sans Serif", 10, QFont.Weight.DemiBold))
        painter.drawText(12, 24, "ROLL / PITCH")
        painter.setFont(QFont("Monospace", 10))
        state = "WAITING FOR IMU" if not self.attitude_ok else "APPROX. / UNCALIBRATED"
        painter.drawText(12, self.height() - 14,
                         f"{state}   R {self.roll:+.1f}°  P {self.pitch:+.1f}°  MAG {self.heading:05.1f}°")


class TransmitterView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.rc: dict = {}
        self.setMinimumSize(500, 350)

    @staticmethod
    def _stick_point(value: int, center: int, span: int) -> float:
        return max(-1.0, min(1.0, (value - center) / span))

    def set_rc(self, rc: dict) -> None:
        self.rc = dict(rc)
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#111a27"))
        painter.setPen(QColor("#e7edf5"))
        painter.setFont(QFont("Sans Serif", 11, QFont.Weight.DemiBold))
        painter.drawText(18, 26, "LIVE TRANSMITTER INPUTS (RECEIVED BY AIRCRAFT)")

        width = self.width()
        cy = int(self.height() * 0.51)
        lx, rx = int(width * 0.29), int(width * 0.70)
        radius = 66
        for x, label in ((lx, "THROTTLE"), (rx, "ROLL / PITCH")):
            painter.setPen(QPen(QColor("#63758d"), 2))
            painter.setBrush(QColor("#1a293a"))
            painter.drawEllipse(x - radius, cy - radius, radius * 2, radius * 2)
            painter.setPen(QPen(QColor("#3b4b61"), 1))
            painter.drawLine(x - radius, cy, x + radius, cy)
            painter.drawLine(x, cy - radius, x, cy + radius)
            painter.setPen(QColor("#9aaac0"))
            painter.drawText(x - 50, cy + radius + 26, label)

        throttle = int(self.rc.get("throttle_us", 1000) or 1000)
        roll = int(self.rc.get("aileron_us", 1500) or 1500)
        pitch = int(self.rc.get("elevator_us", 1500) or 1500)
        ty = cy + int((1.0 - max(0.0, min(1.0, (throttle - 1000) / 1000))) * radius * 0.8 - radius * 0.4)
        px = rx + int(self._stick_point(roll, 1500, 500) * radius * 0.8)
        py = cy - int(self._stick_point(pitch, 1500, 500) * radius * 0.8)
        for x, y in ((lx, ty), (px, py)):
            painter.setPen(QPen(QColor("#f5c451"), 2))
            painter.setBrush(QColor("#f5c451"))
            painter.drawEllipse(x - 9, y - 9, 18, 18)

        armed = bool(self.rc.get("armed", False))
        mode_value = self.rc.get("throttle_mode", "—")
        mode_label = {0: "CRUISE (0)", 1: "CRUISE (1)", 2: "REFLEX (2)"}.get(mode_value, str(mode_value))
        trims = int(self.rc.get("trim_flags", 0) or 0)
        trim_names = ("ELEV ↑", "ELEV ↓", "AIL →", "AIL ←")
        active_trims = [name for bit, name in enumerate(trim_names) if trims & (1 << bit)]
        if not active_trims:
            active_trims = ["none"]
        status = f"ARM: {'ARMED' if armed else 'DISARMED'}     THROTTLE MODE: {mode_label}"
        painter.setPen(QColor("#c4d0df"))
        painter.setFont(QFont("Monospace", 10))
        painter.drawText(18, self.height() - 42, status)
        painter.drawText(18, self.height() - 20, "TRIM BUTTONS: " + ", ".join(active_trims))


class AxisPlot(QWidget):
    COLORS = (QColor("#f06c75"), QColor("#59c6a5"), QColor("#70a5ff"))

    def __init__(self, title: str, unit: str, scale: float) -> None:
        super().__init__()
        self.title = title
        self.unit = unit
        self.scale = scale
        self.history = [deque(maxlen=180) for _ in range(3)]
        self.setMinimumHeight(150)

    def add_values(self, values: list, valid: bool) -> None:
        if not valid or len(values) < 3:
            return
        for series, value in zip(self.history, values[:3]):
            series.append(float(value))
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#111a27"))
        p.setPen(QColor("#e7edf5"))
        p.setFont(QFont("Sans Serif", 10, QFont.Weight.DemiBold))
        p.drawText(10, 20, f"{self.title} ({self.unit})")
        plot = self.rect().adjusted(12, 30, -12, -18)
        p.setPen(QPen(QColor("#2c3b50"), 1))
        for n in range(1, 4):
            y = plot.top() + n * plot.height() // 4
            p.drawLine(plot.left(), y, plot.right(), y)
        for series, color in zip(self.history, self.COLORS):
            if len(series) < 2:
                continue
            p.setPen(QPen(color, 1.5))
            points = list(series)
            for i in range(1, len(points)):
                x1 = plot.left() + (i - 1) * plot.width() // (len(points) - 1)
                x2 = plot.left() + i * plot.width() // (len(points) - 1)
                y1 = plot.center().y() - int(points[i - 1] / self.scale * plot.height() / 2)
                y2 = plot.center().y() - int(points[i] / self.scale * plot.height() / 2)
                p.drawLine(x1, y1, x2, y2)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("NanoLink — Flight Debug Monitor")
        self.resize(1200, 850)
        self.store = TelemetryStore()
        self.attitude = AttitudeEstimator()
        self.port: serial.Serial | None = None
        self.rx_buffer = bytearray()
        self.event_count = 0

        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(12, 12, 12, 10)
        root_layout.setSpacing(10)
        root_layout.addLayout(self._make_connection_bar())
        root_layout.addWidget(self._make_status_grid())
        self.tabs = QTabWidget()
        self.tabs.addTab(self._make_overview_tab(), "Overview")
        self.tabs.addTab(self._make_controls_tab(), "Transmitter")
        self.tabs.addTab(self._make_packets_tab(), "Packets")
        root_layout.addWidget(self.tabs, 1)
        self.statusBar().showMessage("Choose a source mode and USB serial port")

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._read_serial)
        self.poll_timer.start(20)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_health)
        self.ui_timer.start(250)
        self._refresh_ports()
        self._apply_style()

    def _make_connection_bar(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Direct USB · aircraft F401", "aircraft_usb")
        self.mode_combo.addItem("Ground station · F401 + nRF24", "ground_station")
        self.mode_combo.currentIndexChanged.connect(lambda _index: self._mode_changed())
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh ports")
        self.refresh_button.clicked.connect(lambda _checked=False: self._refresh_ports())
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(lambda _checked=False: self._toggle_connection())
        row.addWidget(QLabel("Source"))
        row.addWidget(self.mode_combo, 2)
        row.addWidget(QLabel("USB serial"))
        row.addWidget(self.port_combo, 2)
        row.addWidget(self.refresh_button)
        row.addWidget(self.connect_button)
        return row

    def _make_status_grid(self) -> QWidget:
        panel = QWidget()
        grid = QGridLayout(panel)
        grid.setContentsMargins(0, 0, 0, 0)
        self.tiles = {
            "serial": StatusTile("USB serial"),
            "radio": StatusTile("nRF24 link"),
            "rc": StatusTile("RC input"),
            "imu": StatusTile("MPU9250"),
            "baro": StatusTile("BMP280"),
            "gps": StatusTile("NEO-8M GPS"),
        }
        for col, key in enumerate(self.tiles):
            grid.addWidget(self.tiles[key], 0, col)
        return panel

    def _make_overview_tab(self) -> QWidget:
        page = QWidget()
        layout = QGridLayout(page)
        self.horizon = ArtificialHorizon()
        self.gps_text = QLabel("Waiting for GPS data")
        self.gps_text.setObjectName("infoPanel")
        self.gps_text.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.gps_text.setMinimumWidth(250)
        self.map_button = QPushButton("Open fix in OpenStreetMap")
        self.map_button.clicked.connect(self._open_map)
        gps_panel = QVBoxLayout()
        gps_panel.addWidget(self.gps_text, 1)
        gps_panel.addWidget(self.map_button)
        self.map_button.setEnabled(False)
        gps_widget = QWidget()
        gps_widget.setLayout(gps_panel)
        self.accel_plot = AxisPlot("Acceleration", "mg", 2000)
        self.gyro_plot = AxisPlot("Angular rate", "centidegrees/s", 20000)
        layout.addWidget(self.horizon, 0, 0)
        layout.addWidget(gps_widget, 0, 1)
        layout.addWidget(self.accel_plot, 1, 0)
        layout.addWidget(self.gyro_plot, 1, 1)
        return page

    def _make_controls_tab(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)
        self.transmitter_view = TransmitterView()
        layout.addWidget(self.transmitter_view, 2)
        self.channel_table = QTableWidget(13, 2)
        self.channel_table.setHorizontalHeaderLabels(["Input", "Received value"])
        self.channel_table.verticalHeader().setVisible(False)
        self.channel_table.horizontalHeader().setStretchLastSection(True)
        self.channel_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.channel_names = [
            "Throttle", "Aileron", "Elevator", "Arm", "Throttle mode",
            "Auxiliary A5", "Throttle boost A4", "Elevator trim up",
            "Elevator trim down", "Aileron trim right", "Aileron trim left",
            "Left stick X", "Reserved D7/D8",
        ]
        for row, name in enumerate(self.channel_names):
            self.channel_table.setItem(row, 0, QTableWidgetItem(name))
            self.channel_table.setItem(row, 1, QTableWidgetItem("—"))
        layout.addWidget(self.channel_table, 1)
        return page

    def _make_packets_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.packet_summary = QLabel("No packets received")
        self.packet_log = QPlainTextEdit()
        self.packet_log.setReadOnly(True)
        self.packet_log.setMaximumBlockCount(300)
        self.packet_log.setFont(QFont("Monospace", 9))
        layout.addWidget(self.packet_summary)
        layout.addWidget(self.packet_log, 1)
        return page

    def _refresh_ports(self) -> None:
        selected = self.port_combo.currentData()
        self.port_combo.clear()
        for port in list_ports.comports():
            label = f"{port.device} — {port.description or 'USB serial'}"
            self.port_combo.addItem(label, port.device)
        if selected:
            index = self.port_combo.findData(selected)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _toggle_connection(self) -> None:
        if self.port is not None:
            self._disconnect()
            return
        device = self.port_combo.currentData()
        if not device:
            self.statusBar().showMessage("No serial port selected. Connect an F401 board and refresh ports.")
            return
        try:
            self.port = serial.Serial(device, 115200, timeout=0)
        except serial.SerialException as exc:
            self.port = None
            self.tiles["serial"].set_state("OPEN FAILED", device, "#e27676")
            self.statusBar().showMessage(f"Could not open {device}: {exc}")
            return
        self.rx_buffer.clear()
        self.store.reset()
        self.attitude = AttitudeEstimator()
        self.connect_button.setText("Disconnect")
        self.tiles["serial"].set_state("CONNECTED", device, "#55cf9a")
        self.statusBar().showMessage(f"Connected to {device} at 115200 baud")

    def _disconnect(self) -> None:
        if self.port is not None:
            try:
                self.port.close()
            except serial.SerialException:
                pass
        self.port = None
        self.connect_button.setText("Connect")
        self.tiles["serial"].set_state("DISCONNECTED", "No serial input", "#e2aa57")
        self.statusBar().showMessage("Disconnected")

    def _mode_changed(self) -> None:
        mode = self.mode_combo.currentData()
        if mode == "aircraft_usb":
            self.statusBar().showMessage("Direct mode reads local sensors; nRF and RC are intentionally disabled")
        else:
            self.statusBar().showMessage("Ground-station mode reads nRF telemetry and aircraft-reported RC data")

    def _read_serial(self) -> None:
        if self.port is None:
            return
        try:
            count = self.port.in_waiting
            if count:
                self.rx_buffer.extend(self.port.read(count))
            while b"\n" in self.rx_buffer:
                raw, _, remainder = self.rx_buffer.partition(b"\n")
                self.rx_buffer = bytearray(remainder)
                line = raw.decode("utf-8", errors="replace").strip("\r \t")
                if line:
                    self._handle_line(line)
            if len(self.rx_buffer) > 8192:
                self.rx_buffer.clear()
                self.packet_log.appendPlainText("[parser] serial line exceeded 8 KiB; buffer cleared")
        except serial.SerialException as exc:
            self.packet_log.appendPlainText(f"[serial error] {exc}")
            self._disconnect()

    def _handle_line(self, line: str) -> None:
        try:
            packet = self.store.ingest(line)
        except (json.JSONDecodeError, ValueError) as exc:
            self.packet_log.appendPlainText(f"[invalid] {line[:240]} ({exc})")
            return

        expected = self.mode_combo.currentData()
        if packet.get("source") != expected:
            self.statusBar().showMessage(
                f"Source mismatch: selected {expected}, device reports {packet.get('source')}"
            )
        self.event_count += 1
        summary = f"{packet.get('source')} / {packet.get('kind')} seq={packet.get('seq', '—')}"
        self.packet_summary.setText(f"Frames displayed: {self.event_count}   |   Last: {summary}")
        self.packet_log.appendPlainText(json.dumps(packet, ensure_ascii=False, separators=(",", ":")))

        if packet.get("source") == "aircraft_usb" or packet.get("kind") == "sensor":
            self.accel_plot.add_values(self.store.imu.get("accel_mg", []), self.store.imu.get("ok", False))
            self.gyro_plot.add_values(self.store.imu.get("gyro_cdeg_s", []), self.store.imu.get("ok", False))
            roll, pitch, heading, valid = self.attitude.update(self.store.imu)
            self.horizon.set_attitude(roll, pitch, heading, valid)
        self.transmitter_view.set_rc(self.store.rc)
        self._update_channel_table()
        self._update_gps_panel()
        self._refresh_health()

    def _update_channel_table(self) -> None:
        rc = self.store.rc
        if not rc:
            return
        trim_flags = int(rc.get("trim_flags", 0) or 0)
        values = [
            f"{rc.get('throttle_us', '—')} µs",
            f"{rc.get('aileron_us', '—')} µs",
            f"{rc.get('elevator_us', '—')} µs",
            "ARMED" if rc.get("armed") else "DISARMED",
            {0: "CRUISE (0)", 1: "CRUISE (1)", 2: "REFLEX (2)"}.get(rc.get("throttle_mode"), "—"),
            str(rc.get("aux1", "—")),
            str(rc.get("throttle_boost", "—")),
            "PRESSED" if trim_flags & 1 else "released",
            "PRESSED" if trim_flags & 2 else "released",
            "PRESSED" if trim_flags & 4 else "released",
            "PRESSED" if trim_flags & 8 else "released",
            "not transmitted",
            "not assigned",
        ]
        for row, value in enumerate(values):
            self.channel_table.item(row, 1).setText(value)

    def _update_gps_panel(self) -> None:
        gps = self.store.gps
        if not gps:
            self.gps_text.setText("Waiting for GPS data")
            return
        stream = bool(gps.get("stream", False))
        fix = bool(gps.get("fix", False))
        sat_count = int(gps.get("satellites", 0) or 0)
        lat = int(gps.get("lat_e7", 0) or 0) / 1e7
        lon = int(gps.get("lon_e7", 0) or 0) / 1e7
        altitude = int(gps.get("alt_mm", 0) or 0) / 1000.0
        speed = int(gps.get("speed_cms", 0) or 0) / 100.0
        hdop = int(gps.get("hdop_centi", 0) or 0) / 100.0
        state = "FIX" if fix else ("NMEA stream; searching" if stream else "No GPS data")
        self.gps_text.setText(
            f"GPS STATUS: {state}\n\n"
            f"Satellites: {sat_count}\n"
            f"Fix type: {gps.get('fix_type', '—')}\n"
            f"HDOP: {hdop:.2f}\n\n"
            f"Latitude: {lat:.7f}\n"
            f"Longitude: {lon:.7f}\n"
            f"GPS altitude: {altitude:.1f} m\n"
            f"Ground speed: {speed:.2f} m/s\n"
            f"Course: {int(gps.get('course_centi_deg', 0) or 0) / 100.0:.1f}°"
        )
        self.map_button.setEnabled(fix)

    def _open_map(self) -> None:
        gps = self.store.gps
        if not gps or not gps.get("fix"):
            return
        lat = int(gps.get("lat_e7", 0)) / 1e7
        lon = int(gps.get("lon_e7", 0)) / 1e7
        url = QUrl(f"https://www.openstreetmap.org/?mlat={lat}&mlon={lon}#map=16/{lat}/{lon}")
        QDesktopServices.openUrl(url)

    def _refresh_health(self) -> None:
        if self.port is None:
            self.tiles["serial"].set_state("DISCONNECTED", "Select a USB serial port", "#e2aa57")
        elif self.store.updated_at and time.monotonic() - self.store.updated_at > 2.0:
            self.tiles["serial"].set_state("CONNECTED", "No recent JSON telemetry", "#e2aa57")

        if self.store.source == "aircraft_usb":
            self.tiles["radio"].set_state("DISABLED", "USB-only sensor diagnostic", "#7ca7d8")
            self.tiles["rc"].set_state("N/A", "RC is not read in USB-only mode", "#7ca7d8")
        elif self.store.source == "ground_station":
            radio = self.store.radio
            if "radio_ready" in radio:
                ready = bool(radio.get("radio_ready")) and bool(radio.get("radio_chip_ok"))
                age = int(radio.get("telemetry_age_ms", 65535) or 65535)
                live = ready and age < 1500
                air_tx = ""
                if "air_tx_ok" in radio or "air_tx_failed" in radio:
                    air_tx = f" · aircraft TX {radio.get('air_tx_ok', 0)}/{radio.get('air_tx_failed', 0)} failed"
                self.tiles["radio"].set_state(
                    "LIVE" if live else ("RADIO OK · NO AIR DATA" if ready else "RADIO ERROR"),
                    f"RX {radio.get('rx_packets', 0)} · lost {radio.get('lost_packets', 0)} · auto-ACK {radio.get('auto_ack_count', 0)} · age {age} ms{air_tx}",
                    "#55cf9a" if live else "#e2aa57",
                )
            rc = self.store.rc
            rc_age = int(rc.get("rc_age_ms", 65535) or 65535)
            since_rc = (time.monotonic() - self.store.last_rc_packet_at) * 1000 if self.store.last_rc_packet_at else 65535
            valid = bool(rc.get("valid")) and rc_age + since_rc < 500
            self.tiles["rc"].set_state(
                "LIVE" if valid else "NO RC DATA",
                f"Packets {rc.get('rc_packets', 0)} · age {rc.get('rc_age_ms', '—')} ms",
                "#55cf9a" if valid else "#e2aa57",
            )

        imu = self.store.imu
        if imu.get("ok"):
            who_am_i = int(imu.get("who_am_i", 0) or 0)
            if who_am_i == 0x70:
                imu_detail = "MPU6500-compatible · accel/gyro only"
            elif self.store.source == "aircraft_usb" and who_am_i:
                imu_detail = f"WHO_AM_I 0x{who_am_i:02X} · accel/gyro streaming"
            elif not imu.get("mag_ok", True):
                imu_detail = "accelerometer/gyro OK · magnetometer unavailable"
            else:
                imu_detail = "accelerometer, gyro, and magnetometer streaming"
        elif self.store.source == "aircraft_usb":
            addresses = ", ".join(f"0x{int(addr):02X}" for addr in imu.get("i2c_addresses", [])) or "none"
            who = int(imu.get("who_am_i", 0) or 0)
            addr = int(imu.get("i2c_addr", 0) or 0)
            id_rc = imu.get("id_read_rc", "—")
            imu_detail = f"I2C {addresses} · addr 0x{addr:02X} · ID 0x{who:02X} · read {id_rc}"
        else:
            imu_detail = "Check sensor telemetry"
        self.tiles["imu"].set_state(
            "OK" if imu.get("ok") else "NO DATA",
            imu_detail,
            "#55cf9a" if imu.get("ok") else "#e2aa57",
        )
        self.tiles["baro"].set_state(
            "OK" if self.store.baro.get("ok") else "NO DATA",
            "pressure and temperature" if self.store.baro.get("ok") else "Check BMP280 I2C",
            "#55cf9a" if self.store.baro.get("ok") else "#e2aa57",
        )
        gps = self.store.gps
        telemetry_stale = (
            self.store.source == "ground_station"
            and self.store.last_rf_packet_at
            and time.monotonic() - self.store.last_rf_packet_at > 1.5
        )
        if telemetry_stale:
            self.tiles["gps"].set_state("STALE", "No recent aircraft packets", "#e27676")
        elif gps.get("fix"):
            self.tiles["gps"].set_state("FIX", f"{gps.get('satellites', 0)} satellites", "#55cf9a")
        elif gps.get("stream"):
            self.tiles["gps"].set_state("SEARCHING", f"{gps.get('satellites', 0)} satellites", "#e2aa57")
        elif gps:
            self.tiles["gps"].set_state("NO DATA", "No NMEA sentences", "#e27676")
        else:
            self.tiles["gps"].set_state("WAITING", "No GPS packet", "#65758b")

    def _apply_style(self) -> None:
        self.setStyleSheet("""
            QWidget { background: #0c131d; color: #e7edf5; font-size: 10pt; }
            QMainWindow { background: #0c131d; }
            QComboBox, QPushButton, QPlainTextEdit, QTableWidget {
                background: #172333; border: 1px solid #33465e; border-radius: 5px;
                padding: 5px; selection-background-color: #365d87;
            }
            QPushButton:hover { background: #22364d; }
            QTabWidget::pane { border: 1px solid #26374c; border-radius: 5px; }
            QTabBar::tab { background: #172333; padding: 8px 16px; margin-right: 3px; }
            QTabBar::tab:selected { background: #29415d; }
            QFrame#statusTile, QLabel#infoPanel {
                background: #141f2d; border: 1px solid #26374c; border-radius: 7px;
            }
            QLabel#tileTitle { color: #9aabc1; font-size: 8pt; font-weight: 600; }
            QLabel#tileValue { font-size: 14pt; font-weight: 700; }
            QLabel#tileDetail { color: #aebdd0; font-size: 8pt; }
            QHeaderView::section { background: #1c2b3d; padding: 5px; border: none; }
        """)


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("NanoLink Debug Monitor")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
