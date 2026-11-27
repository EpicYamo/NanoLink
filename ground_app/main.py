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

from telemetry import AttitudeEstimator, BaroTracker, TelemetryStore
from mapview import TileMapWidget
from mission import MissionPlan


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
        self.calibration_status = "WAITING FOR IMU"
        self.setMinimumSize(330, 260)

    def set_attitude(self, roll: float, pitch: float, heading: float, valid: bool, status: str) -> None:
        self.roll, self.pitch, self.heading, self.attitude_ok = roll, pitch, heading, valid
        self.calibration_status = status
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
        painter.drawText(12, self.height() - 14,
                         f"{self.calibration_status}   R {self.roll:+.1f}°  P {self.pitch:+.1f}°  MAG {self.heading:05.1f}°")


class TransmitterView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.rc: dict = {}
        self.aileron_center = 1500
        self.elevator_center = 1500
        self.aileron_min_seen = 1850
        self.aileron_max_seen = 1150
        self.elevator_min_seen = 1850
        self.elevator_max_seen = 1150
        self.setMinimumSize(500, 350)

    def set_rc(self, rc: dict) -> None:
        self.rc = dict(rc)
        # Track observed extremes so the dot reaches the circle boundary
        # in every direction once the sticks have visited their limits.
        roll = int(self.rc.get("aileron_us", 1500) or 1500)
        pitch = int(self.rc.get("elevator_us", 1500) or 1500)
        self.aileron_min_seen = min(self.aileron_min_seen, roll)
        self.aileron_max_seen = max(self.aileron_max_seen, roll)
        self.elevator_min_seen = min(self.elevator_min_seen, pitch)
        self.elevator_max_seen = max(self.elevator_max_seen, pitch)
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

        # Normalize each direction independently from the auto-calibrated
        # center to the observed extreme, so the dot reaches the circle
        # boundary once the sticks have visited their limits.
        ty = cy + int((1.0 - max(0.0, min(1.0, (throttle - 1000) / 1000))) * radius - radius)
        if roll < self.aileron_center:
            denom = max(1, self.aileron_center - self.aileron_min_seen)
            px = rx + int(min(1.0, (self.aileron_center - roll) / denom) * radius)
        else:
            denom = max(1, self.aileron_max_seen - self.aileron_center)
            px = rx - int(min(1.0, (roll - self.aileron_center) / denom) * radius)
        if pitch < self.elevator_center:
            denom = max(1, self.elevator_center - self.elevator_min_seen)
            py = cy - int(min(1.0, (self.elevator_center - pitch) / denom) * radius)
        else:
            denom = max(1, self.elevator_max_seen - self.elevator_center)
            py = cy + int(min(1.0, (pitch - self.elevator_center) / denom) * radius)
        # Clamp the roll/pitch dot inside the circle: when both axes hit
        # their extremes simultaneously, the raw position lands on the corner
        # of a square (outside the circle). Scale it back to the boundary.
        dx = px - rx
        dy = py - cy
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > radius:
            scale = radius / dist
            px = rx + int(dx * scale)
            py = cy + int(dy * scale)
        for x, y in ((lx, ty), (px, py)):
            painter.setPen(QPen(QColor("#f5c451"), 2))
            painter.setBrush(QColor("#f5c451"))
            painter.drawEllipse(x - 9, y - 9, 18, 18)

        armed = bool(self.rc.get("armed", False))
        mode_value = self.rc.get("throttle_mode", "—")
        mode_label = {0: "MANUAL (0)", 1: "STABILIZED (1)", 2: "STABILIZED (2)"}.get(mode_value, str(mode_value))
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
        self.baro = BaroTracker()
        self.baro_calibrated = False
        self.mission = MissionPlan()
        self.stick_center_calibrated = False
        self.port: serial.Serial | None = None
        self.rx_buffer = bytearray()
        self.event_count = 0
        self._pending_log: list[str] = []

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
        self.tabs.addTab(self._make_map_tab(), "Map / Mission")
        self.tabs.addTab(self._make_packets_tab(), "Packets")
        root_layout.addWidget(self.tabs, 1)
        self.statusBar().showMessage("Choose a source mode and USB serial port")

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._read_serial)
        self.poll_timer.start(20)
        self.fast_ui_timer = QTimer(self)
        self.fast_ui_timer.timeout.connect(self._refresh_fast_ui)
        self.fast_ui_timer.start(40)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_slow_ui)
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
            "radio": StatusTile("nRF24 / air link"),
            "rc": StatusTile("RC input"),
            "imu": StatusTile("MPU9250"),
            "baro": StatusTile("BMP280"),
            "gps": StatusTile("NEO-8M GPS"),
            "oled": StatusTile("SSD1306 OLED"),
        }
        for col, key in enumerate(self.tiles):
            grid.addWidget(self.tiles[key], 0, col)
        return panel

    def _make_overview_tab(self) -> QWidget:
        page = QWidget()
        layout = QGridLayout(page)
        self.horizon = ArtificialHorizon()
        self.calibrate_button = QPushButton("Calibrate level / gyro · keep aircraft still")
        self.calibrate_button.clicked.connect(self._restart_attitude_calibration)
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
        attitude_panel = QWidget()
        attitude_layout = QVBoxLayout(attitude_panel)
        attitude_layout.setContentsMargins(0, 0, 0, 0)
        attitude_layout.addWidget(self.horizon, 1)
        attitude_layout.addWidget(self.calibrate_button)
        layout.addWidget(attitude_panel, 0, 0)
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
            "Throttle", "Aileron", "Elevator", "Arm", "Flight mode",
            "Auxiliary A5", "Throw rate A4", "Elevator trim up",
            "Elevator trim down", "Aileron trim right", "Aileron trim left",
            "Left stick X", "Reserved D7/D8",
        ]
        for row, name in enumerate(self.channel_names):
            self.channel_table.setItem(row, 0, QTableWidgetItem(name))
            self.channel_table.setItem(row, 1, QTableWidgetItem("—"))
        layout.addWidget(self.channel_table, 1)
        return page

    def _make_map_tab(self) -> QWidget:
        from PySide6.QtWidgets import QCheckBox, QDoubleSpinBox, QSpinBox

        page = QWidget()
        layout = QHBoxLayout(page)
        self.map_widget = TileMapWidget()
        self.map_widget.map_clicked.connect(self._map_clicked)
        layout.addWidget(self.map_widget, 3)

        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(8, 0, 0, 0)

        title = QLabel("Mission planner")
        title.setObjectName("tileValue")
        side_layout.addWidget(title)

        warning = QLabel(
            "PLANNING ONLY — the aircraft cannot fly this mission yet. "
            "Stabilization and a computer→aircraft command link are not implemented."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color: #e2aa57;")
        side_layout.addWidget(warning)

        self.follow_checkbox = QCheckBox("Center map on aircraft fix")
        self.follow_checkbox.setChecked(True)
        self.follow_checkbox.toggled.connect(self._follow_toggled)
        side_layout.addWidget(self.follow_checkbox)

        side_layout.addWidget(QLabel("Waypoint altitude (m)"))
        self.waypoint_alt_spin = QSpinBox()
        self.waypoint_alt_spin.setRange(5, 500)
        self.waypoint_alt_spin.setValue(50)
        side_layout.addWidget(self.waypoint_alt_spin)

        side_layout.addWidget(QLabel("Loiter radius around home (m)"))
        self.loiter_radius_spin = QSpinBox()
        self.loiter_radius_spin.setRange(5, 500)
        self.loiter_radius_spin.setValue(30)
        self.loiter_radius_spin.valueChanged.connect(self._loiter_radius_changed)
        side_layout.addWidget(self.loiter_radius_spin)

        home_button = QPushButton("Set home from aircraft fix")
        home_button.clicked.connect(lambda _checked=False: self._set_home_from_fix())
        side_layout.addWidget(home_button)

        clear_button = QPushButton("Clear waypoint")
        clear_button.clicked.connect(lambda _checked=False: self._clear_waypoint())
        side_layout.addWidget(clear_button)

        send_button = QPushButton("Send mission to aircraft")
        send_button.clicked.connect(lambda _checked=False: self._send_mission_to_aircraft())
        side_layout.addWidget(send_button)

        start_button = QPushButton("Request START (recorded only)")
        start_button.clicked.connect(lambda _checked=False: self._send_command_line("CMD,start_mission"))
        side_layout.addWidget(start_button)

        abort_button = QPushButton("Request ABORT")
        abort_button.clicked.connect(lambda _checked=False: self._send_command_line("CMD,abort_mission"))
        side_layout.addWidget(abort_button)

        rth_button = QPushButton("Request RTH (draft)")
        rth_button.clicked.connect(lambda _checked=False: self._send_command_line("CMD,rth"))
        side_layout.addWidget(rth_button)

        zoom_row = QHBoxLayout()
        zoom_in_button = QPushButton("Zoom +")
        zoom_in_button.clicked.connect(lambda _checked=False: self.map_widget.zoom_in())
        zoom_out_button = QPushButton("Zoom −")
        zoom_out_button.clicked.connect(lambda _checked=False: self.map_widget.zoom_out())
        zoom_row.addWidget(zoom_in_button)
        zoom_row.addWidget(zoom_out_button)
        side_layout.addLayout(zoom_row)

        self.mission_summary = QLabel("Click the map to place the waypoint.")
        self.mission_summary.setWordWrap(True)
        self.mission_summary.setObjectName("infoPanel")
        self.mission_summary.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        side_layout.addWidget(self.mission_summary, 1)

        self.command_status = QLabel("No commands sent yet.")
        self.command_status.setWordWrap(True)
        self.command_status.setStyleSheet("color: #7ca7d8;")
        side_layout.addWidget(self.command_status)

        layout.addWidget(side, 1)
        return page

    _MISSION_STATE_LABELS = {
        0: "no mission on aircraft",
        1: "mission loaded on aircraft",
        2: "start requested — execution NOT implemented",
        3: "RTH requested — execution NOT implemented",
    }

    def _send_command_line(self, line: str) -> None:
        if self.port is None or self.mode_combo.currentData() != "ground_station":
            self.statusBar().showMessage("Connect the ground station to send aircraft commands")
            return
        try:
            self.port.write((line + "\n").encode("ascii"))
        except serial.SerialException as exc:
            self.statusBar().showMessage(f"Serial write failed: {exc}")

    def _send_mission_to_aircraft(self) -> None:
        if not self.mission.home:
            self.statusBar().showMessage("Set home before sending the mission")
            return
        if not self.mission.waypoint:
            self.statusBar().showMessage("Place a waypoint before sending the mission")
            return
        home_lat, home_lon = self.mission.home
        wp_lat, wp_lon, wp_alt = self.mission.waypoint
        lines = [
            f"CMD,set_home,{int(round(home_lat * 1e7))},{int(round(home_lon * 1e7))}",
            f"CMD,set_waypoint,{int(round(wp_lat * 1e7))},{int(round(wp_lon * 1e7))},{int(round(wp_alt * 1000))}",
            f"CMD,set_loiter,{int(round(self.mission.loiter_radius_m))}",
        ]
        for line in lines:
            self._send_command_line(line)
        self.statusBar().showMessage("Mission commands sent — waiting for aircraft acknowledgments")

    def _refresh_command_status(self) -> None:
        command = self.store.command
        if not command:
            self.command_status.setText("No commands sent yet.")
            return
        text = ""
        if "op" in command:
            text += f"Last sent: {command['op']} (seq {command.get('seq', '—')}) — {command.get('status', '—')}"
        if "ack_op" in command:
            state = self._MISSION_STATE_LABELS.get(
                int(command.get("mission_state", 0) or 0), "unknown state"
            )
            result = int(command.get("ack_result", 0) or 0)
            result_text = {0: "accepted", 1: "rejected (not ready)", 2: "unknown opcode"}.get(
                result, f"result {result}"
            )
            text += (
                f"\nAircraft ACK: {command['ack_op']} (seq {command.get('ack_seq', '—')}) — {result_text}\n"
                f"Aircraft mission state: {state}"
            )
        self.command_status.setText(text or "No commands sent yet.")

    def _map_clicked(self, lat: float, lon: float) -> None:
        try:
            self.mission.set_waypoint(lat, lon, self.waypoint_alt_spin.value())
        except ValueError:
            return
        self.map_widget.set_waypoint(lat, lon, self.waypoint_alt_spin.value())
        self._refresh_mission_summary()
        self.statusBar().showMessage(f"Waypoint set at {lat:.6f}, {lon:.6f}")

    def _follow_toggled(self, checked: bool) -> None:
        self.map_widget.follow = checked

    def _loiter_radius_changed(self, value: int) -> None:
        self.mission.loiter_radius_m = float(value)
        self.map_widget.set_loiter_radius(float(value))
        self._refresh_mission_summary()

    def _set_home_from_fix(self) -> None:
        gps = self.store.gps
        if not gps or not gps.get("fix"):
            self.statusBar().showMessage("No GPS fix available to set home")
            return
        lat = int(gps.get("lat_e7", 0) or 0) / 1e7
        lon = int(gps.get("lon_e7", 0) or 0) / 1e7
        alt = int(gps.get("alt_mm", 0) or 0) / 1000.0
        self.mission.set_home(lat, lon, alt)
        self.map_widget.set_home(lat, lon)
        self._refresh_mission_summary()
        self.statusBar().showMessage(f"Home set at {lat:.6f}, {lon:.6f}")

    def _clear_waypoint(self) -> None:
        self.mission.clear_waypoint()
        self.map_widget.clear_waypoint()
        self._refresh_mission_summary()

    def _refresh_mission_summary(self) -> None:
        aircraft = None
        gps = self.store.gps
        if gps and gps.get("fix"):
            aircraft = (int(gps.get("lat_e7", 0) or 0) / 1e7, int(gps.get("lon_e7", 0) or 0) / 1e7)
        lines = self.mission.summary(aircraft)
        self.mission_summary.setText("\n".join(lines))

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
        self.baro = BaroTracker()
        self.baro_calibrated = False
        self.mission.home = None
        self.map_widget.track.clear()
        self.map_widget.home = None
        self.map_widget.set_aircraft(0.0, 0.0, None, False)
        self.store.command = {}
        self._refresh_mission_summary()
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

    def _restart_attitude_calibration(self) -> None:
        self.attitude.start_calibration()
        self.horizon.set_attitude(0.0, 0.0, 0.0, False, self.attitude.calibration_status)
        pressure = self.store.baro.get("pressure_pa", 0) or 0
        if pressure:
            self.baro.calibrate(pressure)
            self.statusBar().showMessage(
                "Calibrating gyro/level and barometer baseline — keep the aircraft still"
            )
        else:
            self.statusBar().showMessage(
                "Calibrating gyro/level — barometer baseline will be captured when data arrives"
            )
        # In ground-station mode, also tell the aircraft to capture its own
        # level/gyro reference for self-level stabilization.
        if self.mode_combo.currentData() == "ground_station":
            self._send_command_line("CMD,calibrate_level")

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
        self._pending_log.append(json.dumps(packet, ensure_ascii=False, separators=(",", ":")))

        # Fast sensor-only UI: the horizon and plots follow the accelerometer
        # and gyro directly. Everything else renders on timers to keep the
        # event loop responsive.
        if packet.get("source") == "aircraft_usb" or packet.get("kind") == "sensor":
            self.accel_plot.add_values(self.store.imu.get("accel_mg", []), self.store.imu.get("ok", False))
            self.gyro_plot.add_values(self.store.imu.get("gyro_cdeg_s", []), self.store.imu.get("ok", False))
            roll, pitch, heading, valid = self.attitude.update(self.store.imu)
            self.horizon.set_attitude(roll, pitch, heading, valid, self.attitude.calibration_status)
            pressure = self.store.baro.get("pressure_pa", 0) or 0
            if pressure and not self.baro_calibrated:
                self.baro.calibrate(pressure)
                self.baro_calibrated = True

    def _refresh_fast_ui(self) -> None:
        rc = self.store.rc if self.store.rc else self.store.rc_sniff
        self.transmitter_view.set_rc(rc)
        self._update_channel_table(rc)

        # Auto-calibrate stick centers on the first RC packet
        # (sticks should be at rest when the ground station connects).
        if not self.stick_center_calibrated and rc:
            self.transmitter_view.aileron_center = int(
                rc.get("aileron_us", 1500) or 1500
            )
            self.transmitter_view.elevator_center = int(
                rc.get("elevator_us", 1500) or 1500
            )
            self.stick_center_calibrated = True
            self.statusBar().showMessage(
                f"Stick centers calibrated: A={self.transmitter_view.aileron_center}, "
                f"E={self.transmitter_view.elevator_center} (trim-aware)"
            )

    def _refresh_slow_ui(self) -> None:
        # Flush the packet log in batches instead of once per line.
        if self._pending_log:
            chunk = self._pending_log
            self._pending_log = []
            self.packet_log.appendPlainText("\n".join(chunk))
        self.packet_summary.setText(
            f"Frames displayed: {self.event_count}   |   Last: "
            f"{self.store.last_packet.get('source', '—')} / "
            f"{self.store.last_packet.get('kind', '—')} "
            f"seq={self.store.last_packet.get('seq', '—')}"
        )
        self._update_gps_panel()
        self._update_map_from_telemetry()
        self._refresh_command_status()
        self._refresh_mission_summary()
        self._refresh_health()

    def _update_map_from_telemetry(self) -> None:
        gps = self.store.gps
        fix = bool(gps.get("fix"))
        if fix:
            lat = int(gps.get("lat_e7", 0) or 0) / 1e7
            lon = int(gps.get("lon_e7", 0) or 0) / 1e7
            course = int(gps.get("course_centi_deg", 0) or 0) / 100.0
            speed = int(gps.get("speed_cms", 0) or 0) / 100.0
            heading = course if speed > 0.5 else None
            self.map_widget.set_aircraft(lat, lon, heading, True)
            self.map_widget.append_track(lat, lon)
            if not self.mission.home:
                self.mission.set_home(lat, lon, int(gps.get("alt_mm", 0) or 0) / 1000.0)
                self.map_widget.set_home(lat, lon)
            self._refresh_mission_summary()
        else:
            self.map_widget.set_aircraft(0.0, 0.0, None, False)

    def _update_channel_table(self, rc: dict | None = None) -> None:
        if rc is None:
            rc = self.store.rc
        if not rc:
            return
        trim_flags = int(rc.get("trim_flags", 0) or 0)
        mode = rc.get("throttle_mode")
        mode_label = {0: "MANUAL (0)", 1: "STABILIZED (1)", 2: "STABILIZED (2)"}.get(mode, "—")
        rate_pm = int(rc.get("throttle_boost", 0) or 0)
        rate_label = f"{rate_pm / 10:.0f}%" if rate_pm else "—"
        values = [
            f"{rc.get('throttle_us', '—')} µs",
            f"{rc.get('aileron_us', '—')} µs",
            f"{rc.get('elevator_us', '—')} µs",
            "ARMED" if rc.get("armed") else "DISARMED",
            mode_label,
            str(rc.get("aux1", "—")),
            rate_label,
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

        baro_alt_text = "—"
        pressure = self.store.baro.get("pressure_pa", 0) or 0
        if pressure:
            baro_alt = self.baro.altitude_m(pressure)
            baro_alt_text = f"{baro_alt:+.1f} m" + ("" if self.baro.calibrated else " (std baseline)")

        gps_alt_text = f"{altitude:.1f} m" if fix else "needs fix"
        self.gps_text.setText(
            f"GPS STATUS: {state}\n\n"
            f"Satellites: {sat_count}\n"
            f"Fix type: {gps.get('fix_type', '—')}\n"
            f"HDOP: {hdop:.2f}\n\n"
            f"Latitude: {lat:.7f}\n"
            f"Longitude: {lon:.7f}\n"
            f"GPS altitude: {gps_alt_text}\n"
            f"Baro altitude (rel): {baro_alt_text}\n\n"
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
            if radio.get("oled_test_mode"):
                self.tiles["radio"].set_state("RF DISABLED", "OLED-only test firmware", "#7ca7d8")
            elif "radio_ready" in radio:
                ready = bool(radio.get("radio_ready")) and bool(radio.get("radio_chip_ok"))
                age = int(radio.get("telemetry_age_ms", 65535) or 65535)
                has_air_data = self.store.last_rf_packet_at > 0
                live = ready and has_air_data and age < 10000
                air_tx = ""
                if "air_tx_ok" in radio or "air_tx_failed" in radio:
                    air_tx = f" · aircraft TX {radio.get('air_tx_ok', 0)}/{radio.get('air_tx_failed', 0)} failed"
                oled_status = (
                    f" · OLED 0x{int(radio.get('oled_addr', 0)):02X}"
                    if radio.get("oled_ready") else " · OLED not detected"
                )
                radio_value = (
                    "AIR LINK LIVE" if live else
                    "RADIO READY" if ready and not has_air_data else
                    "TELEMETRY STALE" if ready else
                    "RADIO INIT FAIL"
                )
                radio_detail = (
                    f"RX {radio.get('rx_packets', 0)} · lost {radio.get('lost_packets', 0)} · "
                    f"ACK {radio.get('auto_ack_count', 0)} · age {age} ms{air_tx}{oled_status}"
                    if has_air_data else f"Ground radio ready; no aircraft telemetry yet{oled_status}"
                )
                self.tiles["radio"].set_state(
                    radio_value,
                    radio_detail,
                    "#55cf9a" if live else ("#7ca7d8" if ready and not has_air_data else "#e2aa57"),
                )
            rc = self.store.rc
            since_rc = (time.monotonic() - self.store.last_rc_packet_at) * 1000 if self.store.last_rc_packet_at else 65535
            # rc_valid is set by the aircraft only when it has received a recent
            # RC packet, so it is the authoritative signal. The only display
            # condition is that the last control frame is reasonably fresh.
            valid = bool(rc.get("valid")) and since_rc < 10000
            sniff_fresh = (
                self.store.last_sniff_at > 0
                and time.monotonic() - self.store.last_sniff_at < 2.0
            )
            if valid:
                rc_value = "RC LIVE"
                rc_detail = f"Packets {rc.get('rc_packets', 0)} · age {rc.get('rc_age_ms', '—')} ms"
                rc_color = "#55cf9a"
            elif self.store.last_rc_packet_at:
                rc_value = "RC LINK LOST"
                rc_detail = f"Packets {rc.get('rc_packets', 0)} · age {rc.get('rc_age_ms', '—')} ms"
                rc_color = "#e2aa57"
            elif sniff_fresh:
                rc_value = "RC SNIFF LIVE"
                rc_detail = (
                    f"Transmitter heard directly by GCS · sniff packets "
                    f"{self.store.rc_sniff.get('packets', 0)}"
                )
                rc_color = "#7ca7d8"
            else:
                rc_value = "NO AIRCRAFT DATA"
                rc_detail = "RC status is unknown until aircraft telemetry arrives"
                rc_color = "#e2aa57"
            self.tiles["rc"].set_state(rc_value, rc_detail, rc_color)

        if self.store.source == "aircraft_usb":
            self.tiles["oled"].set_state("N/A", "OLED is connected to ground station", "#7ca7d8")
        elif self.store.source == "ground_station" and "oled_ready" in self.store.radio:
            if self.store.radio.get("oled_ready"):
                addr = int(self.store.radio.get("oled_addr", 0) or 0)
                self.tiles["oled"].set_state("FOUND", f"I2C ACK at 0x{addr:02X}", "#55cf9a")
            else:
                self.tiles["oled"].set_state("NOT FOUND", "No ACK at 0x3C or 0x3D", "#e27676")
        else:
            self.tiles["oled"].set_state("WAITING", "Ground station status not received", "#65758b")

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
        pressure = self.store.baro.get("pressure_pa", 0) or 0
        if self.store.baro.get("ok") and pressure:
            rel_alt = self.baro.altitude_m(pressure)
            temp = int(self.store.baro.get("temperature_centi_c", 0) or 0) / 100.0
            baro_detail = (
                f"alt {rel_alt:+.1f} m rel · {temp:.1f} °C"
                + ("" if self.baro.calibrated else " · std baseline")
            )
        else:
            baro_detail = "Check BMP280 I2C"
        self.tiles["baro"].set_state(
            "OK" if self.store.baro.get("ok") else "NO DATA",
            baro_detail,
            "#55cf9a" if self.store.baro.get("ok") else "#e2aa57",
        )
        gps = self.store.gps
        telemetry_stale = (
            self.store.source == "ground_station"
            and self.store.last_rf_packet_at
            and time.monotonic() - self.store.last_rf_packet_at > 10.0
        )
        if self.store.source == "ground_station" and not self.store.last_rf_packet_at:
            self.tiles["gps"].set_state("NO AIRCRAFT DATA", "GPS state is unknown until telemetry arrives", "#7ca7d8")
        elif telemetry_stale:
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
