"""Self-contained OpenStreetMap tile widget (no extra Qt modules).

Tiles are fetched with urllib in background threads, cached on disk, and
drawn with QPainter. Left-drag pans, wheel zooms, and a plain left click
emits map_clicked(lat, lon) for mission planning.
"""

from __future__ import annotations

import math
import os
import tempfile
import urllib.request

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QWidget

TILE_SIZE = 256
MIN_ZOOM = 3
MAX_ZOOM = 19
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "NanoLink-debug-monitor/1.0 (DIY RC telemetry hobby project)"


def _clip_lat(lat: float) -> float:
    return max(-85.05112878, min(85.05112878, lat))


def _world_size(zoom: int) -> float:
    return float(2 ** zoom) * TILE_SIZE


def lon_to_world_x(lon: float, zoom: int) -> float:
    return (lon + 180.0) / 360.0 * _world_size(zoom)


def lat_to_world_y(lat: float, zoom: int) -> float:
    lat = _clip_lat(lat)
    s = math.sin(math.radians(lat))
    return (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * _world_size(zoom)


def world_x_to_lon(x: float, zoom: int) -> float:
    return x / _world_size(zoom) * 360.0 - 180.0


def world_y_to_lat(y: float, zoom: int) -> float:
    n = math.pi - 2.0 * math.pi * y / _world_size(zoom)
    return math.degrees(math.atan(math.sinh(n)))


def meters_per_pixel(lat: float, zoom: int) -> float:
    return 156543.03392 * math.cos(math.radians(_clip_lat(lat))) / float(2 ** zoom)


class _TileSignals(QObject):
    loaded = Signal(int, int, int, bytes)


class _FetchTile(QRunnable):
    def __init__(self, zoom: int, x: int, y: int, cache_dir: str, signals: _TileSignals):
        super().__init__()
        self.zoom, self.x, self.y = zoom, x, y
        self.cache_dir = cache_dir
        self.signals = signals
        self.setAutoDelete(True)

    def run(self) -> None:  # executed in a worker thread
        path = os.path.join(self.cache_dir, f"{self.zoom}_{self.x}_{self.y}.png")
        data: bytes | None = None
        if os.path.exists(path):
            try:
                with open(path, "rb") as fh:
                    data = fh.read()
            except OSError:
                data = None
        if data is None:
            url = TILE_URL.format(z=self.zoom, x=self.x, y=self.y)
            try:
                request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
                with urllib.request.urlopen(request, timeout=6) as response:
                    data = response.read()
                try:
                    os.makedirs(self.cache_dir, exist_ok=True)
                    with open(path, "wb") as fh:
                        fh.write(data)
                except OSError:
                    pass
            except Exception:
                return
        self.signals.loaded.emit(self.zoom, self.x, self.y, data)


class TileMapWidget(QWidget):
    map_clicked = Signal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(620, 460)
        self.zoom = 16
        self.center_lat = 41.0082
        self.center_lon = 28.9784
        self.tiles: dict[tuple[int, int, int], QPixmap] = {}
        self.pending: set[tuple[int, int, int]] = set()
        self.failed: set[tuple[int, int, int]] = set()
        self.cache_dir = os.path.join(tempfile.gettempdir(), "nanolink_tiles")
        self._signals = _TileSignals()
        self._signals.loaded.connect(self._tile_loaded)
        self.pool = QThreadPool(self)
        self.pool.setMaxThreadCount(4)

        self.aircraft: tuple[float, float, float | None] | None = None  # lat, lon, heading
        self.track: list[tuple[float, float]] = []
        self.home: tuple[float, float] | None = None
        self.waypoint: tuple[float, float, float] | None = None  # lat, lon, alt
        self.loiter_radius_m = 30.0
        self.follow = True

        self._dragging = False
        self._drag_last: tuple[int, int] | None = None
        self._drag_moved = False
        self.setMouseTracking(True)

    # ---------------------------------------------------------------- public

    def set_aircraft(self, lat: float, lon: float, heading: float | None, has_fix: bool) -> None:
        if has_fix:
            self.aircraft = (lat, lon, heading)
            if self.follow:
                self.center_lat, self.center_lon = lat, lon
        else:
            self.aircraft = None
        self.update()

    def append_track(self, lat: float, lon: float) -> None:
        if not self.track or self.track[-1] != (lat, lon):
            self.track.append((lat, lon))
        del self.track[:-2000]
        self.update()

    def set_home(self, lat: float, lon: float) -> None:
        self.home = (lat, lon)
        self.update()

    def set_waypoint(self, lat: float, lon: float, alt_m: float) -> None:
        self.waypoint = (lat, lon, alt_m)
        self.update()

    def clear_waypoint(self) -> None:
        self.waypoint = None
        self.update()

    def set_loiter_radius(self, radius_m: float) -> None:
        self.loiter_radius_m = max(5.0, radius_m)
        self.update()

    def zoom_in(self) -> None:
        self._set_zoom(min(MAX_ZOOM, self.zoom + 1))

    def zoom_out(self) -> None:
        self._set_zoom(max(MIN_ZOOM, self.zoom - 1))

    # ------------------------------------------------------------- internals

    def _set_zoom(self, zoom: int) -> None:
        if zoom == self.zoom:
            return
        # Keep the geo point under the cursor stationary while zooming.
        anchor = None
        cursor = self.mapToGlobal(self.rect().center())  # fallback anchor
        if self.underMouse():
            pos = self.mapFromGlobal(cursor)
            anchor = self._screen_to_geo(pos.x(), pos.y())
        if self.follow and self.aircraft is not None and anchor is None:
            anchor = (self.aircraft[0], self.aircraft[1])
        self.zoom = zoom
        self.failed.clear()
        if anchor is not None:
            if self.follow and self.aircraft is not None:
                self.center_lat, self.center_lon = self.aircraft[0], self.aircraft[1]
            else:
                self._center_on_geo_keep_point(anchor[0], anchor[1])
        self.update()

    def _center_on_geo_keep_point(self, lat: float, lon: float) -> None:
        # Center so that (lat, lon) stays at the widget center while zooming.
        self.center_lat, self.center_lon = lat, lon

    def _screen_to_geo(self, x: int, y: int) -> tuple[float, float]:
        wx = lon_to_world_x(self.center_lon, self.zoom) - self.width() / 2.0 + x
        wy = lat_to_world_y(self.center_lat, self.zoom) - self.height() / 2.0 + y
        return world_y_to_lat(wy, self.zoom), world_x_to_lon(wx, self.zoom)

    def _geo_to_screen(self, lat: float, lon: float) -> tuple[int, int]:
        wx = lon_to_world_x(lon, self.zoom)
        wy = lat_to_world_y(lat, self.zoom)
        cx = lon_to_world_x(self.center_lon, self.zoom)
        cy = lat_to_world_y(self.center_lat, self.zoom)
        return int(wx - cx + self.width() / 2.0), int(wy - cy + self.height() / 2.0)

    def _tile_loaded(self, zoom: int, x: int, y: int, data: bytes) -> None:
        self.pending.discard((zoom, x, y))
        pixmap = QPixmap()
        if pixmap.loadFromData(data):
            self.tiles[(zoom, x, y)] = pixmap
            self.update()

    def _request_tile(self, zoom: int, x: int, y: int) -> None:
        count = 2 ** zoom
        x %= count
        key = (zoom, x, y)
        if key in self.tiles or key in self.pending or key in self.failed:
            return
        self.pending.add(key)
        self.pool.start(_FetchTile(zoom, x, y, self.cache_dir, self._signals))

    # ------------------------------------------------------------------ Qt

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = True
            self._drag_last = (event.position().x(), event.position().y())
            self._drag_moved = False

    def mouseMoveEvent(self, event) -> None:
        if not self._dragging or self._drag_last is None:
            return
        dx = event.position().x() - self._drag_last[0]
        dy = event.position().y() - self._drag_last[1]
        if abs(dx) + abs(dy) > 2:
            self._drag_moved = True
        if self._drag_moved:
            wx = lon_to_world_x(self.center_lon, self.zoom) - dx
            wy = lat_to_world_y(self.center_lat, self.zoom) - dy
            self.center_lon = world_x_to_lon(wx, self.zoom)
            self.center_lat = _clip_lat(world_y_to_lat(wy, self.zoom))
            self._drag_last = (event.position().x(), event.position().y())
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        was_click = self._dragging and not self._drag_moved
        self._dragging = False
        self._drag_last = None
        if was_click:
            lat, lon = self._screen_to_geo(int(event.position().x()), int(event.position().y()))
            self.map_clicked.emit(lat, lon)

    def wheelEvent(self, event) -> None:
        if event.angleDelta().y() > 0:
            self.zoom_in()
        elif event.angleDelta().y() < 0:
            self.zoom_out()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#14202e"))

        cx = lon_to_world_x(self.center_lon, self.zoom)
        cy = lat_to_world_y(self.center_lat, self.zoom)
        top_left_x = cx - self.width() / 2.0
        top_left_y = cy - self.height() / 2.0
        tile_x0 = int(math.floor(top_left_x / TILE_SIZE))
        tile_y0 = int(math.floor(top_left_y / TILE_SIZE))
        tile_x1 = int(math.floor((top_left_x + self.width()) / TILE_SIZE))
        tile_y1 = int(math.floor((top_left_y + self.height()) / TILE_SIZE))

        for ty in range(tile_y0, tile_y1 + 1):
            for tx in range(tile_x0, tile_x1 + 1):
                wrapped = tx % (2 ** self.zoom)
                key = (self.zoom, wrapped, ty)
                sx = int(tx * TILE_SIZE - top_left_x)
                sy = int(ty * TILE_SIZE - top_left_y)
                pixmap = self.tiles.get(key)
                if pixmap is not None:
                    painter.drawPixmap(sx, sy, pixmap)
                else:
                    self._request_tile(self.zoom, tx, ty)
                    painter.fillRect(sx, sy, TILE_SIZE, TILE_SIZE, QColor("#1c2a3a"))

        # GPS track history
        if len(self.track) >= 2:
            painter.setPen(QPen(QColor("#59c6a5"), 2))
            previous = self._geo_to_screen(*self.track[0])
            for point in self.track[1:]:
                current = self._geo_to_screen(*point)
                painter.drawLine(previous[0], previous[1], current[0], current[1])
                previous = current

        # Loiter circle around home
        if self.home is not None:
            hx, hy = self._geo_to_screen(*self.home)
            mpp = meters_per_pixel(self.home[0], self.zoom)
            radius_px = max(4.0, self.loiter_radius_m / mpp)
            painter.setPen(QPen(QColor("#e2aa57"), 1, Qt.PenStyle.DashLine))
            painter.drawEllipse(int(hx - radius_px), int(hy - radius_px),
                                 int(radius_px * 2), int(radius_px * 2))

        # Home marker
        if self.home is not None:
            hx, hy = self._geo_to_screen(*self.home)
            painter.setPen(QPen(QColor("#e2aa57"), 2))
            painter.setBrush(QColor("#e2aa57"))
            painter.drawRect(hx - 5, hy - 5, 10, 10)
            painter.setPen(QColor("#e7edf5"))
            painter.setFont(QFont("Monospace", 8))
            painter.drawText(hx + 8, hy - 6, "HOME")

        # Waypoint marker
        if self.waypoint is not None:
            wxp, wyp = self._geo_to_screen(self.waypoint[0], self.waypoint[1])
            painter.setPen(QPen(QColor("#e27676"), 2))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(wxp - 8, wyp - 8, 16, 16)
            painter.drawLine(wxp - 12, wyp, wxp + 12, wyp)
            painter.drawLine(wxp, wyp - 12, wxp, wyp + 12)
            painter.setPen(QColor("#e7edf5"))
            painter.setFont(QFont("Monospace", 8))
            painter.drawText(wxp + 12, wyp + 14, f"WP {self.waypoint[2]:.0f} m")

        # Aircraft marker
        if self.aircraft is not None:
            ax, ay = self._geo_to_screen(self.aircraft[0], self.aircraft[1])
            heading = self.aircraft[2]
            painter.save()
            painter.translate(ax, ay)
            if heading is not None:
                painter.rotate(heading)
            painter.setPen(QPen(QColor("#f5c451"), 2))
            painter.setBrush(QColor("#f5c451"))
            painter.drawPolygon(
                [(0, -10), (7, 8), (0, 4), (-7, 8)],
            )
            painter.restore()

        # Scale bar and hint text
        mpp = meters_per_pixel(self.center_lat, self.zoom)
        bar_px = 80
        bar_m = mpp * bar_px
        if bar_m >= 1000.0:
            label = f"{bar_m / 1000.0:.1f} km"
        else:
            label = f"{bar_m:.0f} m"
        painter.setPen(QPen(QColor("#e7edf5"), 2))
        painter.drawLine(14, self.height() - 18, 14 + bar_px, self.height() - 18)
        painter.setFont(QFont("Monospace", 8))
        painter.drawText(14, self.height() - 24, label)
        painter.drawText(
            14, self.height() - 6,
            f"zoom {self.zoom} · drag pan · wheel zoom · click sets waypoint",
        )
