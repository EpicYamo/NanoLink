"""Ground-side mission planning model.

The planner stores and visualizes a mission (home, one waypoint with a target
altitude, loiter radius around home). Executing the mission on the aircraft is
NOT implemented: the aircraft firmware currently has no stabilization mode and
no RF command uplink. This module only prepares and displays the plan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

EARTH_RADIUS_M = 6371008.8


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters (haversine)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing in degrees, 0 = north, clockwise."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return math.degrees(math.atan2(y, x)) % 360.0


_COMPASS = [
    (0.0, "N"), (22.5, "NNE"), (45.0, "NE"), (67.5, "ENE"),
    (90.0, "E"), (112.5, "ESE"), (135.0, "SE"), (157.5, "SSE"),
    (180.0, "S"), (202.5, "SSW"), (225.0, "SW"), (247.5, "WSW"),
    (270.0, "W"), (292.5, "WNW"), (315.0, "NW"), (337.5, "NNW"),
]


def compass_label(degrees: float) -> str:
    best = _COMPASS[0][1]
    best_delta = 360.0
    for threshold, label in _COMPASS:
        delta = abs((degrees - threshold + 180.0) % 360.0 - 180.0)
        if delta < best_delta:
            best_delta, best = delta, label
    return best


@dataclass
class MissionPlan:
    """One-waypoint mission: fly to the point at altitude, return home, loiter."""

    home: tuple[float, float] | None = None
    home_alt_m: float | None = None
    waypoint: tuple[float, float, float] | None = None  # lat, lon, altitude m
    loiter_radius_m: float = 30.0
    notes: list[str] = field(default_factory=list)

    def set_home(self, lat: float, lon: float, alt_m: float | None = None) -> None:
        self.home = (lat, lon)
        if alt_m is not None:
            self.home_alt_m = alt_m

    def set_waypoint(self, lat: float, lon: float, alt_m: float) -> None:
        if alt_m < 0:
            raise ValueError("waypoint altitude must be >= 0")
        self.waypoint = (lat, lon, alt_m)

    def clear_waypoint(self) -> None:
        self.waypoint = None

    @property
    def ready(self) -> bool:
        return self.home is not None and self.waypoint is not None

    def summary(self, aircraft: tuple[float, float] | None = None) -> list[str]:
        lines: list[str] = []
        if self.home is None:
            lines.append("Home: not set")
        else:
            lines.append(f"Home: {self.home[0]:.6f}, {self.home[1]:.6f}")
        if self.waypoint is None:
            lines.append("Waypoint: not set — click the map to choose one")
        else:
            wp_lat, wp_lon, wp_alt = self.waypoint
            lines.append(f"Waypoint: {wp_lat:.6f}, {wp_lon:.6f} @ {wp_alt:.0f} m")
            if self.home is not None:
                dist = distance_m(self.home[0], self.home[1], wp_lat, wp_lon)
                brg = bearing_deg(self.home[0], self.home[1], wp_lat, wp_lon)
                lines.append(
                    f"Home → WP: {dist:.0f} m, bearing {brg:.0f}° ({compass_label(brg)})"
                )
            if aircraft is not None:
                dist = distance_m(aircraft[0], aircraft[1], wp_lat, wp_lon)
                brg = bearing_deg(aircraft[0], aircraft[1], wp_lat, wp_lon)
                lines.append(
                    f"Aircraft → WP: {dist:.0f} m, bearing {brg:.0f}° ({compass_label(brg)})"
                )
        lines.append(f"Loiter radius around home: {self.loiter_radius_m:.0f} m")
        lines.append("Execution: not available — aircraft command uplink is not implemented")
        return lines
