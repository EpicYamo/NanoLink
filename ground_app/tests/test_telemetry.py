import json
import unittest

from mission import MissionPlan, bearing_deg, compass_label, distance_m
from telemetry import AttitudeEstimator, BaroTracker, TelemetryStore


class GeoHelperTests(unittest.TestCase):
    def test_equatorial_one_degree_longitude(self):
        d = distance_m(0.0, 0.0, 0.0, 1.0)
        self.assertGreater(d, 110_000)
        self.assertLess(d, 112_000)

    def test_bearing_cardinal(self):
        self.assertAlmostEqual(bearing_deg(0.0, 0.0, 1.0, 0.0), 0.0, delta=0.1)   # north
        self.assertAlmostEqual(bearing_deg(0.0, 0.0, 0.0, 1.0), 90.0, delta=0.1)   # east
        self.assertAlmostEqual(bearing_deg(1.0, 0.0, 0.0, 0.0), 180.0, delta=0.1)  # south
        self.assertAlmostEqual(compass_label(350.0), "N")
        self.assertAlmostEqual(compass_label(45.0), "NE")


class MissionPlanTests(unittest.TestCase):
    def test_summary_before_any_setup(self):
        plan = MissionPlan()
        lines = plan.summary()
        self.assertTrue(any("not set" in line for line in lines))
        self.assertFalse(plan.ready)

    def test_ready_and_distances(self):
        plan = MissionPlan()
        plan.set_home(41.0, 29.0, alt_m=100.0)
        plan.set_waypoint(41.01, 29.01, 60.0)
        self.assertTrue(plan.ready)
        lines = plan.summary(aircraft=(41.005, 29.005))
        joined = "\n".join(lines)
        self.assertIn("Home → WP", joined)
        self.assertIn("Aircraft → WP", joined)
        self.assertIn("not implemented", joined)

    def test_waypoint_rejects_negative_altitude(self):
        plan = MissionPlan()
        with self.assertRaises(ValueError):
            plan.set_waypoint(41.0, 29.0, -5.0)


class BaroTrackerTests(unittest.TestCase):
    def test_zero_at_calibrated_baseline(self):
        baro = BaroTracker()
        baro.calibrate(100_000.0)
        self.assertAlmostEqual(baro.altitude_m(100_000.0), 0.0, delta=0.5)

    def test_climb_decreases_pressure(self):
        baro = BaroTracker()
        baro.calibrate(100_000.0)
        alt = baro.altitude_m(90_000.0)
        self.assertGreater(alt, 800.0)
        self.assertLess(alt, 960.0)
        self.assertTrue(baro.calibrated)

    def test_standard_baseline_before_calibration(self):
        baro = BaroTracker()
        self.assertFalse(baro.calibrated)
        alt = baro.altitude_m(101_325.0)
        self.assertAlmostEqual(alt, 0.0, delta=1.0)

    def test_invalid_pressure_is_ignored_on_calibration(self):
        baro = BaroTracker()
        baro.calibrate(0)
        self.assertFalse(baro.calibrated)


class TelemetryStoreTests(unittest.TestCase):
    def test_direct_usb_sample(self):
        store = TelemetryStore()
        packet = {
            "source": "aircraft_usb",
            "kind": "sample",
            "nrf": {"enabled": False},
            "imu": {"ok": True, "accel_mg": [0, 0, 1000], "gyro_cdeg_s": [0, 0, 0]},
            "baro": {"ok": True, "pressure_pa": 101325},
            "gps": {"stream": False, "fix": False},
            "rc": {"valid": False},
        }
        store.ingest(json.dumps(packet))
        self.assertEqual(store.source, "aircraft_usb")
        self.assertTrue(store.imu["ok"])
        self.assertFalse(store.rc["valid"])

    def test_ground_station_packets_merge(self):
        store = TelemetryStore()
        packets = [
            {"source": "ground_station", "kind": "sensor", "imu_ok": True,
             "accel_mg": [0, 0, 1000], "gyro_cdeg_s": [0, 0, 0], "baro_ok": True},
            {"source": "ground_station", "kind": "control", "rc_valid": True,
             "armed": False, "throttle_us": 1200, "aileron_us": 1500,
             "elevator_us": 1500, "trim_flags": 2},
            {"source": "ground_station", "kind": "gps", "stream_ok": True,
             "fix_ok": True, "satellites": 8, "lat_e7": 410000000},
        ]
        for packet in packets:
            store.ingest(json.dumps(packet))
        self.assertTrue(store.imu["ok"])
        self.assertTrue(store.rc["valid"])
        self.assertEqual(store.rc["throttle_us"], 1200)
        self.assertTrue(store.gps["fix"])
        self.assertEqual(store.gps["satellites"], 8)

    def test_attitude_estimator_uses_static_gravity(self):
        estimator = AttitudeEstimator()
        imu = {
            "ok": True,
            "accel_mg": [0, 0, -1000],
            "gyro_cdeg_s": [100, 0, 0],
        }
        for _ in range(estimator.CALIBRATION_SAMPLES):
            roll, pitch, _heading, valid = estimator.update(imu)
        self.assertTrue(valid)
        self.assertAlmostEqual(roll, 0.0, delta=0.1)
        self.assertAlmostEqual(pitch, 0.0, delta=0.1)
        self.assertEqual(estimator.calibration_status, "CALIBRATED · DISPLAY ONLY")


if __name__ == "__main__":
    unittest.main()
