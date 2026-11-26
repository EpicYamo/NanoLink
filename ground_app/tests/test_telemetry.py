import json
import unittest

from telemetry import AttitudeEstimator, TelemetryStore


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
