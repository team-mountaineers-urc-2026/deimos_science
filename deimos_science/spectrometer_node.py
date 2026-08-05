"""
spectrometer_node.py  (runs on the ROVER)

Topics published:
  /spectrometer/result          robot_interfaces/msg/SpectrometerData
      — live frames only while streaming is enabled (Live toggle in GUI)
  /spectrometer/collect_data    robot_interfaces/msg/SpectrometerData
      — fired ONCE per Collect Data press; single raw frame, no baseline needed
        the base station viz node writes this to ~/spectrometer_readings/<label>.csv
  /spectrometer/snapshot_data   robot_interfaces/msg/SpectrometerData
      — fired ONCE per Save Snapshot press; absorbance + raw pair
        requires a baseline to have been captured first

Topics subscribed:
  /spectrometer/request          std_msgs/msg/Int32   — set integration time µs
  /spectrometer/capture_baseline std_msgs/msg/Empty   — average last 10 live frames → baseline
  /spectrometer/save_snapshot    std_msgs/msg/String  — trigger absorbance snapshot pair
  /spectrometer/collect_data_req std_msgs/msg/String  — trigger single one-shot CSV
  /spectrometer/set_streaming    std_msgs/msg/Bool    — enable/disable live stream

Parameters:
  serial_number            (string, default "S14413")
  integration_time_micros  (int,    default 100000)
  stream_hz                (double, default 10.0)  dfdf
"""

import numpy as np
from collections import deque
from datetime import datetime

import seabreeze
seabreeze.use('cseabreeze')
from seabreeze.spectrometers import Spectrometer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Empty, String, Bool
from robot_interfaces.msg import SpectrometerData


class SpectrometerNode(Node):

    def __init__(self):
        super().__init__('spectrometer_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_number', 'S14413')
        self.declare_parameter('integration_time_micros', 100_000)
        self.declare_parameter('stream_hz', 10.0)

        self.integration_time: int = (
            self.get_parameter('integration_time_micros')
            .get_parameter_value().integer_value
        )

        # ── Internal state ────────────────────────────────────────────────────
        self.spec = None
        self.wavelengths: list[float] = []
        self.frame_buffer: deque = deque(maxlen=10)
        self.baseline: np.ndarray | None = None
        self.streaming_enabled: bool = False

        # ── Publishers ────────────────────────────────────────────────────────
        self.live_pub = self.create_publisher(
            SpectrometerData, '/spectrometer/result', 10)
        self.collect_pub = self.create_publisher(
            SpectrometerData, '/spectrometer/collect_data', 10)
        self.snapshot_pub = self.create_publisher(
            SpectrometerData, '/spectrometer/snapshot_data', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Int32,  '/spectrometer/request',          self._on_request,          10)
        self.create_subscription(
            Empty,  '/spectrometer/capture_baseline', self._on_capture_baseline, 10)
        self.create_subscription(
            Empty, '/spectrometer/reset_baseline', self._on_reset_baseline, 10)
        self.create_subscription(
            String, '/spectrometer/save_snapshot',    self._on_save_snapshot,    10)
        self.create_subscription(
            String, '/spectrometer/collect_data_req', self._on_collect_data,     10)
        self.create_subscription(
            Bool,   '/spectrometer/set_streaming',    self._on_set_streaming,    10)

        # ── Streaming timer ───────────────────────────────────────────────────
        hz = self.get_parameter('stream_hz').get_parameter_value().double_value
        self.stream_timer = self.create_timer(1.0 / hz, self._stream_tick)

        self._connect_device()
        self.get_logger().info('SpectrometerNode ready. Streaming OFF by default.')

    # ── Device ────────────────────────────────────────────────────────────────

    def _connect_device(self) -> bool:
        sn = self.get_parameter('serial_number').get_parameter_value().string_value
        try:
            self.spec = Spectrometer.from_serial_number(sn)
        except Exception:
            try:
                self.get_logger().warn(f'Serial {sn} not found, trying first available.')
                self.spec = Spectrometer.from_first_available()
            except Exception as e:
                self.get_logger().error(f'No spectrometer found: {e}')
                self.spec = None
                return False
        self.spec.integration_time_micros(self.integration_time)
        self.wavelengths = list(self.spec.wavelengths())
        self.get_logger().info(f'Connected: {self.spec.model}')
        return True

    def _ensure_device(self) -> bool:
        return self.spec is not None or self._connect_device()

    def _read_frame(self) -> 'np.ndarray | None':
        """Read one raw frame from hardware. Returns None on failure."""
        if not self._ensure_device():
            return None
        try:
            return self.spec.intensities(
                correct_dark_counts=False, correct_nonlinearity=False)
        except Exception as e:
            self.get_logger().warn(f'Read error: {e}')
            try:
                self.spec.close()
            except Exception:
                pass
            self.spec = None
            return None

    # ── Live streaming ────────────────────────────────────────────────────────

    def _stream_tick(self):
        if not self.streaming_enabled:
            return
        intensities = self._read_frame()
        if intensities is None:
            return

        self.frame_buffer.append(intensities)

        # Use mean of last N frames instead of single frame
        averaged = np.mean(np.stack(list(self.frame_buffer), axis=0), axis=0)

        msg = SpectrometerData()
        msg.wavelengths = self.wavelengths
        msg.is_successful = True

        if self.baseline is not None:
            eps = 1e-12
            ratio = averaged / np.clip(self.baseline, eps, None)
            absorbance = -np.log10(np.clip(ratio, eps, None))
            msg.intensities = absorbance.tolist()
            msg.filename = 'live_absorbance'
        else:
            msg.intensities = averaged.tolist()
            msg.filename = 'live_intensity'

        self.live_pub.publish(msg)

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_set_streaming(self, msg: Bool):
        self.streaming_enabled = msg.data
        self.get_logger().info(f'Streaming {"ON" if msg.data else "OFF"}.')

    def _on_request(self, msg: Int32):
        new_time = msg.data
        if not (10 <= new_time <= 85_000_000):
            self.get_logger().warn(f'Integration time {new_time} µs out of range. Ignored.')
            return
        self.integration_time = new_time
        if self._ensure_device():
            self.spec.integration_time_micros(new_time)
            self.get_logger().info(f'Integration time set to {new_time} µs.')

    def _on_reset_baseline(self, _msg: Empty):
        self.baseline = None
        self.frame_buffer.clear()
        self.get_logger().info('Baseline reset.')

    def _on_collect_data(self, msg: String):
        """
        One-shot reading. Reads a single raw frame and publishes it on
        /spectrometer/collect_data. No baseline required. The viz node on the
        base station writes the CSV to ~/spectrometer_readings/<label>.csv with
        two tab-separated columns:
            wavelength_nm   intensity
        """
        intensities = self._read_frame()
        if intensities is None:
            self.get_logger().error('Collect Data failed: could not read from device.')
            return

        base_name = msg.data.strip() or 'reading'
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        label = f'{base_name}_{timestamp}'

        collect_msg = SpectrometerData()
        collect_msg.wavelengths = self.wavelengths
        collect_msg.intensities = intensities.tolist()
        collect_msg.is_successful = True
        collect_msg.filename = label
        self.collect_pub.publish(collect_msg)

        self.get_logger().info(f'Published collect_data: {label}')

    def _on_capture_baseline(self, _msg: Empty):
        if len(self.frame_buffer) < self.frame_buffer.maxlen:
            self.get_logger().warn(
                f'Only {len(self.frame_buffer)}/{self.frame_buffer.maxlen} frames buffered. '
                'Enable live streaming and wait ~1 second, then try again.')
            return
        self.baseline = np.mean(np.stack(list(self.frame_buffer), axis=0), axis=0)
        self.get_logger().info('Baseline captured.')

    def _on_save_snapshot(self, msg: String):
        """
        Absorbance snapshot — requires a baseline to have been captured.
        Reads one fresh frame, computes absorbance, and publishes TWO messages
        on /spectrometer/snapshot_data:
          1. absorbance values  (filename ends in _absorbance)
          2. raw intensities    (filename ends in _raw)
        The viz node writes both CSVs to ~/spectrometer_readings/.
        """
        if self.baseline is None:
            self.get_logger().warn('Snapshot requested but no baseline captured.')
            return

        intensities = self._read_frame()
        if intensities is None:
            self.get_logger().error('Snapshot failed: could not read from device.')
            return

        base_name = msg.data.strip() or 'snapshot'
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        label = f'{base_name}_{timestamp}'

        eps = 1e-12
        ratio = intensities / np.clip(self.baseline, eps, None)
        absorbance = -np.log10(np.clip(ratio, eps, None))

        abs_msg = SpectrometerData()
        abs_msg.wavelengths = self.wavelengths
        abs_msg.intensities = absorbance.tolist()
        abs_msg.is_successful = True
        abs_msg.filename = f'{label}_absorbance'
        self.snapshot_pub.publish(abs_msg)

        raw_msg = SpectrometerData()
        raw_msg.wavelengths = self.wavelengths
        raw_msg.intensities = intensities.tolist()
        raw_msg.is_successful = True
        raw_msg.filename = f'{label}_raw'
        self.snapshot_pub.publish(raw_msg)

        self.get_logger().info(f'Published snapshot pair: {label}')

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.spec is not None:
            try:
                self.spec.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpectrometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
