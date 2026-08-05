import seabreeze
seabreeze.use('cseabreeze')

from seabreeze.spectrometers import Spectrometer, list_devices
import numpy as np
from collections import deque
import time

try:
    from scipy.signal import find_peaks
except ImportError as exc:
    print("Missing dependency: scipy. Install with: pip install scipy")
    raise SystemExit(1) from exc

try:
    import colour
except ImportError as exc:
    print("Missing dependency: colour-science. Install with: pip install colour-science")
    raise SystemExit(1) from exc

try:
    _wavelength_to_xyz = colour.wavelength_to_XYZ
except AttributeError:
    from colour.colorimetry import wavelength_to_XYZ as _wavelength_to_xyz

try:
    _XYZ_to_srgb = colour.XYZ_to_sRGB
except AttributeError:
    from colour.models import XYZ_to_sRGB as _XYZ_to_srgb

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import SpectrometerData
from std_msgs.msg import Int32, Empty, String
from robot_interfaces.msg import StringArray

# Install the following two commands:
# pip install seabreeze[cseabreeze]
# seabreeze_os_setup
# pip install scipy colour-science

# Additionally, add user to dialout and tty groups using:
# sudo usermod -a -G tty username_here
# sudo usermod -a -G dialout username_here

VISIBLE_MIN_NM  = 380.0
VISIBLE_MAX_NM  = 780.0
PEAK_COUNT      = 3
VALLEY_COUNT    = 3
PEAK_PROMINENCE = 0.01
REPORT_EVERY_S  = 1.0


def wavelengths_to_srgb(wavelengths_nm):
    """Convert an array of wavelengths (nm) to sRGB colours using colour-science."""
    cmfs       = colour.MSDS_CMFS["CIE 1931 2 Degree Standard Observer"]
    illuminant = colour.SDS_ILLUMINANTS["D65"]
    rgbs = np.zeros((len(wavelengths_nm), 3), dtype=float)
    for idx, wl in enumerate(wavelengths_nm):
        if VISIBLE_MIN_NM <= wl <= VISIBLE_MAX_NM:
            try:
                xyz = _wavelength_to_xyz(wl, cmfs=cmfs, illuminant=illuminant)
            except TypeError:
                xyz = _wavelength_to_xyz(wl, cmfs=cmfs)
            xyz = np.asarray(xyz, dtype=float)
            if xyz[1] > 0:
                xyz = xyz / xyz[1]
            rgb = _XYZ_to_srgb(xyz)
            rgbs[idx] = np.clip(rgb, 0.0, 1.0)
    return rgbs


def rgb_to_hex(rgb):
    r, g, b = (np.clip(rgb, 0.0, 1.0) * 255).astype(int)
    return f"#{r:02X}{g:02X}{b:02X}"


class SpectrometerNode(Node):

    def __init__(self):
        super().__init__('science_spectrometer_node')

        # ── Parameters ───────────────────────────────────────────────────────────
        self.declare_parameter('serial_number', "S14413")
        self.declare_parameter('integration_time_micros', 100_000)
        self.declare_parameter('stream_hz', 10.0)

        self.integration_time = (
            self.get_parameter('integration_time_micros')
            .get_parameter_value().integer_value
        )

        # ── Internal state ────────────────────────────────────────────────────────
        self.spec             = None
        self.wavelengths      = []
        self.spectrum_rgb     = []          # per-wavelength sRGB colours
        self.frame_buffer     = deque(maxlen=10)
        self.baseline         = None
        self.last_report_time = time.monotonic()

        # ── Publishers ────────────────────────────────────────────────────────────
        self.live_pub  = self.create_publisher(SpectrometerData, '/spectrometer/result', 10)
        self.save_pub  = self.create_publisher(SpectrometerData, '/spectrometer/save',   10)
        self.peaks_pub = self.create_publisher(String,           '/spectrometer/peaks',  10)

        # ── Subscribers ───────────────────────────────────────────────────────────
        # integration time change + one-shot capture (matches old node's topic)
        self.create_subscription(Int32, 'spectrometer/request',
                                 self._on_request, 10)
        # baseline capture trigger from GUI
        self.create_subscription(Empty, '/spectrometer/capture_baseline',
                                 self._on_capture_baseline, 10)
        # snapshot save trigger from GUI (optional — GUI can also use collect data)
        self.create_subscription(String, '/spectrometer/save_snapshot',
                                 self._on_save_snapshot, 10)

        # ── Streaming timer ───────────────────────────────────────────────────────
        hz = self.get_parameter('stream_hz').get_parameter_value().double_value
        self.create_timer(1.0 / hz, self._stream_tick)

        # ── Connect device ────────────────────────────────────────────────────────
        self._connect_device()
        self.get_logger().info('SpectrometerNode ready.')

    # ── Device management ─────────────────────────────────────────────────────────

    def _connect_device(self) -> bool:
        sn = self.get_parameter('serial_number').get_parameter_value().string_value
        try:
            self.spec = Spectrometer.from_serial_number(sn)
        except Exception:
            try:
                self.get_logger().warn(
                    f'Serial {sn} not found, trying first available device.')
                self.spec = Spectrometer.from_first_available()
            except Exception as e:
                self.get_logger().error(f'No spectrometer found: {e}')
                self.spec = None
                return False

        self.spec.integration_time_micros(self.integration_time)
        self.wavelengths  = np.array(self.spec.wavelengths())
        self.spectrum_rgb = wavelengths_to_srgb(self.wavelengths)
        self.get_logger().info(f'Connected to: {self.spec.model}')
        return True

    def _ensure_device(self) -> bool:
        if self.spec is not None:
            return True
        return self._connect_device()

    # ── Streaming ─────────────────────────────────────────────────────────────────

    def _stream_tick(self):
        if not self._ensure_device():
            return

        try:
            intensities = self.spec.intensities(
                correct_dark_counts=False, correct_nonlinearity=False)
        except Exception as e:
            self.get_logger().warn(f'Read error, dropping frame: {e}')
            try:
                self.spec.close()
            except Exception:
                pass
            self.spec = None
            return

        self.frame_buffer.append(intensities)

        # Compute display values — absorbance if baseline set, else raw intensity
        if self.baseline is not None:
            current_y = self._compute_absorbance(intensities)
            label = 'live_absorbance'
        else:
            current_y = intensities
            label = 'live_intensity'

        # Publish live frame
        msg = SpectrometerData()
        msg.wavelengths   = self.wavelengths.tolist()
        msg.intensities   = current_y.tolist()
        msg.is_successful = True
        msg.filename      = label
        self.live_pub.publish(msg)

        # Periodic peak/valley report (only meaningful in absorbance mode)
        if self.baseline is not None:
            self._maybe_publish_peaks(current_y)

    # ── Absorbance ────────────────────────────────────────────────────────────────

    def _compute_absorbance(self, intensities: np.ndarray) -> np.ndarray:
        eps        = 1e-12
        ratio      = intensities / np.clip(self.baseline, eps, None)
        absorbance = -np.log10(np.clip(ratio, eps, None))
        return absorbance

    # ── Peak / valley detection ───────────────────────────────────────────────────

    def _maybe_publish_peaks(self, current_y: np.ndarray):
        now = time.monotonic()
        if now - self.last_report_time < REPORT_EVERY_S:
            return
        self.last_report_time = now

        peaks,   peak_props   = find_peaks( current_y, prominence=PEAK_PROMINENCE)
        valleys, valley_props = find_peaks(-current_y, prominence=PEAK_PROMINENCE)

        peak_prom   = peak_props.get('prominences',   np.array([]))
        valley_prom = valley_props.get('prominences', np.array([]))

        peak_order   = np.argsort(peak_prom)[::-1][:PEAK_COUNT]   if len(peak_prom)   else []
        valley_order = np.argsort(valley_prom)[::-1][:VALLEY_COUNT] if len(valley_prom) else []

        peak_idx   = peaks[peak_order]     if len(peak_order)   else []
        valley_idx = valleys[valley_order] if len(valley_order) else []

        def format_features(label, indices):
            if len(indices) == 0:
                return f"{label}: none"
            parts = []
            for idx in indices:
                wl    = self.wavelengths[idx]
                val   = current_y[idx]
                color = rgb_to_hex(self.spectrum_rgb[idx])
                parts.append(f"{wl:.1f}nm {val:.3f} AU {color}")
            return f"{label}: " + ", ".join(parts)

        report = (
            f"{format_features('Peaks', peak_idx)} | "
            f"{format_features('Valleys', valley_idx)}"
        )
        self.get_logger().info(report)

        peaks_msg      = String()
        peaks_msg.data = report
        self.peaks_pub.publish(peaks_msg)

    # ── Subscribers ───────────────────────────────────────────────────────────────

    def _on_request(self, msg: Int32):
        """Change integration time and trigger a one-shot save capture."""
        new_time = msg.data
        if not (10 <= new_time <= 85_000_000):
            self.get_logger().warn(
                f'Integration time {new_time} µs out of range [10, 85000000]. Ignored.')
            return

        self.integration_time = new_time
        if self._ensure_device():
            self.spec.integration_time_micros(new_time)
            self.get_logger().info(f'Integration time set to {new_time} µs.')
            self._capture_and_save()

    def _on_capture_baseline(self, _msg: Empty):
        """Average the last 10 buffered frames and store as baseline."""
        if len(self.frame_buffer) < self.frame_buffer.maxlen:
            self.get_logger().warn(
                f'Only {len(self.frame_buffer)}/{self.frame_buffer.maxlen} frames '
                'buffered — keep streaming and try again.')
            return
        self.baseline = np.mean(np.stack(list(self.frame_buffer), axis=0), axis=0)
        self.get_logger().info('Baseline captured from last 10 frames.')

    def _on_save_snapshot(self, msg: String):
        """Save the current absorbance frame as a named CSV snapshot."""
        if self.baseline is None:
            self.get_logger().warn('No baseline set — cannot save absorbance snapshot.')
            return
        if not self._ensure_device():
            return
        try:
            intensities = self.spec.intensities(
                correct_dark_counts=False, correct_nonlinearity=False)
        except Exception as e:
            self.get_logger().error(f'Snapshot read failed: {e}')
            return

        absorbance = self._compute_absorbance(intensities)
        self._publish_save(absorbance, label=msg.data or 'snapshot')

    # ── One-shot capture ──────────────────────────────────────────────────────────

    def _capture_and_save(self):
        """Read one frame and publish on /spectrometer/save for spectrometer_viz."""
        if not self._ensure_device():
            return
        try:
            intensities = self.spec.intensities(
                correct_dark_counts=False, correct_nonlinearity=False)
        except Exception as e:
            self.get_logger().error(f'Capture failed: {e}')
            return

        # Save raw intensity (baseline may not be set at collection time)
        self._publish_save(intensities, label='capture')

    def _publish_save(self, data: np.ndarray, label: str):
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        fname     = f'spect_data_{timestamp}_{label}.csv'

        save_msg              = SpectrometerData()
        save_msg.wavelengths  = self.wavelengths.tolist()
        save_msg.intensities  = data.tolist()
        save_msg.is_successful = True
        save_msg.filename     = fname
        self.save_pub.publish(save_msg)
        self.get_logger().info(f'Published save frame: {fname}')

    # ── Cleanup ───────────────────────────────────────────────────────────────────

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