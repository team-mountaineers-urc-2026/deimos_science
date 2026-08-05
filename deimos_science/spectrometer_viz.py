"""
spectrometer_viz.py  (runs on the BASE STATION)

Receives spectrometer data over ROS and writes CSVs locally to ~/spectrometer_readings/.
Files are only written when triggered by the GUI — never continuously.

Handles multiple CSV formats:
  - New 2-column:  wavelength_nm, intensity   (Collect Data one-shot)
  - New 2-column:  wavelength_nm, absorbance  (Save Snapshot absorbance file)
  - New 2-column:  wavelength_nm, intensity   (Save Snapshot raw file)
  - Old 4-column:  wavelength_nm, absorbance, baseline intensity, sample intensity
  - Legacy 2-row:  first row = wavelengths, second row = intensities

Topics subscribed:
  /spectrometer/collect_data       robot_interfaces/msg/SpectrometerData
      — one-shot raw reading from Collect Data button; writes one CSV
  /spectrometer/snapshot_data      robot_interfaces/msg/SpectrometerData
      — absorbance or raw snapshot; writes one CSV per message (two messages per press)
  /base_station/spectro_folder_req std_msgs/msg/Empty
      — GUI requesting the current file list
  /base_station/spectro_graph_gen  robot_interfaces/msg/StringArray
      — GUI requesting CSV contents for charting

Topics published:
  /base_station/spectro_graph_list robot_interfaces/msg/StringArray
      — list of CSV filenames in the data folder
  /spectrometer/historical         robot_interfaces/msg/SpectrometerData
      — CSV contents sent back to the GUI chart
"""

import rclpy
import csv
import os

from rclpy.node import Node
from std_msgs.msg import Empty
from robot_interfaces.msg import StringArray, SpectrometerData

# Column names treated as the Y value to plot, checked in priority order.
# Lowercase, stripped — headers are normalised before comparing.
VALUE_COLUMNS = ('absorbance', 'intensity', 'sample intensity')


class SpectrometerViz(Node):

    def __init__(self):
        super().__init__('spectrometer_viz')

        # All CSVs live in ~/spectrometer_readings/ — folder must already exist
        self.declare_parameter('data_folder', os.path.expanduser('~/spectrometer_readings'))
        self.data_folder = self.get_parameter('data_folder').get_parameter_value().string_value
        os.makedirs(self.data_folder, exist_ok=True)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            SpectrometerData, '/spectrometer/collect_data',
            self.on_collect_data, 10)
        self.create_subscription(
            SpectrometerData, '/spectrometer/snapshot_data',
            self.on_snapshot_data, 10)
        self.create_subscription(
            Empty, '/base_station/spectro_folder_req',
            self.send_folder, 10)
        self.create_subscription(
            StringArray, '/base_station/spectro_graph_gen',
            self.send_historical_data, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.folder_publisher = self.create_publisher(
            StringArray, '/base_station/spectro_graph_list', 10)
        self.historical_data_publisher = self.create_publisher(
            SpectrometerData, '/spectrometer/historical', 10)

        self.send_folder()
        self.get_logger().info(f'SpectrometerViz ready. Saving to: {self.data_folder}')

    # ── Collect Data: one-shot raw reading → single CSV ───────────────────────

    def on_collect_data(self, msg: SpectrometerData):
        """
        Triggered once per Collect Data button press.
        Writes a single 2-column CSV:
            wavelength_nm   intensity
        """
        if not msg.is_successful:
            self.get_logger().error('Received failed collect_data message, skipping.')
            return

        if not msg.wavelengths or not msg.intensities:
            self.get_logger().error('collect_data message has no data, skipping.')
            return

        fname = f'{msg.filename}.csv'
        fpath = os.path.join(self.data_folder, fname)

        try:
            with open(fpath, 'w', newline='') as f:
                writer = csv.writer(f, delimiter='\t')
                writer.writerow(['wavelength_nm', 'intensity'])
                for w, v in zip(msg.wavelengths, msg.intensities):
                    writer.writerow([f'{w:.4f}', f'{v:.6f}'])
            self.get_logger().info(f'Collect Data: wrote {fname}')
        except Exception as e:
            self.get_logger().error(f'Failed to write {fname}: {e}')
            return

        self.send_folder()

    # ── Snapshot: absorbance or raw CSV ───────────────────────────────────────

    def on_snapshot_data(self, msg: SpectrometerData):
        """
        Called twice per Save Snapshot press — once for absorbance, once for raw.
        Writes one 2-column CSV per call:
            wavelength_nm   absorbance   (for _absorbance files)
            wavelength_nm   intensity    (for _raw files)
        """
        if not msg.is_successful:
            self.get_logger().error('Received failed snapshot message, skipping.')
            return

        if not msg.wavelengths or not msg.intensities:
            self.get_logger().error('snapshot_data message has no data, skipping.')
            return

        fname = f'{msg.filename}.csv'
        fpath = os.path.join(self.data_folder, fname)

        value_header = 'absorbance' if '_absorbance' in msg.filename else 'intensity'

        try:
            with open(fpath, 'w', newline='') as f:
                writer = csv.writer(f, delimiter='\t')
                writer.writerow(['wavelength_nm', value_header])
                for w, v in zip(msg.wavelengths, msg.intensities):
                    writer.writerow([f'{w:.4f}', f'{v:.6f}'])
            self.get_logger().info(f'Snapshot: wrote {fname}')
        except Exception as e:
            self.get_logger().error(f'Failed to write {fname}: {e}')
            return

        self.send_folder()

    # ── File list ─────────────────────────────────────────────────────────────

    def send_folder(self, msg=None):
        """Scan the data folder and publish the sorted list of CSV filenames."""
        try:
            names = sorted(f for f in os.listdir(self.data_folder) if f.endswith('.csv'))
        except Exception as e:
            self.get_logger().error(f'Could not scan data folder: {e}')
            names = []
        self.folder_publisher.publish(StringArray(data=names))

    # ── Stream CSV contents to GUI chart ──────────────────────────────────────

    def send_historical_data(self, msg: StringArray):
        """Read requested CSVs and publish their contents for the GUI chart."""
        for filename in msg.data:
            filepath = os.path.join(self.data_folder, filename)

            if not os.path.exists(filepath):
                self.get_logger().error(f'File not found: {filename}')
                continue

            wavelengths, intensities = self._read_csv(filepath)
            intensities = self._smooth(intensities, window=5)

            if not wavelengths:
                self.get_logger().error(f'No data parsed from {filename} — skipping.')
                continue

            response = SpectrometerData()
            response.wavelengths = wavelengths
            response.intensities = intensities
            response.is_successful = True
            response.filename = filename
            self.get_logger().info(
                f'Publishing {filename} ({len(wavelengths)} points) to GUI.')
            self.historical_data_publisher.publish(response)

    # ── Smoother ──────────────────────────────────────────────────────────────

    def _smooth(self, values, window=10):
        """Apply a simple moving average to smooth noisy spectrometer data."""
        smoothed = []
        for i in range(len(values)):
            start = max(0, i - window // 2)
            end = min(len(values), i + window // 2 + 1)
            smoothed.append(sum(values[start:end]) / (end - start))
        return smoothed

    # ── CSV parser ────────────────────────────────────────────────────────────

    def _read_csv(self, filepath: str):
        """
        Robustly parse a CSV regardless of format.
        Returns (wavelengths, values) as lists of float, or ([], []) on error.

        Format detection
        ----------------
        1.  Header row: first cell is 'wavelength_nm' or 'wavelength'
              - Find first column matching VALUE_COLUMNS (priority order)
              - Fall back to column index 1 if nothing matches
              - Skip unparseable rows with a warning

        2.  No recognised header → legacy two-row format
              - Row 0: all wavelength values
              - Row 1: all intensity values
        """
        wavelengths = []
        values = []
        fname = os.path.basename(filepath)

        try:
            with open(filepath, 'r', newline='', encoding='utf-8-sig') as f:
                raw = f.read()

            lines = raw.splitlines()
            if not lines:
                self.get_logger().warn(f'{fname}: file is empty.')
                return [], []

            sample = raw[:2000]
            delimiter = '\t' if '\t' in sample else ','
            reader = csv.reader(lines, delimiter=delimiter)
            rows = [row for row in reader if any(cell.strip() for cell in row)]

            if not rows:
                self.get_logger().warn(f'{fname}: no non-empty rows found.')
                return [], []

            first_cell = rows[0][0].strip().lower().lstrip('\ufeff')

            # ── Headered format ───────────────────────────────────────────────
            if first_cell in ('wavelength_nm', 'wavelength'):
                headers = [h.strip().lower() for h in rows[0]]
                self.get_logger().info(f'{fname}: detected headers: {headers}')

                value_col = None
                for candidate in VALUE_COLUMNS:
                    if candidate in headers:
                        value_col = headers.index(candidate)
                        self.get_logger().info(
                            f'{fname}: using column "{candidate}" (index {value_col})')
                        break

                if value_col is None:
                    value_col = 1
                    self.get_logger().warn(
                        f'{fname}: no recognised value column in {headers}, '
                        f'falling back to column index 1.')

                skipped = 0
                for row_num, row in enumerate(rows[1:], start=2):
                    if len(row) <= value_col:
                        skipped += 1
                        continue
                    wl_str = row[0].strip()
                    val_str = row[value_col].strip()
                    if not wl_str or not val_str:
                        skipped += 1
                        continue
                    try:
                        wavelengths.append(float(wl_str))
                        values.append(float(val_str))
                    except ValueError:
                        self.get_logger().warn(
                            f'{fname} row {row_num}: could not parse '
                            f'"{wl_str}" / "{val_str}" as float — skipping row.')
                        skipped += 1

                if skipped:
                    self.get_logger().warn(f'{fname}: skipped {skipped} unparseable rows.')

            # ── Legacy two-row format ─────────────────────────────────────────
            else:
                self.get_logger().info(
                    f'{fname}: no header detected, trying legacy two-row format.')
                if len(rows) < 2:
                    self.get_logger().error(
                        f'{fname}: legacy format needs at least 2 rows, found {len(rows)}.')
                    return [], []

                try:
                    wavelengths = [float(v.strip()) for v in rows[0] if v.strip()]
                    values      = [float(v.strip()) for v in rows[1] if v.strip()]
                except ValueError as e:
                    self.get_logger().error(
                        f'{fname}: error parsing legacy two-row format: {e}')
                    return [], []

                if len(wavelengths) != len(values):
                    self.get_logger().warn(
                        f'{fname}: wavelength row has {len(wavelengths)} values '
                        f'but intensity row has {len(values)} — truncating to shorter.')
                    n = min(len(wavelengths), len(values))
                    wavelengths = wavelengths[:n]
                    values = values[:n]

        except Exception as e:
            self.get_logger().error(f'Unexpected error reading {fname}: {e}')
            return [], []

        self.get_logger().info(f'{fname}: parsed {len(wavelengths)} points successfully.')
        return wavelengths, values


def main(args=None):
    rclpy.init(args=args)
    node = SpectrometerViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()