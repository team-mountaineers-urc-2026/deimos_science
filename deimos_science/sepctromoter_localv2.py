import seabreeze
seabreeze.use('cseabreeze')

from seabreeze.spectrometers import Spectrometer, list_devices
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from matplotlib.path import Path
import csv
import time
import select
import sys

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

devices = list_devices()
if not devices:
	print("No spectrometer detected.")
	print("Check USB connection, power, and permissions, then try again.")
	raise SystemExit(1)

spec = Spectrometer(devices[0])
print("Connected to:", spec.model)

VISIBLE_MIN_NM = 380.0
VISIBLE_MAX_NM = 780.0


cmfs = colour.MSDS_CMFS["CIE 1931 2 Degree Standard Observer"]
illuminant = colour.SDS_ILLUMINANTS["D65"]


def wavelengths_to_srgb(wavelengths_nm):
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


# Less exact fallback (uncomment to use if colour-science is unavailable):
# def wavelengths_to_srgb(wavelengths_nm):
# 	def wavelength_to_rgb_approx(wl):
# 		if wl < 380 or wl > 780:
# 			return np.array([0.0, 0.0, 0.0])
# 		if wl < 440:
# 			r = -(wl - 440) / (440 - 380)
# 			g = 0.0
# 			b = 1.0
# 		elif wl < 490:
# 			r = 0.0
# 			g = (wl - 440) / (490 - 440)
# 			b = 1.0
# 		elif wl < 510:
# 			r = 0.0
# 			g = 1.0
# 			b = -(wl - 510) / (510 - 490)
# 		elif wl < 580:
# 			r = (wl - 510) / (580 - 510)
# 			g = 1.0
# 			b = 0.0
# 		elif wl < 645:
# 			r = 1.0
# 			g = -(wl - 645) / (645 - 580)
# 			b = 0.0
# 		else:
# 			r = 1.0
# 			g = 0.0
# 			b = 0.0
# 		return np.array([r, g, b])
# 	return np.stack([wavelength_to_rgb_approx(wl) for wl in wavelengths_nm], axis=0)


def build_clip_path(x, y, y0):
	upper = np.column_stack([x, y])
	lower = np.column_stack([x[::-1], np.full_like(x, y0)])
	verts = np.vstack([upper, lower, upper[:1]])
	codes = [Path.MOVETO] + [Path.LINETO] * (len(verts) - 2) + [Path.CLOSEPOLY]
	return Path(verts, codes)




def save_snapshot_csv(path, wavelengths_nm, absorbance,baseline_intensity,sample_intensity):
	with open(path, "w", newline="") as csv_file:
		writer = csv.writer(csv_file)
		writer.writerow(["wavelength_nm", "absorbance","baseline intensity","sample intensity"])
		for wl, ab, bi,si in zip(wavelengths_nm, absorbance, baseline_intensity, sample_intensity):
			writer.writerow([wl, ab, bi,si])

try:
	
	# Use a fixed integration time for all acquisitions.
	spec.integration_time_micros(100_000)
	wavelengths = spec.wavelengths()
	spectrum_rgb = wavelengths_to_srgb(wavelengths)

	plt.ion()
	fig, ax = plt.subplots(figsize=(10, 5))
	intensities = spec.intensities(correct_dark_counts=False, correct_nonlinearity=False)
	background = ax.imshow(
		spectrum_rgb[np.newaxis, :, :],
		extent=[wavelengths.min(), wavelengths.max(), 0, 1],
		aspect="auto",
		origin="lower",
		zorder=0,
	)
	line, = ax.plot(wavelengths, intensities, zorder=2)
	ax.set_xlabel("Wavelength (nm)")
	ax.set_ylabel("Intensity (counts)")
	ax.set_title("Live spectrum")
	ax.grid(True)
	fig.tight_layout()
	fig.show()

	ax.relim()
	ax.autoscale_view(scalex=False, scaley=True)
	y_min, y_max = ax.get_ylim()
	background.set_extent([wavelengths.min(), wavelengths.max(), y_min, y_max])
	background.set_clip_path(build_clip_path(wavelengths, intensities, y_min), ax.transData)

	frame_buffer = deque(maxlen=10)
	baseline = None
	baseline_requested = False
	last_report_time = time.monotonic()
	print("Streaming... Press Enter to capture baseline from last 10 frames.")
	print("After baseline: type 's' + Enter to save a CSV snapshot.")
	print("Close the plot window or press Ctrl+C to stop.")
	while plt.fignum_exists(fig.number):
		intensities = spec.intensities(correct_dark_counts=False, correct_nonlinearity=False)
		frame_buffer.append(intensities)

		if baseline is None:
			current_y = intensities
			line.set_ydata(current_y)
			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				sys.stdin.readline()
				baseline_requested = True
				print("Capturing baseline from last 10 frames...", flush=True)
			if baseline_requested and len(frame_buffer) >= frame_buffer.maxlen:
				baseline = np.mean(np.stack(frame_buffer, axis=0), axis=0)
				baseline_requested = False
				ax.set_ylabel("Absorbance (AU)")
				ax.set_title("Live absorbance")
		else:
			eps = 1e-12
			ratio = intensities / np.clip(baseline, eps, None)
			absorbance = -np.log10(np.clip(ratio, eps, None))
			current_y = absorbance
			line.set_ydata(current_y)
			
			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				command = sys.stdin.readline().strip().lower()
				if command in {"s", "save"}:
					name = input("Snapshot filename (blank to cancel): ").strip()
					if name:
						if not name.lower().endswith(".csv"):
							name = f"{name}.csv"
						save_snapshot_csv(name, wavelengths, absorbance,baseline,intensities)
						print(f"Saved snapshot to {name}", flush=True)


		if baseline is None:
			ax.relim()
			ax.autoscale_view(scalex=False, scaley=True)
			y_min, y_max = ax.get_ylim()
		else:
			# Explicit autoscaling for absorbance mode
			y_min = np.min(current_y)
			y_max = np.max(current_y)

			# Add padding so the trace is not touching edges
			padding = max((y_max - y_min) * 0.1, 0.01)

			ax.set_ylim(y_min - padding, y_max + padding)

		background.set_extent([wavelengths.min(), wavelengths.max(), y_min, y_max])
		background.set_clip_path(build_clip_path(wavelengths, current_y, y_min), ax.transData)
		fig.canvas.draw_idle()
		plt.pause(0.05)
finally:
	spec.close()