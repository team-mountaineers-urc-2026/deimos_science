import seabreeze
seabreeze.use('cseabreeze')

from seabreeze.spectrometers import Spectrometer, list_devices
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import select
import sys

devices = list_devices()
if not devices:
	print("No spectrometer detected.")
	print("Check USB connection, power, and permissions, then try again.")
	raise SystemExit(1)

spec = Spectrometer(devices[0])
print("Connected to:", spec.model)

try:
	
	# Use a fixed integration time for all acquisitions.
	spec.integration_time_micros(100_000)
	wavelengths = spec.wavelengths()

	plt.ion()
	fig, ax = plt.subplots(figsize=(10, 5))
	intensities = spec.intensities(correct_dark_counts=False, correct_nonlinearity=False)
	line, = ax.plot(wavelengths, intensities)
	ax.set_xlabel("Wavelength (nm)")
	ax.set_ylabel("Intensity (counts)")
	ax.set_title("Live spectrum")
	ax.grid(True)
	fig.tight_layout()
	fig.show()

	frame_buffer = deque(maxlen=10)
	baseline = None
	print("Streaming... Press Enter to capture baseline from last 10 frames.")
	print("Close the plot window or press Ctrl+C to stop.")
	while plt.fignum_exists(fig.number):
		intensities = spec.intensities(correct_dark_counts=False, correct_nonlinearity=False)
		frame_buffer.append(intensities)

		if baseline is None:
			line.set_ydata(intensities)
			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				sys.stdin.readline()
				if len(frame_buffer) < frame_buffer.maxlen:
					print("Collecting frames for baseline...", flush=True)
				else:
					baseline = np.mean(np.stack(frame_buffer, axis=0), axis=0)
					ax.set_ylabel("Absorbance (AU)")
					ax.set_title("Live absorbance")
		else:
			eps = 1e-12
			ratio = intensities / np.clip(baseline, eps, None)
			absorbance = -np.log10(np.clip(ratio, eps, None))
			line.set_ydata(absorbance)
		ax.relim()
		ax.autoscale_view(scalex=False, scaley=True)
		fig.canvas.draw_idle()
		plt.pause(0.05)
finally:
	spec.close()