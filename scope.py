import sys
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sounddevice as sd
import pyaudio

# -----------------------------
# SETTINGS
# -----------------------------
BUFFER_SIZE = 2048       # Samples per audio callback
UPDATE_MS = 30           # GUI refresh rate (~33 FPS)
p = pyaudio.PyAudio()

for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    print(i, dev['name'], dev['maxInputChannels'])



DEVICE_INDEX = int(input("device: "))       # ScopeSink.monitor
TRIGGER_LEVEL = 0.05     # Rising edge trigger threshold
Y_RANGE = 0.1            # Amplitude vertical zoom

# -----------------------------
# AUDIO SETUP
# -----------------------------
device_info = sd.query_devices(DEVICE_INDEX)
SAMPLE_RATE = int(device_info['default_samplerate'])
CHANNELS = min(device_info['max_input_channels'], 2)

audio_buffer = np.zeros(BUFFER_SIZE, dtype=np.float32)

def audio_callback(indata, frames, time, status):
    global audio_buffer
    if status:
        print(status)
    # Take first channel only
    audio_buffer = indata[:, 0]

stream = sd.InputStream(
    samplerate=SAMPLE_RATE,
    blocksize=BUFFER_SIZE,
    channels=CHANNELS,
    dtype='float32',
    callback=audio_callback,
    device=DEVICE_INDEX
)
stream.start()

# -----------------------------
# TRIGGER FUNCTION
# -----------------------------
def find_trigger(data, level=TRIGGER_LEVEL):
    """Find the first rising edge crossing of trigger level."""
    for i in range(len(data)-1):
        if data[i] < level and data[i+1] >= level:
            return i
    return 0

# -----------------------------
# PYQTGRAPH WINDOW
# -----------------------------
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Synth Oscilloscope")
win.resize(1000, 400)

plot = win.addPlot(title="Waveform")
plot.setYRange(-Y_RANGE, Y_RANGE)
plot.showGrid(x=True, y=True)
plot.setLabel('left', 'Amplitude')
plot.setLabel('bottom', 'Samples')
curve = plot.plot(pen='y')

# -----------------------------
# UPDATE FUNCTION
# -----------------------------
def update():
    # Find trigger point
    idx = find_trigger(audio_buffer)
    # Align waveform to trigger
    triggered = np.roll(audio_buffer, -idx)
    curve.setData(triggered)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(UPDATE_MS)

win.show()
sys.exit(app.exec_())
