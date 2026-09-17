from vispy import scene, app
import numpy as np
import struct
import serial

canvas = scene.SceneCanvas(show=True)
view = canvas.central_widget.add_view()

# Real camera object, not string
cam = scene.cameras.TurntableCamera(
    fov=70,
    distance=1000,
    elevation=20,
    azimuth=-60,
)
view.camera = cam

# Fix clipping
cam._near = 0.1
cam._far = 1e7
cam.view_changed()

moving  = scene.visuals.Markers()
moving2 = scene.visuals.Markers()
moving3 = scene.visuals.Markers()
view.add(moving)
view.add(moving2)
view.add(moving2)

static = scene.visuals.Markers()
view.add(static)

# Dotted line object for connecting static points
static_lines = scene.visuals.Line(method='gl')
view.add(static_lines)

moving_lines = scene.visuals.Line(method='gl')
view.add(moving_lines)

axes = scene.visuals.XYZAxis()
grid = scene.visuals.GridLines(scale=(1, 1), color='w',
                               grid_bounds=(-1000, 1000, -1000, 1000),
                               border_width=0.1)
# Add to your existing canvas/view:
L = 500.0   # axis length

# X axis (red)
x_line = scene.visuals.Line(
    pos=np.array([[0, 0, 0], [L, 0, 0]], dtype=np.float32),
    color=(1, 0, 0, 1),
    width=2
)
view.add(x_line)

# Y axis (green)
y_line = scene.visuals.Line(
    pos=np.array([[0, 0, 0], [0, L, 0]], dtype=np.float32),
    color=(0, 1, 0, 1),
    width=2
)
view.add(y_line)

# Z axis (blue)
z_line = scene.visuals.Line(
    pos=np.array([[0, 0, 0], [0, 0, L]], dtype=np.float32),
    color=(0, 0, 1, 1),
    width=2
)
view.add(z_line)

# fifo = open("/tmp/coords", "rb")
ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0.1)
ser.reset_input_buffer()

def read_exactly(n):
    """Read exactly n bytes from serial, waiting until enough bytes are available."""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            continue  # no data yet, try again
        buf.extend(chunk)
    return bytes(buf)

pts = np.empty((5,3))
moving_old1 = np.zeros((1,3), dtype=np.float32)
moving_old2 = np.zeros((1,3), dtype=np.float32)
def update(ev):
    global pts
    cmd = read_exactly(1)
    if not cmd:
        return

    if cmd == b'I':
        # read 5 anchor coordinate floats (3 axes -> 3 floats -> 12 bytes/anchor -> 5 anchors -> 60 bytes)
        raw = read_exactly(60)
        if raw is None:
            return
        
        pts = np.frombuffer(raw, dtype=np.float32).reshape(5, 3)
        static.set_data(pts, face_color='red', size=16)

        # Fully connected line set (complete graph)
        segs = []
        n = len(pts)
        for i in range(n):
            for j in range(i + 1, n):
                segs.append(pts[i])
                segs.append(pts[j])

        segments = np.array(segs, dtype=np.float32)

        static_lines.set_data(
            segments,
            color=(1, 0, 0, 0.3),   # transparent red
            width=1,
            connect='segments'
        )

    elif cmd == b'U':
        # 1 point: 3 floats = 12 bytes
        raw = read_exactly(12)
        if raw is None:
            return

        global moving_old1
        global moving_old2
        x, y, z = struct.unpack('<fff', raw)
        # print(x, y, z)
        moving_point = np.array([[x, y, z]], dtype=np.float32)
        moving.set_data(moving_point, face_color=(0, 1.0, 0, 1.0), size=16)
        moving2.set_data(moving_old1, face_color=(0, 1.0, 0, 0.4), size=16)
        moving3.set_data(moving_old2, face_color=(0, 1.0, 0, 0.1), size=16)
        moving_old1 = moving_point
        moving_old2 = moving_old1

        # Draw green lines from this moving point to all static points
        segs = []
        for i in range(5):
            segs.append(moving_point[0])
            segs.append(pts[i])

        segments = np.array(segs, dtype=np.float32)

        moving_lines.set_data(
            segments,
            color=(0.5, 1, 0.5, 0.5),   # transparent green
            width=1,
            connect='segments'
        )

    else:
        # ignore unknown commands
        return

# Timer to rotate
def rotate(event):
    view.camera.azimuth += 0.05   # horizontal rotation

timer1 = app.Timer(interval=0.015, connect=update, start=True)
timer2 = app.Timer(interval=0.015, connect=rotate, start=True)
app.run()