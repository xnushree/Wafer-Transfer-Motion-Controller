"""Record the RViz window to an animated GIF (no ffmpeg needed).

Grabs the X display directly with python-xlib, so it works on machines such
as The Construct where ffmpeg is not installed and Pillow cannot read the
16-bit display. Install python-xlib somewhere importable first, e.g.

    pip install --target /tmp/xlibpkg python-xlib
    PYTHONPATH=/tmp/xlibpkg python3 record_rviz_gif.py out.gif 18

Arguments: output file, duration in seconds (default 15), frames per second
(default 10). The Displays panel, toolbars and status bar of RViz are cropped
off so only the 3D view is kept.
"""

import sys
import time

import numpy as np
from PIL import Image
from Xlib import X, display


OUTPUT = sys.argv[1] if len(sys.argv) > 1 else "rviz.gif"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
FPS = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
WIDTH = 640                                   # output width in pixels

# Fractions of the RViz window to cut off: left panel, top bars, bottom bar
CROP_LEFT, CROP_TOP, CROP_RIGHT, CROP_BOTTOM = 0.245, 0.075, 0.0, 0.045


def find_window(window, name):

    try:
        title = window.get_wm_name() or ""
    except Exception:
        title = ""

    if name in str(title):
        return window

    for child in window.query_tree().children:
        found = find_window(child, name)
        if found is not None:
            return found

    return None


def to_rgb(raw, width, height, depth):

    if depth == 16:
        pixels = np.frombuffer(raw, dtype=np.uint16)
        pixels = pixels[: len(pixels) // height * height].reshape(height, -1)[:, :width]
        r = ((pixels >> 11) & 0x1F) * 255 // 31
        g = ((pixels >> 5) & 0x3F) * 255 // 63
        b = (pixels & 0x1F) * 255 // 31
        return np.dstack([r, g, b]).astype(np.uint8)

    pixels = np.frombuffer(raw, dtype=np.uint8)
    pixels = pixels[: len(pixels) // height * height].reshape(height, -1)
    pixels = pixels[:, : width * 4].reshape(height, width, 4)
    return pixels[:, :, [2, 1, 0]]


screen = display.Display().screen()
root = screen.root
rviz = find_window(root, "RViz")

if rviz is None:
    sys.exit("No RViz window found on this display")

geometry = rviz.get_geometry()
origin = rviz.translate_coords(root, 0, 0)
x0, y0 = -origin.x, -origin.y
w, h = geometry.width, geometry.height

left, top = int(w * CROP_LEFT), int(h * CROP_TOP)
right, bottom = w - int(w * CROP_RIGHT), h - int(h * CROP_BOTTOM)

print(f"RViz window {w}x{h} at ({x0}, {y0}), depth {screen.root_depth}; "
      f"recording {DURATION:.0f} s at {FPS:.0f} fps")

frames = []
period = 1.0 / FPS
next_time = time.time()
end_time = next_time + DURATION

while time.time() < end_time:

    image = root.get_image(x0, y0, w, h, X.ZPixmap, 0xFFFFFFFF)
    rgb = to_rgb(image.data, w, h, screen.root_depth)[top:bottom, left:right]

    frame = Image.fromarray(rgb)
    height = int(frame.height * WIDTH / frame.width)
    frames.append(frame.resize((WIDTH, height), Image.LANCZOS))

    next_time += period
    time.sleep(max(0.0, next_time - time.time()))

frames[0].save(
    OUTPUT,
    save_all=True,
    append_images=frames[1:],
    duration=int(1000 / FPS),
    loop=0,
    optimize=True
)

print(f"Saved {OUTPUT}: {len(frames)} frames")
