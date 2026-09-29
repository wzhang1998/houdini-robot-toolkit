"""TouchDesigner's canvas preview without TouchDesigner: pixel_scan's image
and canvas_sim.glsl ported to numpy (TD-ROBOT-UVSCAN td-modules/pixel_scan),
for Isaac videos made headless (run_show --canvas-sim). TD stays the one to
tune against; this follows its defaults (PIXEL_SCAN) and is not checked
against it frame by frame.

    img = prepare_image(BANANA)                    # Cols x Rows levels 0..1, rows top first
    m = CanvasModel(90, 90)
    m.step(dt, u, leds_at(img, u) * MASTER)        # each frame; u -1 outside the scan
    w, h, rgb = m.view()                           # canvas_link's image: the paper as TD's monitor shows it

    python scripts/canvas_model.py --self-test
"""

import sys

import numpy as np

BANANA = r"C:\Program Files\Derivative\TouchDesigner\Samples\Map\Banana.tif"   # pixel_scan's Pattern, for now

# pixel_scan's parameters as the project has them (2026-09-29)
PIXEL_SCAN = {"cols": 60, "rows": 60, "invert": True, "gamma": 1.0, "master": 0.8, "flip": False,
              "led_gap_m": 0.06, "travel_m": 1.0, "span_m": 1.0, "offset_m": 0.0, "nleds": 60,
              "pitch_m": 1.0 / 60, "halflife_s": 60.0, "colour_time_s": 6.0, "display_gain": 10.0}
PAPER = (0.96, 0.95, 0.92)                         # paper_to_violet_keys: 0
VIOLET = (0.35, 0.1, 0.6)                          # ... 1


def prepare_image(path, cols=PIXEL_SCAN["cols"], rows=PIXEL_SCAN["rows"], invert=PIXEL_SCAN["invert"],
                  gamma=PIXEL_SCAN["gamma"]):
    """The LED levels an image gives, as pixel_scan makes them: premultiplied
    by its alpha (TD's Movie File In), fitted to its height (Fit TOP
    `fitvert`, the sides cropped or padded black), cols x rows, luminance,
    inverted (dark parts expose the paper), gamma. Rows top first."""
    from PIL import Image
    im = Image.open(path).convert("RGBA")
    a = np.asarray(im, dtype=np.float32) / 255.0
    rgb = a[:, :, :3] * a[:, :, 3:4]
    im = Image.fromarray((rgb * 255.0 + 0.5).astype(np.uint8), "RGB")
    w = max(1, int(round(im.width * rows / float(im.height))))
    im = im.resize((w, rows), Image.BILINEAR)
    out = Image.new("RGB", (cols, rows))
    out.paste(im, ((cols - w) // 2, 0))
    lum = np.asarray(out, dtype=np.float32) / 255.0 @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    if invert:
        lum = 1.0 - lum
    return np.clip(lum, 0.0, 1.0) ** (1.0 / max(gamma, 1e-6))


def leds_at(img, u, flip=PIXEL_SCAN["flip"]):
    """The strip's levels at u (0..1 down the scan): the image's row at u from
    the top (pixel_scan `column`, the strip level), LED 0 at its left unless
    flip. Zeros outside the scan."""
    if not 0.0 <= u <= 1.0:
        return np.zeros(img.shape[1], dtype=np.float32)
    row = img[min(img.shape[0] - 1, int(u * img.shape[0]))]
    return row[::-1] if flip else row


class CanvasModel:
    """canvas_sim.glsl on a w x h grid, rows top first, for a pass down: under
    the strip's band (Gaussian, band_m wide at half power, of the scan's
    travel) the paper colours towards 1 (time constant colour_time_s at full
    light); everywhere it fades back (half-life halflife_s)."""

    def __init__(self, w=90, h=90, p=PIXEL_SCAN):
        self.w, self.h, self.p = w, h, p
        self.c = np.zeros((h, w), dtype=np.float32)
        self.pass_ = (np.arange(h, dtype=np.float32) + 0.5) / h            # u of each row (0 top)
        along = (np.arange(w, dtype=np.float32) + 0.5) / w
        strip = p["nleds"] * p["pitch_m"]
        s = along * (p["span_m"] / strip) + ((strip - p["span_m"]) / 2.0 - p["offset_m"]) / strip
        self.s = 1.0 - s if p["flip"] else s                              # each column's place along the strip
        self.band = (1.5 * p["led_gap_m"] + 0.005) / p["travel_m"]

    def step(self, dt, u, leds):
        """Advance dt s with the strip at u (-1: off the paper) lit at leds (0..1, LED 0 first)."""
        if u >= -0.5 and np.any(leds):
            n = len(leds)
            level = np.interp(self.s * n - 0.5, np.arange(n), leds, left=0.0, right=0.0)
            level[(self.s < 0.0) | (self.s > 1.0)] = 0.0
            sigma = max(self.band, 1e-4) / 2.3548
            g = np.exp(-0.5 * ((self.pass_ - u) / sigma) ** 2)
            light = g[:, None] * level[None, :]
            self.c += dt * light * (1.0 - self.c) / max(self.p["colour_time_s"], 1e-3)
        self.c *= 2.0 ** (-dt / max(self.p["halflife_s"], 1e-3))

    def view(self):
        """(w, h, RGB8 bytes rows top first): the paper as TD's canvas_view
        shows it (display gain, then paper to violet)."""
        k = np.clip(self.c * self.p["display_gain"], 0.0, 1.0)[:, :, None]
        rgb = np.array(PAPER, dtype=np.float32) + (np.array(VIOLET, dtype=np.float32) - np.array(PAPER)) * k
        return self.w, self.h, (np.clip(rgb, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8).tobytes()


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    m = CanvasModel(20, 40)
    for _ in range(30):
        m.step(1.0 / 30, 0.25, np.ones(60))
    top, mid, low = m.c[10].mean(), m.c[20].mean(), m.c[35].mean()
    check("the paper colours under the strip's band (u 0.25: row 10 of 40), not far from it",
          top > 0.1 and mid < top * 0.01 and low < 1e-6, (round(float(top), 4), float(mid), float(low)))
    c0 = float(m.c[10].mean())
    for _ in range(60 * 30):
        m.step(1.0 / 30, -1.0, np.zeros(60))
    check("... and fades back with the half-life (60 s: half)", abs(m.c[10].mean() / c0 - 0.5) < 0.01,
          round(float(m.c[10].mean() / c0), 4))
    m2 = CanvasModel(20, 40)
    leds = np.zeros(60)
    leds[:30] = 1.0
    for _ in range(30):
        m2.step(1.0 / 30, 0.5, leds)
    check("LED 0's half lights the left half (as seen from the robot)", m2.c[20, :9].min() > 0.1 and m2.c[20, 11:].max() < 0.02,
          (float(m2.c[20, :9].min()), float(m2.c[20, 11:].max())))
    w, h, rgb = CanvasModel(4, 2).view()
    check("fresh paper is paper white in the view", (w, h) == (4, 2) and rgb[:3] == bytes(int(x * 255 + 0.5) for x in PAPER))
    img = np.array([[0.0, 1.0], [0.5, 0.25]])
    check("the strip at u shows the image's row at u from the top; dark outside the scan",
          list(leds_at(img, 0.1)) == [0.0, 1.0] and list(leds_at(img, 0.9)) == [0.5, 0.25]
          and not leds_at(img, -1.0).any())
    try:
        banana = prepare_image(BANANA)
        check("the banana: 60 x 60 levels, its dark background lit (premultiplied, inverted), the fruit darker",
              banana.shape == (60, 60) and banana[2, 2] > 0.9 and banana.min() < 0.5,
              (banana.shape, float(banana[2, 2]), float(banana.min())))
    except OSError as e:
        print("skip  the banana (%s)" % e)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    print(__doc__)
