"""TouchDesigner's canvas preview (TD-ROBOT-UVSCAN pixel_scan's canvas_view:
the photochromic paper as the scan leaves it) sent to Isaac Sim over UDP, one
small image a packet:

    b"CNV1" | width u16 | height u16 | RGB8, rows top first      (little endian)

    rx = Receiver(6457)
    img = rx.poll()            # (w, h, rgb bytes) of the newest packet; None before any

    python scripts/canvas_link.py --self-test
    python scripts/canvas_link.py --send-test 6457     stand in for TD: a test image (which way is up and left)

Standard library only.
"""

import socket
import struct
import sys
import time

from udp_latest import LatestReceiver

MAGIC = b"CNV1"
PREVIEW_PORT = 6457
MAX_BYTES = 65000                   # one UDP datagram


def pack(w, h, rgb):
    """A packet of a w x h image, rgb: w*h*3 bytes, rows top first."""
    if len(rgb) != w * h * 3:
        raise ValueError("%d bytes for a %d x %d RGB image (want %d)" % (len(rgb), w, h, w * h * 3))
    return MAGIC + struct.pack("<HH", w, h) + bytes(rgb)


def parse(packet):
    """(w, h, rgb bytes) of a canvas packet; None for anything else."""
    if len(packet) < 8 or packet[:4] != MAGIC:
        return None
    w, h = struct.unpack("<HH", packet[4:8])
    rgb = packet[8:]
    if w == 0 or h == 0 or len(rgb) != w * h * 3:
        return None
    return w, h, rgb


class Receiver(LatestReceiver):
    """Canvas packets on a UDP port, never blocking: poll() gives the newest
    (w, h, rgb); age() -- TD stopped sending?"""

    def __init__(self, port=PREVIEW_PORT, host="127.0.0.1"):
        super().__init__(port, parse, host, bufsize=MAX_BYTES + 16)


def test_image(w=90, h=90):
    """What --send-test sends: the top half red on the left, green on the
    right; the bottom paper white to violet, left to right (which way is up
    and left as seen from the robot)."""
    px = bytearray()
    for r in range(h):
        for c in range(w):
            if r < h // 2:
                px += bytes([230, 40, 40]) if c < w // 2 else bytes([40, 200, 60])
            else:
                k = c / float(max(1, w - 1))
                px += bytes([int(245 - 90 * k), int(240 - 170 * k), int(235 - 10 * k)])
    return bytes(px)


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    rgb = bytes(range(2 * 3 * 3))
    check("a packet gives back its size and pixels", parse(pack(3, 2, rgb)) == (3, 2, rgb))
    check("a short, foreign or truncated packet is not a canvas",
          parse(b"CNV1") is None and parse(b"Art-Net\x00" + bytes(20)) is None and parse(pack(3, 2, rgb)[:-1]) is None)
    try:
        pack(3, 2, rgb[:-1])
        bad = False
    except ValueError:
        bad = True
    check("packing the wrong number of bytes is refused", bad)
    check("a 216 x 90 canvas (a 2.4 : 1 area) fits one datagram", len(pack(216, 90, bytes(216 * 90 * 3))) <= MAX_BYTES)
    rx = Receiver(port=0)
    port = rx.sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    check("nothing yet: None", rx.poll() is None and rx.age() is None)
    tx.sendto(pack(2, 1, bytes([1, 2, 3, 4, 5, 6])), ("127.0.0.1", port))
    tx.sendto(pack(90, 90, bytes([7]) * (90 * 90 * 3)), ("127.0.0.1", port))
    for _ in range(50):
        got = rx.poll()
        if rx.packets >= 2:
            break
        time.sleep(0.01)
    check("the receiver keeps the newest image", got is not None and got[:2] == (90, 90) and got[2][0] == 7,
          got[:2] if got else None)
    tx.close()
    rx.close()
    img = test_image(4, 2)
    check("the test image: top left red, top right green, bottom paper to violet",
          img[:3] == bytes([230, 40, 40]) and img[9:12] == bytes([40, 200, 60]) and img[12] > img[21], img[12:24])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    if "--send-test" in sys.argv:
        k = sys.argv.index("--send-test")
        port = int(sys.argv[k + 1]) if len(sys.argv) > k + 1 and sys.argv[k + 1].isdigit() else PREVIEW_PORT
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packet = pack(90, 90, test_image())
        print("sending the test image to 127.0.0.1:%d, 15 a second (Ctrl+C to stop)" % port)
        try:
            while True:
                tx.sendto(packet, ("127.0.0.1", port))
                time.sleep(1 / 15.0)
        except KeyboardInterrupt:
            pass
        sys.exit(0)
    print(__doc__)
