"""The LED strip's levels as TouchDesigner sends them: Art-Net (ArtDmx), one
universe, R G B of each LED (TD-ROBOT-UVSCAN pixel_scan's DMX Out CHOP; the
strip's three dies per LED are the same 395 nm UV, so a LED's level is its
brightest channel). A preview in Isaac Sim listens for a second DMX Out CHOP
aimed at it:

    rx = Receiver(6455)            # TD: DMX Out CHOP, Art-Net, 127.0.0.1, Network Port 6455
    lv = rx.poll()                 # each frame: 60 levels 0..1, the latest packet's; None before any

    python scripts/artnet.py --self-test
    python scripts/artnet.py --listen 6455      print what arrives (a check of TD's output)
    python scripts/artnet.py --send-test 6455   stand in for TD: a comet along the strip (Isaac without TD)

Standard library only.
"""

import socket
import struct
import sys
import time

from udp_latest import LatestReceiver

ID = b"Art-Net\x00"
OP_DMX = 0x5000
PREVIEW_PORT = 6455                  # not Art-Net's 6454: TD and a real node may hold that one
CEILING_PORT = 6458                  # TD ceiling_light's preview: one dimmer, 8 bit on channel 1
LEDS = 60


def artdmx(universe, data, sequence=0):
    """An ArtDmx packet (a test sender's; TD makes the real ones)."""
    data = bytes(data)
    if len(data) % 2:
        data += b"\x00"                                  # the spec wants an even length
    return (ID + struct.pack("<H", OP_DMX) + struct.pack(">H", 14) + bytes([sequence & 0xFF, 0])
            + struct.pack("<H", universe) + struct.pack(">H", len(data)) + data)


def parse(packet):
    """(universe, channel bytes) of an ArtDmx packet; None for anything else
    (a poll, a short or foreign packet)."""
    if len(packet) < 18 or packet[:8] != ID or struct.unpack("<H", packet[8:10])[0] != OP_DMX:
        return None
    universe = struct.unpack("<H", packet[14:16])[0]
    n = struct.unpack(">H", packet[16:18])[0]
    return universe, packet[18:18 + n]


def levels(data, n=LEDS, start=0, per_led=3):
    """n levels 0..1 from channel bytes: LED i is channels start + i*per_led
    ... (+per_led), its brightest; channels past the data are dark."""
    out = []
    for i in range(n):
        ch = data[start + i * per_led:start + (i + 1) * per_led]
        out.append(max(ch) / 255.0 if ch else 0.0)
    return out


def dimmer(data):
    """A dimmer's level 0..1 from channel bytes: channel 1, 8 bit (TD ceiling_light's preview; TD sends the
    universe whole); None for none."""
    return data[0] / 255.0 if data else None


class DimmerReceiver(LatestReceiver):
    """The ceiling's level as TD ceiling_light previews it (ArtDmx, one universe, never blocking)."""

    def __init__(self, port=CEILING_PORT, universe=0, host="127.0.0.1"):
        self.universe = universe
        super().__init__(port, self._level, host, bufsize=1024)

    def _level(self, packet):
        got = parse(packet)
        return dimmer(got[1]) if got and got[0] == self.universe else None


class Receiver(LatestReceiver):
    """ArtDmx for one universe on a UDP port, never blocking: poll() gives the
    newest packet's n levels; age() -- TD stopped sending?"""

    def __init__(self, port=PREVIEW_PORT, universe=0, host="127.0.0.1", n=LEDS):
        self.universe, self.n = universe, n
        super().__init__(port, self._levels, host, bufsize=1024)

    def _levels(self, packet):
        got = parse(packet)
        return levels(got[1], self.n) if got and got[0] == self.universe else None


def test_pattern(t, n=LEDS):
    """What --send-test sends at time t: a comet with a tail running along the
    strip, 20 LEDs a second, and LED 0's end always a little lit (which end is 0)."""
    head = (t * 20.0) % n
    return [max(max(0.0, 1.0 - abs(i - head) / 8.0) if i <= head else 0.0, 0.15 if i < 5 else 0.0) for i in range(n)]


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    data = bytes([0, 0, 0, 255, 255, 255, 0, 128, 0])
    check("an ArtDmx packet gives its universe and channels (padded to an even length)",
          parse(artdmx(3, data)) == (3, data + b"\x00"))
    check("anything else is not DMX (a poll, a short packet, another protocol)",
          parse(ID + struct.pack("<H", 0x2000) + bytes(10)) is None and parse(b"Art-Net") is None
          and parse(b"x" * 30) is None)
    lv = levels(data, n=4)
    check("a LED's level is its brightest channel, 0..1; missing LEDs dark",
          lv[0] == 0.0 and lv[1] == 1.0 and abs(lv[2] - 128 / 255.0) < 1e-9 and lv[3] == 0.0, lv)
    check("... from the start channel", levels(bytes([9, 9, 9, 255, 0, 0]), n=1, start=3) == [1.0])
    rx = Receiver(port=0)                                # any free port: a real TD stays untouched
    port = rx.sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    check("nothing yet: None", rx.poll() is None and rx.age() is None)
    tx.sendto(artdmx(1, bytes([255] * 180)), ("127.0.0.1", port))          # another universe
    tx.sendto(artdmx(0, bytes([51] * 180)), ("127.0.0.1", port))
    tx.sendto(artdmx(0, bytes([255, 0, 0] + [0] * 177)), ("127.0.0.1", port))
    for _ in range(50):
        got = rx.poll()
        if rx.packets >= 2:
            break
        time.sleep(0.01)
    check("the receiver keeps the newest packet of its universe, all 60 LEDs",
          got is not None and len(got) == 60 and got[0] == 1.0 and got[1] == 0.0 and rx.packets == 2,
          (got[:3] if got else None, rx.packets))
    check("... and knows how old it is", rx.age() is not None and rx.age() < 1.0)
    check("the ceiling's dimmer: channel 1, 8 bit, 0..1 (TD sends the universe whole); none: None",
          dimmer(bytes([51] + [0] * 511)) == 0.2 and dimmer(bytes([255, 255, 7])) == 1.0 and dimmer(b"") is None)
    drx = DimmerReceiver(port=0)
    tx.sendto(artdmx(0, bytes([64] + [0] * 511)), ("127.0.0.1", drx.sock.getsockname()[1]))
    for _ in range(50):
        if drx.poll() is not None:
            break
        time.sleep(0.01)
    check("... its receiver keeps the newest level", drx.poll() == 64 / 255.0, drx.poll())
    drx.close()
    tp = test_pattern(1.0)
    check("the test pattern: a comet 20 LEDs in after 1 s, LED 0's end marked", tp[20] == 1.0 and tp[0] == 0.15
          and tp[30] == 0.0, (tp[0], tp[20], tp[30]))
    tx.close()
    rx.close()
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if "--self-test" in argv:
        return self_test()
    if "--send-test" in argv:
        k = argv.index("--send-test")
        port = int(argv[k + 1]) if len(argv) > k + 1 and argv[k + 1].isdigit() else PREVIEW_PORT
        seconds = float(argv[argv.index("--seconds") + 1]) if "--seconds" in argv else 1e9
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("sending the test pattern to 127.0.0.1:%d, 40 a second (Ctrl+C to stop)" % port)
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < seconds:
                lv = test_pattern(time.monotonic() - t0)
                tx.sendto(artdmx(0, bytes(int(255 * x) for x in lv for _ in range(3))), ("127.0.0.1", port))
                time.sleep(1 / 40.0)
        except KeyboardInterrupt:
            pass
        return 0
    if "--listen" in argv:
        port = int(argv[argv.index("--listen") + 1]) if len(argv) > argv.index("--listen") + 1 else PREVIEW_PORT
        rx = Receiver(port)
        print("listening on 127.0.0.1:%d (Ctrl+C to stop)" % port)
        try:
            while True:
                lv = rx.poll()
                if lv is not None:
                    print("\r%4d packets  %s" % (rx.packets, "".join(" .:-=+*#%@"[min(9, int(x * 9.99))] for x in lv)),
                          end="", flush=True)
                time.sleep(0.05)
        except KeyboardInterrupt:
            rx.close()
        return 0
    print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
