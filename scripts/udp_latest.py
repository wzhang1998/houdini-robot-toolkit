"""The newest message on a UDP port, never blocking: what a live preview
wants from TouchDesigner (artnet.py's LEDs, canvas_link.py's canvas) -- read
everything waiting, keep the last that parses.

    rx = LatestReceiver(port, parse)      # parse(bytes) -> a value, or None to skip the packet
    v = rx.poll()                         # the newest value; None until one arrives
    rx.age()                              # seconds since it arrived (None: none yet)

    python scripts/udp_latest.py --self-test

Standard library only.
"""

import socket
import sys
import time


class LatestReceiver:
    def __init__(self, port, parse, host="127.0.0.1", bufsize=65536, rcvbuf=1 << 20):
        self.parse, self.bufsize = parse, bufsize
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.latest, self.t_last, self.packets = None, None, 0

    def poll(self):
        """The newest value (every waiting packet read); None until one arrives."""
        while True:
            try:
                packet = self.sock.recv(self.bufsize)
            except (BlockingIOError, ConnectionResetError):
                break
            got = self.parse(packet)
            if got is not None:
                self.latest, self.t_last = got, time.monotonic()
                self.packets += 1
        return self.latest

    def age(self):
        return None if self.t_last is None else time.monotonic() - self.t_last

    def port(self):
        return self.sock.getsockname()[1]

    def close(self):
        self.sock.close()


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    rx = LatestReceiver(0, lambda b: b.decode() if b.startswith(b"ok") else None)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    check("nothing yet: None, no age", rx.poll() is None and rx.age() is None)
    for m in (b"ok 1", b"junk", b"ok 2", b"nope"):
        tx.sendto(m, ("127.0.0.1", rx.port()))
    for _ in range(50):
        got = rx.poll()
        if rx.packets >= 2:
            break
        time.sleep(0.01)
    check("the newest that parses; the rest skipped", got == "ok 2" and rx.packets == 2, (got, rx.packets))
    check("... and how old it is", rx.age() is not None and rx.age() < 1.0)
    tx.close()
    rx.close()
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    print(__doc__)
