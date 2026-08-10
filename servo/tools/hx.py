#!/usr/bin/env python3
"""
Talk to an HX-30HM through the FPGA bridge bitstream (src/bridge.v).

The FPGA relays whole packets between this port at 115200 and the servo's
1 Mbaud single-wire bus, so everything below is plain protocol.

    make bridge-flash
    python3 tools/hx.py diag          # what state is the servo actually in?
    python3 tools/hx.py spin 4000     # constant-speed mode at 4000 steps/s
    python3 tools/hx.py stop
    python3 tools/hx.py read 0x21 2
    python3 tools/hx.py write 0x2E 4000 2

No dependency on the vendor SDK -- and deliberately not modelled on it,
since its half-duplex handling reads its own echo as a valid reply.
"""

import sys, time, glob

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  python3 -m pip install --user pyserial")

BROADCAST = 0xFE

# Widths corrected against the real servo. Hiwonder's register table is
# internally inconsistent -- it lists 0x13 as 2 bytes while also listing 0x14
# as a separate register, which cannot both be true. Reading 0x13 as 2 bytes
# returns 12076 against a documented range of 0..1000. These are the widths
# that actually read back sane values.
REGS = {
    0x00: ("fw_major", 1), 0x01: ("fw_minor", 1),
    0x05: ("id", 1),       0x06: ("baud_idx", 1),
    0x08: ("resp_level", 1),
    0x0D: ("temp_limit", 1),
    0x13: ("max_torque", 1),
    0x14: ("protect_ctrl", 1),
    0x1F: ("current_limit_mA", 2),
    0x21: ("ctrl_mode", 1),          # 1 byte, not 2 as documented
    0x28: ("reg_0x28", 2),
    0x2A: ("target_pos", 2), 0x2C: ("run_time", 2), 0x2E: ("run_speed", 2),
    0x37: ("nvs_lock", 1),
    0x38: ("cur_pos", 2), 0x3A: ("cur_speed", 2), 0x3C: ("cur_load", 2),
    0x3E: ("voltage_mV", 2), 0x3F: ("temp_C", 1),
    0x41: ("status_bits", 1), 0x42: ("moving", 1),
    0x45: ("current_mA", 2),
}

STATUS_BITS = ["over-voltage", "under-voltage", "over-temperature",
               "over-current", "overload", "instruction error"]


def find_port():
    """Channel B of the FT2232 is the UART; channel A is the JTAG."""
    ports = sorted(glob.glob("/dev/cu.usbserial-*"))
    if not ports:
        sys.exit("no /dev/cu.usbserial-* found -- is the dock plugged in?")
    return ports[-1]          # highest suffix = channel B


class Bus:
    def __init__(self, port=None, baud=115200):
        self.ser = serial.Serial(port or find_port(), baud, timeout=0.25)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    @staticmethod
    def checksum(payload):
        return (~sum(payload)) & 0xFF

    def send(self, sid, instr, params=()):
        params = list(params)
        payload = [sid, len(params) + 2, instr] + params
        pkt = bytes([0xFF, 0xFF] + payload + [self.checksum(payload)])
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        return pkt

    def recv(self, timeout=0.3):
        """Read one reply. The bridge gates the echo, so anything arriving
        here came from the servo -- no need to skip our own transmission."""
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            buf += b
            if len(buf) >= 2 and buf[-2:] == b"\xff\xff":
                break
        else:
            return None

        head = self.ser.read(2)                     # id, length
        if len(head) < 2:
            return None
        sid, length = head[0], head[1]
        rest = self.ser.read(length)                # error + params + checksum
        if len(rest) < length:
            return None
        err, params, cs = rest[0], list(rest[1:-1]), rest[-1]
        if self.checksum([sid, length, err] + params) != cs:
            return {"id": sid, "error": err, "params": params, "bad_checksum": True}
        return {"id": sid, "error": err, "params": params, "bad_checksum": False}

    def txn(self, sid, instr, params=()):
        self.send(sid, instr, params)
        return self.recv()

    def ping(self, sid=1):
        return self.txn(sid, 0x01)

    def read(self, sid, addr, n):
        r = self.txn(sid, 0x02, [addr, n])
        if not r or r.get("bad_checksum"):
            return None
        return r["params"]

    def read_word(self, sid, addr, n=2):
        p = self.read(sid, addr, n)
        if p is None or len(p) < n:
            return None
        return p[0] | (p[1] << 8) if n == 2 else p[0]

    def write(self, sid, addr, data):
        return self.txn(sid, 0x03, [addr] + list(data))


def w16(v):
    return [v & 0xFF, (v >> 8) & 0xFF]


def cmd_scan(bus):
    print("scanning IDs 0..20 ...")
    found = []
    for sid in range(0, 21):
        if bus.ping(sid):
            print(f"  ID {sid}: responds")
            found.append(sid)
    if not found:
        print("  nothing answered.")
    return found


def cmd_diag(bus, sid=1):
    print(f"--- HX-30HM diagnostics (ID {sid}) ---")
    r = bus.ping(sid)
    if not r:
        print("PING: no reply. Check wiring, ground, baud, and servo ID.")
        print("      Try: python3 tools/hx.py scan")
        return
    print(f"PING: ok, error byte 0x{r['error']:02X}")

    for addr in (0x00, 0x05, 0x06, 0x21, 0x2A, 0x2E, 0x37,
                 0x38, 0x3A, 0x3C, 0x3E, 0x3F, 0x41, 0x42, 0x45):
        name, n = REGS.get(addr, (f"reg_{addr:02X}", 1))
        v = bus.read_word(sid, addr, n)
        if v is None:
            print(f"  0x{addr:02X} {name:<18} <no reply>")
            continue
        extra = ""
        if addr == 0x3E:
            extra = f"  = {v/1000:.2f} V"   # measured: mV, not 0.1 V
        elif addr == 0x21:
            extra = {0: "  = POSITION mode", 1: "  = CONSTANT-SPEED mode",
                     2: "  = open-loop PWM"}.get(v & 0x7, "")
        elif addr == 0x37:
            extra = "  = LOCKED" if v else "  = unlocked"
        elif addr == 0x41 and v:
            extra = "  = " + ", ".join(
                STATUS_BITS[i] for i in range(6) if v & (1 << i))
        print(f"  0x{addr:02X} {name:<18} {v:<6} 0x{v:04X}{extra}")


def cmd_spin(bus, speed, sid=1):
    """Enter constant-speed mode and command a speed."""
    print(f"unlocking NVS ...");        bus.write(sid, 0x37, [0])
    print(f"control mode -> 1 ...");    bus.write(sid, 0x21, w16(1))
    print(f"re-locking NVS ...");       bus.write(sid, 0x37, [1])

    mode = bus.read_word(sid, 0x21, 2)
    print(f"read back ctrl_mode = {mode}")
    if mode != 1:
        print("  !! mode did NOT take. NVS may only load at boot --")
        print("     power-cycle the SERVO (not the FPGA), then run:")
        print(f"       python3 tools/hx.py setspeed {speed}")
        return

    print(f"speed -> {speed} ...")
    bus.write(sid, 0x2E, w16(speed))
    time.sleep(0.3)
    print(f"cur_speed={bus.read_word(sid,0x3A,2)}  "
          f"current={bus.read_word(sid,0x45,2)} mA  "
          f"load={bus.read_word(sid,0x3C,2)}")


def cmd_setspeed(bus, speed, sid=1):
    """Speed only -- for use after a power cycle, when mode is already set."""
    bus.write(sid, 0x2E, w16(speed))
    time.sleep(0.3)
    print(f"mode={bus.read_word(sid,0x21,2)}  "
          f"cur_speed={bus.read_word(sid,0x3A,2)}  "
          f"current={bus.read_word(sid,0x45,2)} mA")


def cmd_stop(bus, sid=1):
    bus.write(sid, 0x2E, w16(0))
    bus.write(sid, 0x37, [0])
    bus.write(sid, 0x21, w16(0))
    bus.write(sid, 0x37, [1])
    print(f"stopped; ctrl_mode={bus.read_word(sid,0x21,2)}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    bus = Bus()
    print(f"port {bus.ser.port} @ {bus.ser.baudrate}\n")
    a = sys.argv[1]

    if   a == "diag":  cmd_diag(bus)
    elif a == "scan":  cmd_scan(bus)
    elif a == "ping":  print(bus.ping(1))
    elif a == "spin":  cmd_spin(bus, int(sys.argv[2], 0))
    elif a == "setspeed": cmd_setspeed(bus, int(sys.argv[2], 0))
    elif a == "stop":  cmd_stop(bus)
    elif a == "read":
        addr, n = int(sys.argv[2], 0), int(sys.argv[3], 0)
        print(bus.read(1, addr, n))
    elif a == "write":
        addr, val, n = int(sys.argv[2], 0), int(sys.argv[3], 0), int(sys.argv[4], 0)
        print(bus.write(1, addr, w16(val) if n == 2 else [val & 0xFF]))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
