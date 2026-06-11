"""Quick scan: probe slave IDs 1-15 with FC01 read-coils to find the device."""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"


def crc(d: bytes) -> bytes:
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return bytes([c & 0xFF, (c >> 8) & 0xFF])


s = serial.Serial(PORT, 9600, timeout=0.5)
time.sleep(0.2)
print(f"Scanning slave IDs 1..15 on {PORT} @ 9600 8N1 ...")
found = False
for sid in range(1, 16):
    body = bytes([sid, 0x01, 0, 0, 0, 0x07])
    tx = body + crc(body)
    s.reset_input_buffer()
    s.write(tx)
    s.flush()
    time.sleep(0.15)
    rx = s.read(256)
    if rx:
        found = True
        print(f"  slave {sid:>2}: RX {rx.hex(' ').upper()}")
s.close()
print("scan complete" + ("" if found else " - no replies"))
