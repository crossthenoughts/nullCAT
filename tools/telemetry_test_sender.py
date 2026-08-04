#!/usr/bin/env python3
"""
telemetry_test_sender.py - Sends 16-bit nullCAT-format UDP packets for testing.
Packet format: NULLCAT,<axis1>,<axis2>,...,<axis10>
Values: 0-65535, center=32767
"""
import socket, time, math, sys

HOST = "127.0.0.1"
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 4444
HZ   = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0

# [amplitude_normalized (0-1.0), frequency_hz, phase_rad]
# amplitude 1.0 = full stroke from center
AXIS_PARAMS = [
    (0.8, 0.3, 0.0),   # Heave
    (0.6, 0.4, 0.5),   # Pitch
    (0.6, 0.5, 1.0),   # Roll
    (0.5, 0.2, 1.5),   # Surge
    (0.5, 0.25, 2.0),  # Sway
    (0.7, 0.15, 0.3),  # TractionLoss
    (0.5, 0.18, 0.7),  # Drive 7
    (0.5, 0.22, 1.1),  # Drive 8
    (0.5, 0.28, 1.6),  # Drive 9
    (0.4, 0.35, 2.1),  # Drive 10
]

CENTER = 32767
HALF   = 32767

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / HZ
    start_time = time.perf_counter()
    packet_count = 0

    print(f"nullCAT 16-bit test sender | {HOST}:{PORT} | {HZ}Hz | Ctrl+C to stop")

    next_send = time.perf_counter()
    try:
        while True:
            elapsed = time.perf_counter() - start_time
            values = []
            for amp, freq, phase in AXIS_PARAMS:
                norm = amp * math.sin(2 * math.pi * freq * elapsed + phase)
                raw = int(CENTER + norm * HALF)
                raw = max(0, min(65535, raw))
                values.append(str(raw))

            packet = "NULLCAT," + ",".join(values) + "\n"
            sock.sendto(packet.encode("ascii"), (HOST, PORT))
            packet_count += 1

            if packet_count % int(HZ * 2) == 0:
                print(f"\r  t={elapsed:.1f}s pkt={packet_count} "
                      f"A1={values[0]} A2={values[1]} A3={values[2]}   ",
                      end="", flush=True)

            next_send += interval
            sleep_time = next_send - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\nStopped. {packet_count} packets sent.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()