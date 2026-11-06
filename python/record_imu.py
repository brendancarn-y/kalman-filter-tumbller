"""record_imu.py: save Tumbller IMU data from the serial port to a CSV file.

Usage (in a terminal):
    python record_imu.py COM5 --seconds 30 --out tilt_test.csv

Close the Arduino Serial Monitor first: only one program can use the port at a time.
"""
import argparse
import csv
import time

import serial

COLUMNS = ["t_us", "ax", "ay", "az", "gx", "gy", "gz"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("port", help="e.g. COM5 (see Arduino IDE > Tools > Port)")
    p.add_argument("--seconds", type=float, default=30)
    p.add_argument("--out", default="imu_log.csv")
    p.add_argument("--baud", type=int, default=115200)
    a = p.parse_args()

    good = bad = 0
    try:
        ser = serial.serial_for_url(a.port, a.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {a.port}: {e}")
        print("Is the Serial Monitor still open, or is the port name wrong?")
        return

    with ser, open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        time.sleep(2)              # the Nano resets when the port opens
        ser.reset_input_buffer()   # throw away anything sent during the reset

        print(f"Recording {a.seconds:.0f} s to {a.out}. Ctrl+C stops early.")
        t_end = time.time() + a.seconds
        last_report = time.time()
        try:
            while time.time() < t_end:
                raw = ser.readline()
                if not raw:
                    print("WARNING: no data for 1 s")
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                try:
                    row = [int(x) for x in line.split(",")]
                    if len(row) != len(COLUMNS):
                        raise ValueError
                except ValueError:
                    bad += 1
                    if bad <= 20:
                        print(f"skipped bad line: {line!r}")
                    continue
                w.writerow(row)
                good += 1
                if time.time() - last_report >= 1:
                    print(f"{good} samples, {bad} bad lines")
                    last_report = time.time()
        except serial.SerialException as e:
            print(f"\nUSB CONNECTION DROPPED: {e}")
            print("Data up to this point is saved. Check the cable and the USB port.")
        except KeyboardInterrupt:
            print("\nStopped by you.")

    print(f"Done: {good} samples saved to {a.out}, {bad} bad lines skipped.")


if __name__ == "__main__":
    main()
