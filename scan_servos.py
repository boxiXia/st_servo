
"""
Scan for ST Servos and optionally rename one.

Usage:
    scan_servos.py [options]

Options:
    --new-id <id>    New ID to assign (0-253). Only works if exactly one servo is connected.
"""
from st_servo import StServo
import time
import tyro
from typing import Optional, Annotated


def scan_servos(new_id: Annotated[Optional[int], tyro.conf.Positional] = None):
    """
    Scan for ST Servos and optionally rename one.

    Args:
        new_id: New ID to assign (0-253). Only works if exactly one servo is connected.
    """
    port = '/dev/ttyACM0'
    print(f"Connecting to {port}...")
    try:
        servo = StServo(port)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("Scanning for servos...")
    start_time = time.time()

    # Try Broadcast Ping for quick info (optional)
    try:
        ret = servo.ping(254)
        if ret:
            f_id, err, _ = ret
            if f_id != 254:
                print(f"Broadcast Ping reply from ID {f_id}! (Checking for others...)")
    except Exception:
        pass

    # Always perform full sequential scan to ensure we find ALL connected servos
    # (Broadcast ping stops at first reply and is unreliable for multi-servo counting)
    found_servos = servo.scan()

    duration = time.time() - start_time
    print(f"\nScan Complete in {duration:.2f}s. Found {len(found_servos)} servos: {found_servos}")

    if new_id is not None:
        if len(found_servos) == 1:
            current_id = found_servos[0]
            if current_id == new_id:
                print(f"Servo is already at ID {new_id}. No change needed.")
            else:
                print(f"Renaming Servo {current_id} to {new_id}...")
                servo.write_id(current_id, new_id)
                time.sleep(0.1)  # Wait for write

                # Verify
                print(f"Verifying ID {new_id}...")
                if servo.ping(new_id):
                    print("Success!")
                else:
                    print("Verification failed. Please check connection.")

                print("Done. Please power cycle the servo if it doesn't respond immediately.")
        elif len(found_servos) == 0:
            print("Error: No servo found. Cannot rename.")
        else:
            print(f"Error: Multiple servos found {found_servos}. Please connect only one servo to rename.")


if __name__ == "__main__":
    tyro.cli(scan_servos)
