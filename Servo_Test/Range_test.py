#!/usr/bin/env python3
import time
import sys
import argparse

try:
    import pigpio
except ImportError:
    print("Error: pigpio module not found. Install with: sudo apt-get install pigpio")
    sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description="Step a servo through 500, 1000, 1500, 2000, 2500 µs using pigpio."
    )
    parser.add_argument("--pins", type=int, nargs="+", default=[18],
                        help="BCM GPIO pin(s) to drive (default: 18). Example: --pins 18 13")
    parser.add_argument("--interactive", action="store_true",
                        help="Wait for Enter between steps so you can mark/measure.")
    parser.add_argument("--dwell", type=float, default=2.0,
                        help="Seconds to hold each pulse in timed mode (default: 2.0s).")
    args = parser.parse_args()

    pulses_us = [500, 1000, 1500, 2000, 2500]

    pi = pigpio.pi()
    if not pi.connected:
        print("Error: pigpio daemon not running. Start it with: sudo pigpiod")
        sys.exit(1)

    try:
        print(f"Using GPIO pin(s): {args.pins}")
        print("Sequence (µs):", pulses_us)
        print("Note: 0 µs will be sent at the end to stop PWM output on each pin.")
        print()

        for us in pulses_us:
            # Apply pulsewidth to all pins
            for pin in args.pins:
                # pigpio set_servo_pulsewidth uses 50 Hz and accepts 500..2500 µs
                pi.set_servo_pulsewidth(pin, us)

            # Report
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] -> Set pulsewidth = {us} µs on pins {args.pins}")

            if args.interactive:
                input("   Press Enter to continue...")
            else:
                time.sleep(args.dwell)

        print("\nDone stepping through pulses. Stopping servo outputs (0 µs).")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Turn off servo pulses
        for pin in args.pins:
            pi.set_servo_pulsewidth(pin, 0)
        pi.stop()
        print("pigpio cleaned up.")

if __name__ == "__main__":
    main()
