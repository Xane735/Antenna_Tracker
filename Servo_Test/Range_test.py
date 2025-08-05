#!/usr/bin/env python3
import time
import sys
import argparse
import pigpio

def set_pulse_and_wait(pi, pins, pulse_us, dwell, interactive):
    for pin in pins:
        pi.set_servo_pulsewidth(pin, pulse_us)
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] -> Set pulsewidth = {pulse_us} µs on pins {pins}")
    if interactive:
        input("   Press Enter to continue...")
    else:
        time.sleep(dwell)

def sweep(pi, pins, start_us, end_us, step_us, dwell, interactive):
    print(f"\nStarting sweep from {start_us} µs to {end_us} µs and back.")
    sweep_range = list(range(start_us, end_us + step_us, step_us)) + list(range(end_us - step_us, start_us - step_us, -step_us))
    for pulse in sweep_range:
        set_pulse_and_wait(pi, pins, pulse, dwell, interactive)

def main():
    parser = argparse.ArgumentParser(description="Step servos and sweep using pigpio.")
    parser.add_argument("--pins", type=int, nargs="+", default=[18],
                        help="BCM GPIO pin(s) to drive (default: 18). Example: --pins 18 13")
    parser.add_argument("--interactive", action="store_true",
                        help="Wait for Enter between steps so you can mark/measure.")
    parser.add_argument("--dwell", type=float, default=2.0,
                        help="Seconds to hold each pulse in timed mode (default: 2.0s).")
    args = parser.parse_args()

    pi = pigpio.pi()
    if not pi.connected:
        print("Error: pigpio daemon not running. Start it with: sudo pigpiod")
        sys.exit(1)

    '''for i in range (5):
        pulse_ms_val = input("Enter the Signal Value")
        set_pulse_and_wait(pi, args.pins, pulse_ms_val, args.dwell, args.interactive)'''


    pulses_us = [900, 1500, 2100]

    try:
        print(f"Using GPIO pin(s): {args.pins}")
        print("Stepping through fixed pulsewidths:", pulses_us)

        for pulse in pulses_us:
            set_pulse_and_wait(pi, args.pins, pulse, args.dwell, args.interactive)

        # Sweep from 900 to 2100 and back
        sweep(pi, args.pins, start_us=900, end_us=2100, step_us=100,
              dwell=0.05 if not args.interactive else args.dwell,  # faster sweep in timed mode
              interactive=args.interactive)

        print("\nDone with stepping and sweeping. Stopping servo outputs (0 µs).")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        for pin in args.pins:
            pi.set_servo_pulsewidth(pin, 0)
        pi.stop()
        print("pigpio cleaned up.")

if __name__ == "__main__":
    main()
