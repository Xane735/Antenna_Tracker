import time
import sys
import argparse
import pigpio

pulses_us = [900, 1050, 1200, 1350, 1500, 1650, 1800, 1950, 2100]

def set_pulse_and_wait(pi, pins, pulse_us, delay=2):
    for pin in pins:
        pi.set_servo_pulsewidth(pin, pulse_us)
    
    print(f"Set pulsewidth = {pulse_us} us on pins {pins}")
    time.sleep(delay)

def run_sweep(pi, pins, start_us, end_us, step_us):
    print(f"\nStarting sweep from {start_us} us to {end_us} us and back.")
    sweep_range = list(range(start_us, end_us + step_us, step_us)) + \
                  list(range(end_us - step_us, start_us - step_us, -step_us))
    
    for pulse in sweep_range:
        set_pulse_and_wait(pi, pins, pulse, delay=0.05)
    
    print("\nDone with sweeping.")

def main():
    parser = argparse.ArgumentParser(description="Step servos and sweep using pigpio.")
    parser.add_argument("--pins", type=int, nargs="+", default=[18,17], help="GPIO pin(s) to drive (default: 18). Example: --pins 18 17")
    parser.add_argument("--sweep", action="store_true", help="Set this flag to enable the servo sweep.")
    args = parser.parse_args()

    pi = pigpio.pi()
    if not pi.connected:
        print("Error: pigpio daemon not running. Start it with: sudo pigpiod")
        sys.exit(1)
    
    try:
        print(f"Using GPIO pin(s): {args.pins}")

        if args.sweep:
            run_sweep(pi, args.pins, start_us=900, end_us=2100, step_us=100)
        else:
            n = int(input("Enter Number of itterations to be run "))
            for i in range(n):
                try:
                    pulse_us_val = int(input(f"Enter the pulse width value (in us): "))
                    set_pulse_and_wait(pi, args.pins, pulse_us_val, delay=1)
                except ValueError:
                    print("Invalid input. Please enter an integer.")
            print("\nDone with user input iterations.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        for pin in args.pins:
            pi.set_servo_pulsewidth(pin, 0)
        pi.stop()
        print("pigpio cleaned up.")

if __name__ == "__main__":
    main()