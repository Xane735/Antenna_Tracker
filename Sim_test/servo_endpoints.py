#!/usr/bin/env python3
import time
import pigpio

AZI_PIN = 18  # BCM pin for azimuth signal

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running. Start it with: sudo pigpio")

try:
    pi.set_mode(AZI_PIN, pigpio.OUTPUT)

    for us in (1000, 1500, 2000):
        print(f"AZI pulse {us} us")
        pi.set_servo_pulsewidth(AZI_PIN, us)
        time.sleep(2.0)

    # Optional: sweep slowly end to end
    for us in range(1000, 2001, 50):
        pi.set_servo_pulsewidth(AZI_PIN, us)
        time.sleep(0.05)

    time.sleep(1.0)
finally:
    pi.set_servo_pulsewidth(AZI_PIN, 0)  # stop pulses
    pi.stop()
