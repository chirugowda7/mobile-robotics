#!/usr/bin/env python3
"""
Hardware Diagnostic Tool for Cytron Motor Driver.
Tests motor wiring, direction, and PWM speeds before autonomous tracking.
"""

import time
import sys
import config
from motor_driver import CytronMotorDriver


def print_menu():
    print("\n" + "=" * 50)
    print("      CYTRON MOTOR DRIVER HARDWARE TEST TOOL      ")
    print("=" * 50)
    print(" [1] Test Left Motor (Forward @ 40%)")
    print(" [2] Test Left Motor (Reverse @ 40%)")
    print(" [3] Test Right Motor (Forward @ 40%)")
    print(" [4] Test Right Motor (Reverse @ 40%)")
    print(" [5] Test Both Motors Forward (Cruising @ 40%)")
    print(" [6] Test Both Motors Reverse (@ 40%)")
    print(" [7] Test Pivot Turn Left (@ 35%)")
    print(" [8] Test Pivot Turn Right (@ 35%)")
    print(" [9] Speed Ramp Test (0% -> 70% -> 0%)")
    print(" [0] STOP All Motors")
    print(" [Q] Quit")
    print("=" * 50)


def main():
    print("[INIT] Initializing Cytron Motor Driver...")
    motors = CytronMotorDriver()

    try:
        while True:
            print_menu()
            choice = input("Enter option [0-9 or Q]: ").strip().lower()

            if choice == '1':
                print(">> Running Left Motor Forward for 2 seconds...")
                motors.set_speeds(40.0, 0.0)
                time.sleep(2.0)
                motors.stop()

            elif choice == '2':
                print(">> Running Left Motor Reverse for 2 seconds...")
                motors.set_speeds(-40.0, 0.0)
                time.sleep(2.0)
                motors.stop()

            elif choice == '3':
                print(">> Running Right Motor Forward for 2 seconds...")
                motors.set_speeds(0.0, 40.0)
                time.sleep(2.0)
                motors.stop()

            elif choice == '4':
                print(">> Running Right Motor Reverse for 2 seconds...")
                motors.set_speeds(0.0, -40.0)
                time.sleep(2.0)
                motors.stop()

            elif choice == '5':
                print(">> Running Both Motors Forward for 2 seconds...")
                motors.set_speeds(40.0, 40.0)
                time.sleep(2.0)
                motors.stop()

            elif choice == '6':
                print(">> Running Both Motors Reverse for 2 seconds...")
                motors.set_speeds(-40.0, -40.0)
                time.sleep(2.0)
                motors.stop()

            elif choice == '7':
                print(">> Pivot Turn Left for 1.5 seconds...")
                motors.set_speeds(-35.0, 35.0)
                time.sleep(1.5)
                motors.stop()

            elif choice == '8':
                print(">> Pivot Turn Right for 1.5 seconds...")
                motors.set_speeds(35.0, -35.0)
                time.sleep(1.5)
                motors.stop()

            elif choice == '9':
                print(">> Ramping speed from 0% to 70%...")
                for spd in range(0, 71, 5):
                    motors.set_speeds(spd, spd)
                    time.sleep(0.1)
                print(">> Ramping down from 70% to 0%...")
                for spd in range(70, -1, -5):
                    motors.set_speeds(spd, spd)
                    time.sleep(0.1)
                motors.stop()

            elif choice == '0':
                print(">> Motors STOPPED.")
                motors.stop()

            elif choice == 'q':
                print(">> Exiting test tool.")
                break

            else:
                print(">> Invalid choice! Please select 0-9 or Q.")

    finally:
        motors.cleanup()
        print("[CLEANUP] Motors stopped and GPIO freed.")


if __name__ == "__main__":
    main()
