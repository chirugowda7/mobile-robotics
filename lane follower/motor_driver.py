"""
Cytron Motor Driver Controller for Raspberry Pi.
Supports Cytron PWM + DIR motor drivers (e.g., MDD10A, MD10C, Maker Drive, MDD3A).
Includes automatic simulation fallback for non-Pi environments.
"""

import sys
import time
import config

# Attempt to load Raspberry Pi GPIO library
GPIO_AVAILABLE = False
_gpio_mode = None

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    _gpio_mode = "RPi.GPIO"
except ImportError:
    try:
        import gpiozero
        # We can also check if running under gpiozero
        GPIO_AVAILABLE = False # Default to custom RPi.GPIO or simulation
    except ImportError:
        pass


class CytronMotorDriver:
    """
    Controls two DC motors using Cytron PWM + DIR driver architecture.
    Speed range: -100.0 (Full Reverse) to +100.0 (Full Forward).
    """

    def __init__(self,
                 pwm_left=config.PIN_PWM_LEFT,
                 dir_left=config.PIN_DIR_LEFT,
                 pwm_right=config.PIN_PWM_RIGHT,
                 dir_right=config.PIN_DIR_RIGHT,
                 freq=config.PWM_FREQUENCY,
                 invert_left=config.INVERT_LEFT_MOTOR,
                 invert_right=config.INVERT_RIGHT_MOTOR):
        self.pwm_left_pin = pwm_left
        self.dir_left_pin = dir_left
        self.pwm_right_pin = pwm_right
        self.dir_right_pin = dir_right
        self.freq = freq
        self.invert_left = invert_left
        self.invert_right = invert_right

        self.left_pwm_device = None
        self.right_pwm_device = None
        self.is_hardware_active = False

        self.current_left_speed = 0.0
        self.current_right_speed = 0.0

        self._init_gpio()

    def _init_gpio(self):
        """Initializes GPIO pins on Raspberry Pi or enters simulation mode."""
        global GPIO_AVAILABLE
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)

                # Set DIR pins as digital outputs
                GPIO.setup(self.dir_left_pin, GPIO.OUT)
                GPIO.setup(self.dir_right_pin, GPIO.OUT)

                # Set PWM pins as outputs
                GPIO.setup(self.pwm_left_pin, GPIO.OUT)
                GPIO.setup(self.pwm_right_pin, GPIO.OUT)

                # Initialize PWM instances
                self.left_pwm_device = GPIO.PWM(self.pwm_left_pin, self.freq)
                self.right_pwm_device = GPIO.PWM(self.pwm_right_pin, self.freq)

                self.left_pwm_device.start(0)
                self.right_pwm_device.start(0)

                self.is_hardware_active = True
                print(f"[MOTOR] Cytron Hardware GPIO initialized on BCM pins:")
                print(f"        Left Motor : PWM={self.pwm_left_pin}, DIR={self.dir_left_pin}")
                print(f"        Right Motor: PWM={self.pwm_right_pin}, DIR={self.dir_right_pin}")
                return
            except Exception as e:
                print(f"[MOTOR] Failed to initialize RPi.GPIO ({e}). Falling back to simulation mode.")
                self.is_hardware_active = False

        print("[MOTOR] Running in SIMULATION / VIRTUAL mode (No physical Raspberry Pi GPIO).")
        self.is_hardware_active = False

    def set_speeds(self, left_speed: float, right_speed: float):
        """
        Sets speed and direction for both motors.
        :param left_speed: -100.0 to 100.0 (%)
        :param right_speed: -100.0 to 100.0 (%)
        """
        # Clamp input between -100.0 and 100.0
        left_speed = max(-100.0, min(100.0, float(left_speed)))
        right_speed = max(-100.0, min(100.0, float(right_speed)))

        self.current_left_speed = left_speed
        self.current_right_speed = right_speed

        if self.is_hardware_active:
            self._apply_hardware_motor(
                left_speed,
                self.dir_left_pin,
                self.left_pwm_device,
                self.invert_left
            )
            self._apply_hardware_motor(
                right_speed,
                self.dir_right_pin,
                self.right_pwm_device,
                self.invert_right
            )

    def _apply_hardware_motor(self, speed, dir_pin, pwm_device, invert):
        """Helper to write direction and PWM duty cycle to Cytron driver."""
        is_forward = speed >= 0
        if invert:
            is_forward = not is_forward

        # Direction logic: HIGH = forward, LOW = reverse
        dir_value = GPIO.HIGH if is_forward else GPIO.LOW
        GPIO.output(dir_pin, dir_value)

        # Duty cycle is absolute value of speed (0 - 100%)
        duty = min(100.0, abs(speed))
        pwm_device.ChangeDutyCycle(duty)

    def stop(self):
        """Immediately stops both motors (0% duty cycle)."""
        self.set_speeds(0.0, 0.0)

    def get_speeds(self):
        """Returns current (left_speed, right_speed)."""
        return self.current_left_speed, self.current_right_speed

    def cleanup(self):
        """Safely stops motors and releases GPIO."""
        self.stop()
        if self.is_hardware_active:
            try:
                if self.left_pwm_device:
                    self.left_pwm_device.stop()
                if self.right_pwm_device:
                    self.right_pwm_device.stop()
                GPIO.cleanup([
                    self.pwm_left_pin, self.dir_left_pin,
                    self.pwm_right_pin, self.dir_right_pin
                ])
                print("[MOTOR] GPIO cleanup completed.")
            except Exception as e:
                print(f"[MOTOR] Cleanup error: {e}")
