"""Utilities for the A4988 stepper controller used in PiLiDAR.

References for curious readers:

* Datasheet: http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx
* Microstepping table: https://i.stack.imgur.com/vN7JL.png

The original project only worked on a Raspberry Pi because it directly
imported ``RPi.GPIO`` during module import.  For the rewritten workflow we
allow importing the module on every platform by providing a very small mock
GPIO implementation.  The mock prints informative messages so that absolute
beginners understand what would happen on the real hardware.
"""

from __future__ import annotations

import os
from time import sleep
from typing import Iterable, Optional

# ---------------------------------------------------------------------------
# GPIO fallback for non Raspberry Pi systems
# ---------------------------------------------------------------------------
try:  # pragma: no cover - exercised implicitly on Raspberry Pi
    # allow running on non-Pi systems when rpi-lgpio is installed
    os.environ.setdefault("RPI_LGPIO_REVISION", "0xa020d3")
    import RPi.GPIO as _REAL_GPIO  # type: ignore
except Exception:  # pragma: no cover - triggered on CI or development machines
    class _MockGPIO:
        """Tiny GPIO stand-in that mimics the methods used in this project.

        The mock keeps the same method names as the real library so our code
        does not need special cases later on.  The methods simply store the
        requested state which makes unit tests possible without any hardware
        attached.
        """

        BCM = "BCM"
        OUT = "OUT"
        HIGH = True
        LOW = False

        def __init__(self) -> None:
            self._pins = {}
            self._pwms = {}

        def setwarnings(self, _: bool) -> None:
            pass

        def setmode(self, mode: str) -> None:
            self.mode = mode

        def setup(self, pin: int, _: str) -> None:
            self._pins[pin] = self.LOW

        def output(self, pin: int, state: bool) -> None:
            self._pins[pin] = state

        class _MockPWM:
            def __init__(self, pin: int, frequency: float) -> None:
                self.pin = pin
                self.frequency = frequency
                self.duty_cycle = 0.0
                self.running = False

            def start(self, duty_cycle: float) -> None:
                self.duty_cycle = duty_cycle
                self.running = True

            def ChangeFrequency(self, frequency: float) -> None:
                self.frequency = frequency

            def ChangeDutyCycle(self, duty_cycle: float) -> None:
                self.duty_cycle = duty_cycle

            def stop(self) -> None:
                self.running = False

        def PWM(self, pin: int, frequency: float):
            pwm = self._MockPWM(pin, frequency)
            self._pwms[pin] = pwm
            return pwm

        def cleanup(self, pins: Iterable[int] | int | None = None) -> None:
            if pins is None:
                self._pins.clear()
                return
            if isinstance(pins, int):
                pins = [pins]
            for pin in pins:
                self._pins.pop(pin, None)

    _REAL_GPIO = _MockGPIO()


GPIO = _REAL_GPIO


class A4988:
    """Control helper for the A4988 stepper driver.

    Parameters are kept intentionally explicit so newcomers immediately see
    what each value represents.  ``gear_ratio`` describes the reduction of the
    gearbox that sits on top of the stepper motor.  A value above ``1`` slows
    the output shaft down and increases torque, which is exactly what the
    PiLiDAR build needs for smooth rotations.
    """

    def __init__(
        self,
        dir_pin: int,
        step_pin: int,
        ms_pins: Iterable[int],
        delay: float = 0.001,
        step_angle: float = 1.8,
        microsteps: int = 16,
        gear_ratio: float = 1.0,
        enable_pin: Optional[int] = None,
        pwm_frequency: Optional[float] = None,
        use_pwm: bool = False,
        verbose: bool = False,
    ) -> None:

        # The warning output of the GPIO library can be confusing for
        # beginners.  We therefore disable it by default and only enable it
        # when ``verbose`` is ``True``.
        GPIO.setwarnings(verbose)

        # The BCM numbering scheme matches the pin labels of the Raspberry Pi.
        GPIO.setmode(GPIO.BCM)

        # Store pin assignments.
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.ms_pins = list(ms_pins)
        self.enable_pin = enable_pin

        # Configure the GPIO pins as outputs.
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        for pin in self.ms_pins:
            GPIO.setup(pin, GPIO.OUT)
        if self.enable_pin is not None:
            GPIO.setup(self.enable_pin, GPIO.OUT)
            # Enable-Pin ist low-aktiv.  Wir lassen den Treiber deaktiviert,
            # bis tatsächlich Bewegungen anstehen.
            GPIO.output(self.enable_pin, GPIO.HIGH)

        # Internal state that allows us to keep track of the absolute angle.
        self.current_steps = 0

        # Motor characteristics and behaviour tuning.
        self.step_angle = step_angle
        self.microsteps = microsteps
        self.gear_ratio = gear_ratio
        self.delay = delay
        self._supports_pwm = hasattr(GPIO, "PWM")
        self.use_pwm = bool(use_pwm and self._supports_pwm)
        self._pwm: Optional[object] = None
        # PWM-Frequenz: aus der eingestellten Verzögerung berechnet oder vom
        # Aufrufer vorgegeben.  Ein Minimum von 1 Hz verhindert Division durch 0.
        base_frequency = 1.0 / max(self.delay, 0.0001)
        self.pwm_frequency = max(pwm_frequency or base_frequency, 1.0)
        # sehr kurze Bewegungen arbeiten stabiler mit manuellen Einzelimpulsen
        self._pwm_min_steps = 8

        # Microstepping allows very gentle motion because the driver performs
        # fractional steps.  The mapping below comes straight from the
        # datasheet and is written down here so the logic is easy to follow.
        self.step_modes = {
            1: [False, False, False],
            2: [True, False, False],
            4: [False, True, False],
            8: [True, True, False],
            16: [True, True, True],
        }

        if self.microsteps in self.step_modes:
            for pin, state in zip(self.ms_pins, self.step_modes[self.microsteps]):
                GPIO.output(pin, state)

    # ------------------------------------------------------------------
    # Helper functions for enable/PWM handling
    # ------------------------------------------------------------------
    def enable(self) -> None:
        """Activate the stepper driver."""

        if self.enable_pin is not None:
            GPIO.output(self.enable_pin, GPIO.LOW)

    def disable(self) -> None:
        """Disable the stepper driver."""

        if self.enable_pin is not None:
            GPIO.output(self.enable_pin, GPIO.HIGH)

    def _ensure_pwm(self):
        if not self._supports_pwm:
            raise RuntimeError("PWM support not available on this platform")
        if self._pwm is None:
            self._pwm = GPIO.PWM(self.step_pin, self.pwm_frequency)
        return self._pwm

    def set_direction(self, direction: bool) -> None:
        """Set the turning direction of the motor shaft."""

        GPIO.output(self.dir_pin, direction)

    def get_steps_for_angle(self, angle: float) -> int:
        """Translate an angle in degrees to the required number of micro steps."""

        return int((angle / self.step_angle) * self.microsteps * self.gear_ratio)

    def get_angle_for_steps(self, steps: int) -> float:
        """Translate a step count back into degrees for human readable output."""

        return (steps / (self.microsteps * self.gear_ratio)) * self.step_angle

    def step(self) -> None:
        """Perform a single micro step.

        The very short pulse on ``step_pin`` triggers the driver.  Afterwards we
        wait ``self.delay`` seconds so that the heavy scanner construction has
        time to settle.  Slow and steady motion prevents vibrations when the
        gearbox is engaged.
        """

        self.enable()
        GPIO.output(self.step_pin, True)
        sleep(0.0001)
        GPIO.output(self.step_pin, False)
        sleep(self.delay)

    def move_steps(self, steps: int) -> None:
        """Move the motor by a given amount of micro steps.

        Negative values rotate clockwise, positive values rotate
        counter-clockwise.  The method keeps track of the absolute position so
        later calls to :meth:`move_to_angle` know where the scanner is.
        """

        direction = steps < 0
        steps = abs(int(steps))

        if steps == 0:
            return

        self.set_direction(direction)
        if self.use_pwm and self._supports_pwm and steps >= self._pwm_min_steps:
            self._move_steps_pwm(steps, direction)
        else:
            self._move_steps_manual(steps, direction)

    def move_angle(self, angle: float) -> int:
        """Convenience wrapper that accepts a rotation in degrees."""

        steps = self.get_steps_for_angle(abs(angle))
        self.move_steps(steps if angle >= 0 else -steps)
        return steps

    # ------------------------------------------------------------------
    # internal movement helpers
    # ------------------------------------------------------------------
    def _move_steps_manual(self, steps: int, direction: bool) -> None:
        self.enable()
        for _ in range(steps):
            self.step()
        self.current_steps += steps if not direction else -steps

    def _move_steps_pwm(self, steps: int, direction: bool) -> None:
        self.enable()
        pwm = self._ensure_pwm()
        pwm.ChangeFrequency(self.pwm_frequency)
        pwm.ChangeDutyCycle(50.0)
        pwm.start(50.0)

        # Ein PWM-Zyklus liefert genau einen Step-Impuls (Rising Edge).
        duration = steps / self.pwm_frequency
        sleep(duration)
        pwm.stop()
        GPIO.output(self.step_pin, False)

        self.current_steps += steps if not direction else -steps

    def move_to_angle(self, target_angle: float, mod: bool = True) -> None:
        """Rotate the platform to an absolute target angle.

        ``mod`` keeps the result within 0° and 360° which avoids confusing
        values after multiple full rotations.
        """

        if mod:
            target_angle %= 360
        current_angle = self.get_current_angle()
        angle_difference = target_angle - current_angle

        # Find the shortest path to the target
        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360

        steps_difference = self.get_steps_for_angle(angle_difference)
        self.move_steps(steps_difference)

    def get_current_angle(self, mod: bool = True) -> float:
        """Return the current angle of the scanner platform."""

        current_angle = self.current_steps / (self.microsteps * self.gear_ratio) * self.step_angle

        if mod:
            current_angle %= 360
        return current_angle

    def close(self) -> None:
        """Release the GPIO pins."""

        if self._pwm is not None:
            try:
                self._pwm.stop()
            except AttributeError:
                pass
            self._pwm = None

        self.disable()
        GPIO.cleanup(self.ms_pins)
        GPIO.cleanup(self.dir_pin)
        GPIO.cleanup(self.step_pin)
        if self.enable_pin is not None:
            GPIO.cleanup(self.enable_pin)


if __name__ == "__main__":
    import numpy as np

    from config import Config

    config = Config()
    config.init(scan_id="_")

    config.update_target_res(1.)
    
    scan_delay = config.get("STEPPER", "SCAN_DELAY")  # 1 / (SAMPLING_RATE * TARGET_RES / 360)
    
    # initialize stepper
    stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"),
                    config.get("STEPPER", "pins", "STEP_PIN"),
                    config.get("STEPPER", "pins", "MS_PINS"),
                    delay      = config.get("STEPPER", "STEP_DELAY"),  # 0.001
                    step_angle = config.get("STEPPER", "STEP_ANGLE"),  # 360 / STEPPER_RES
                    microsteps = config.get("STEPPER", "MICROSTEPS"),  # 16
                    gear_ratio = config.get("STEPPER", "GEAR_RATIO"),
                    enable_pin = config.get("STEPPER", "pins", "ENABLE_PIN", default=None),
                    use_pwm    = True)  # demonstriert den PWM-Betrieb


    # # TEST: MOVE FULL 360° FORWARD AND BACKWARD
    # while True:
    #     stepper.move_steps(config.microsteps_per_revolution)
    #     sleep(0.2)
    #     stepper.move_steps(-config.microsteps_per_revolution)
    #     sleep(0.2)


    try:
        # 360° SHOOTING PHOTOS
        for i in range(4):
            print(f"Photo {i+1} at {round(stepper.get_current_angle(), 2)}°")
            sleep(0.5) # delay for photo
            stepper.move_to_angle(90 * (i+1))
        
        stepper.move_to_angle(0)
        stepper.move_steps(1)  # compensate negative value caused by rounding
        sleep(1)

        # 180° SCAN
        print(f"starting angle: {round(stepper.get_current_angle(), 2)}°")
        start_angle = 0 if config.SCAN_ANGLE > 0 else abs(config.SCAN_ANGLE)
        for z_angle in np.linspace(start_angle, config.SCAN_ANGLE, int(abs(config.SCAN_ANGLE)/config.h_res), endpoint=False):
            sleep(scan_delay) # duration of one lidar duration
            stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)

        print(f"reached {round(stepper.get_current_angle(), 2)}° (current steps: {stepper.current_steps}), returning ..")
        stepper.move_to_angle(0)
        
    finally:
        stepper.close()
