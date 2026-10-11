'''
A4988 datasheet:
http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx

microstepping:
https://i.stack.imgur.com/vN7JL.png
'''

from gpiozero import OutputDevice       # type: ignore
import time


class A4988:
    def __init__(self, dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=1.0):
        self.dir_pin = OutputDevice(dir_pin)

        self.step_pin = OutputDevice(step_pin)

        self.ms_pins = [OutputDevice(pin) for pin in ms_pins]

        self.step_angle = step_angle
        self.microsteps = microsteps
        self.gear_ratio = gear_ratio
        self.delay = delay

        self.step_modes = {1: [False, False, False],
                           2: [True, False, False],
                           4: [False, True, False],
                           8: [True, True, False],
                           16: [True, True, True]}

        if self.microsteps in self.step_modes:
            for pin, state in zip(self.ms_pins, self.step_modes[self.microsteps]):
                pin.value = state

    def set_direction(self, direction):
        self.dir_pin.value = direction

    def step(self):
        self.step_pin.on()
        time.sleep(0.0001)
        self.step_pin.off()
        time.sleep(self.delay)

    def move_steps(self, steps):
        direction = steps > 0
        steps = abs(int(steps))
        
        self.set_direction(direction)
        for _ in range(steps):
            self.step()

    def move_angle(self, angle):
        steps = int((angle / self.step_angle) * self.microsteps * self.gear_ratio)
        self.move_steps(steps)


if __name__ == "__main__":
    dir_pin = 26
    step_pin = 19
    ms_pins = [5, 6, 13]
    gear_ratio = 1 + 38/14

    driver = A4988(dir_pin, step_pin, ms_pins, delay=0.001, step_angle=1.8, microsteps=16, gear_ratio=gear_ratio)

    while True:
        driver.move_angle(360)
        time.sleep(0.2)
        
        # driver.move_angle(-360)
        # time.sleep(0.2)
