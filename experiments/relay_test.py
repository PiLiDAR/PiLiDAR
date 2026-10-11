import RPi.GPIO as GPIO  # type: ignore
from time import sleep


GPIO.setwarnings(False)  # Disable warnings

GPIO.setmode(GPIO.BCM)
# GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.OUT)

try:  
    while True:
        GPIO.output(24, 1)
        print("on")
        sleep(2)
        GPIO.output(24, 0)
        print("off")
        sleep(2)


        # if not GPIO.input(4):
        #     GPIO.output(24, 1)
        #     sleep(0.1)
        # else:  
        #     GPIO.output(24, 0)
        #     sleep(0.1)
  
finally:
    GPIO.cleanup()
