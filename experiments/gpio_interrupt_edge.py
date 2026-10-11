import RPi.GPIO as GPIO  # type: ignore
import subprocess

BUTTON_GPIO = 17
CONDA_PATH  = "/home/pi/miniforge3/condabin/conda"
CONDA_ENV   = "py311"  # None
MAIN_SCRIPT = "/home/pi/Documents/PiDAR/PiDAR.py"

# Set up the GPIO mode and the pin direction
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize the process variable
process = None

try:
    while True:
        GPIO.wait_for_edge(BUTTON_GPIO, GPIO.FALLING)
        
        # Check if there is an existing process
        if process is not None:
            return_code = process.poll()
            # If the return code is None, the process is still running
            if return_code is None:
                print("Process is still running, skipping button press")
                continue
            else:
                print("Process has finished, return code:", return_code)
                
        # Start a new process with the main script
        call = ["python3", MAIN_SCRIPT] if CONDA_ENV is None else [CONDA_PATH, "run", "-n", CONDA_ENV, "python", MAIN_SCRIPT]
        process = subprocess.Popen(call)
        print("Started a new process, pid:", process.pid)

except KeyboardInterrupt:
    GPIO.cleanup()
