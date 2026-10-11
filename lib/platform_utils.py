import sys
import os
import subprocess


def get_platform():
    platform = sys.platform

    if "win32" in platform:
        return 'Windows'
    elif 'darwin' in platform:
        return 'Mac'
    elif 'linux' in platform:
        # Check Raspberry Pi hardware/device-tree model
        try:
            for dt_path in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
                if os.path.exists(dt_path):
                    with open(dt_path, 'r', errors='ignore') as f:
                        if 'Raspberry Pi' in f.read():
                            return 'RaspberryPi'
            if os.path.exists('/etc/rpi-issue'):
                return 'RaspberryPi'
        except Exception:
            pass
        return 'Linux'
    else:
        try:
            machine = os.uname().machine
            if 'Pico W' in machine:
                return 'Pico W'
            elif 'Pico' in machine:
                return 'Pico'
            elif 'MIMXRT1011' in machine:
                return 'Metro M7'
        except Exception:
            pass
    return platform


def get_hardware_model():
    """Best-effort hardware description for scan provenance.

    On a Pi this is the device-tree model (e.g. "Raspberry Pi 4 Model B Rev 1.4");
    elsewhere it falls back to the OS machine string. Never raises.
    """
    try:
        for dt_path in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
            if os.path.exists(dt_path):
                with open(dt_path, 'r', errors='ignore') as f:
                    model = f.read().strip('\x00\n ').strip()
                    if model:
                        return model
    except Exception:
        pass
    try:
        return f"{sys.platform} {os.uname().machine}"
    except Exception:
        return sys.platform


platform = get_platform()

if platform in ['Windows', 'Mac', 'Linux', 'RaspberryPi']:
    pass

elif platform in ['Pico', 'Pico W', 'Metro M7']:
    pass    # type: ignore


def boardpin(pin):
    board = sys.modules['board']
    return getattr(board, pin)

def init_serial(port='/dev/ttyUSB0', baudrate=230400):
    ''' USB: "/dev/ttyUSB0"  GPIO: "/dev/ttyS0" '''
    import serial
    return serial.Serial(port=port, baudrate=baudrate, timeout=1.0, bytesize=8, parity='N', stopbits=1)

def init_serial_MCU(pin='GP1', baudrate=230400):
    from busio import UART          # type: ignore
    return UART(None, boardpin(pin), baudrate=baudrate, bits=8, parity=None, stop=1)


def init_pwm_Pi(pwm_channel=0, frequency=30000):
    '''pwm_channel 0: GP18, pwm_channel 1: GP19'''
    from rpi_hardware_pwm import HardwarePWM   # type: ignore
    return HardwarePWM(pwm_channel=pwm_channel, hz=frequency, chip=0)

def init_pwm_MCU(pin="GP2", frequency=30000):
    from pwmio import PWMOut        # type: ignore
    return PWMOut(boardpin(pin), frequency=frequency)


# legacy code: allow access to serial port on Raspberry Pi
def allow_serial():
    if get_platform() == "RaspberryPi":
        # Use subprocess to allow serial communication on Raspberry Pi
        sudo_command = "sudo chmod a+rw /dev/ttyS0"
        process = subprocess.Popen(sudo_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output, error
    else:
        print("[WARNING] platform is no Pi.")


if __name__ == "__main__":
    platform = get_platform()

    print("platform:", platform)
