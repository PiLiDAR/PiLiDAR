# Setup Guide

## Python Environment (venv)
PiLiDAR uses a plain `python3 -m venv` on both the desktop dev PC and the
Raspberry Pi — no conda/miniforge needed. On the Pi, create the venv from the
**system** `python3` so it matches the OS camera stack (picamera2/libcamera):

    sudo apt-get install -y python3-venv
    python3 -m venv ~/venv
    ~/venv/bin/pip install --upgrade pip
    cd ~/PiLiDAR
    ~/venv/bin/pip install -r requirements.txt

On the Pi, bridge the system `picamera2`/`libcamera` packages into the venv
(one-time, replaces the old `PYTHONPATH=/usr/lib/python3/dist-packages` prefix):

    echo "/usr/lib/python3/dist-packages" > ~/venv/lib/python3.11/site-packages/pilidar-system-bridge.pth

Run everything with `~/venv/bin/python` (e.g. `~/venv/bin/python PiLiDAR.py`).
On the desktop, use your own venv the same way.

## Power Button (Wakeup & Shutdown)
- Wakeup is hardwired to Pin 3
- enable gpio-shutdown

        echo "dtoverlay=gpio-shutdown" >> /boot/firmware/config.txt

- if necesessary:

        sudo nano /etc/systemd/logind.conf
        HandlePowerKey=poweroff

## enable i2c-GPIO for GY-521 Accelerometer

GY-521 (MPU 6060): Accelerometer, Gyroscope and thermometer
i2c adress: 0x68
![GY-521](https://www.makershop.de/download/MPU6050-Pinout.png)

Since GPIO3 is hardwired to the Power Button, we need to use i2c-GPIO to map custom i2c pins ([tutorial](https://www.instructables.com/Raspberry-PI-Multiple-I2c-Devices/)). Unlike serial is not getting crossed, so we connect SDA-SDA and SCL-SCL.
SDA: GPIO22
SCL: GPIO27

disable ic2_arm and enable i2c-gpio in /boot/firmware/config.txt

    dtparam=i2c_arm=off
    dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=1,i2c_gpio_sda=22,i2c_gpio_scl=27

search for devices on i2c bus 3:

    sudo i2cdetect -y 3

## Power LED and CPU fan

    # CPU fan at lower temp
    echo "dtoverlay=gpio-fan,gpiopin=4,temp=45000" >> /boot/firmware/config.txt


    # Power LED Heartbeat:
    echo "dtparam=pwr_led_trigger=timer" >> /boot/firmware/config.txt

## Scan Button: register GPIO interrupt
make script executable:

    chmod +x gpio_interrupt.py


create new service for autostart

    sudo nano /etc/systemd/system/pilidar.service

content:

    [Unit]
    Description=PiLiDAR-Button
    After=network.target

    [Service]
    Type=simple
    User=pi
    Environment=LG_WD=/tmp
    ExecStart=/home/pi/venv/bin/python /home/pi/PiLiDAR/gpio_interrupt.py
    Restart=no

    [Install]
    WantedBy=multi-user.target

reload daemon, enable and start service:

    sudo systemctl daemon-reload
    sudo systemctl enable pilidar.service
    sudo systemctl start pilidar.service

check service if necessary:

    sudo systemctl status pilidar.service


## Pre-flight Hardware Checks
Before a scan, run the diagnostics to catch a dead LiDAR, a loose I2C jumper,
a camera that failed to enumerate, or a full disk:

    ~/venv/bin/python -m lib.diagnostics

It checks the Python version, platform, free storage, Hugin tools, the LiDAR
UART packet stream, the MPU6050 over I2C, camera enumeration, and GPIO access,
printing one line per check. Use `--json` for machine-readable output,
`--lidar-seconds N` to sample the UART longer, and `--skip lidar imu ...` to
skip checks. The exit code is `1` if any check FAILs (WARN/SKIP are
non-fatal), so it can gate a scan in a wrapper script.

## set Permission for UART on Raspberry Pi
temporary solution:

    sudo chmod a+rw /dev/ttyS0

#### old solution: make it permanent by disabling password for chmod:

    sudo visudo
    pi ALL=(ALL:ALL) NOPASSWD: /usr/bin/chmod a+rw /dev/ttyS0

then execute the _temporary_ solution from python:

    import subprocess
    command = "sudo chmod a+rw /dev/ttyS0"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

#### new solution: grant permissions for the serial port using udev rules

(TODO: check and remove old!)
- forget about `visudo` and the subprocess call above.
- Open a terminal and run the following command: `sudo nano /etc/udev/rules.d/50-ttyS0.rules`
- Write the following line in the file and save it: `KERNEL=="ttyS0",GROUP="dialout",MODE="0660"`
- Run the following command to check if your user is a member of the dialout group: `groups`
- If you see `dialout` in the output, you are already a member of the group. If not, run the following command to add your user to the group: `sudo usermod -a -G dialout pi`
- Run the following command to reload the udev rules: `sudo udevadm control --reload-rules`
- Unplug and replug the serial device, or reboot the system, to apply the changes.

## Hardware PWM on Raspberry Pi
enable GPIO_18 (PWM0) and GPIO_19 (PWM1)

    echo "dtoverlay=pwm-2chan" >> /boot/firmware/config.txt

check if "pwm_bcm2835" now exists:

    lsmod | grep pwm

Install [RPi Hardware PWM library](https://github.com/Pioreactor/rpi_hardware_pwm):

    pip install rpi-hardware-pwm


## Panorama Stitching
PiLiDAR calls `hugin_executor` from `PATH`, so install the command-line tools
and Enblend from the OS package manager. The newer Hugin desktop application is
installed separately from Flathub.

    sudo apt-get update
    sudo apt-get install -y flatpak hugin-tools enblend
    flatpak remote-add --user --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo
    flatpak install --user flathub net.sourceforge.Hugin

This installs Hugin 2025.0.1 for the current user on supported x86_64 and
ARM64 systems. The Flathub build is community-provided; the Hugin project links
to it as a Linux option.

## Meshing (pymeshlab)
PiLiDAR does all meshing headlessly through [**pymeshlab**](https://pymeshlab.readthedocs.io/)
(the Python interface to MeshLab) — no GUI, no `meshlabserver`, no `.mlx`
scripts. It is a regular pip package with wheels for both x86_64 and aarch64,
so the same `~/venv` works on the dev PC and the Pi:

    ~/venv/bin/pip install pymeshlab

pymeshlab's plugins link against the Mesa OpenGL runtime, so install it once
per machine (the Pi usually already has it):

    sudo apt-get install -y libopengl0 libgl1 libegl1

The meshing pipeline (point-cloud simplification → normal smoothing → Screened
Poisson surface reconstruction → Laplacian smoothing → Hausdorff-based cleanup
→ Z-up to Y-up rotation, plus optional UV projection / texture baking) is
implemented in `meshlab/pymeshlab_test.py`:

    ~/venv/bin/python meshlab/pymeshlab_test.py

> The older `meshlab/meshlabserver_test.sh` + `.mlx` scripts (which needed the
> apt `meshlab` 2020.09 package and OpenGL) are superseded by pymeshlab and
> kept only for reference.


## power switching the USB port (LiDAR idle power-off)

The STL27L is powered over USB: cutting USB VBUS while idle turns it fully off
(no red spin, no power draw) — this replaces the old LD06 power relay.

**On the Raspberry Pi 4, only *whole-hub* switching actually cuts VBUS.** The
USB-A power switch lives in the VL805 USB3 controller (bus 2), and per-port
(`-p`) / exact-location (`-e`) flags merely drop the USB *data* link while the
LiDAR keeps drawing power (verified on hardware: the blue LED stays on). The
working commands are:

    sudo uhubctl -l 2 -a 0     # OFF — all four USB-A ports
    sudo uhubctl -l 2 -a 1     # ON

Note this powers down **all** USB-A ports together (both the blue 3.0 and black
2.0 ports), so do not put anything on USB-A that must stay powered while the
LiDAR is idle. Reference:
[how can I cut power out of the Pi's USB ports](https://raspberrypi.stackexchange.com/questions/5407#74906).

Using the [uhubctl](https://github.com/mvp/uhubctl) cli tool:

    sudo apt-get install uhubctl

list all hubs, ports and devices:

    sudo uhubctl

PiLiDAR calls this automatically. Configure the hub in `config.json`:

    "USB_POWER": { "ENABLE": true, "HUB": "2", "DELAY": 5 }

`PiLiDAR.py` powers the hub on before opening the serial port (so the LiDAR
enumerates rather than the port falling back to `/dev/ttyS0`) and cuts it again
before post-processing; `gpio_interrupt.py` cuts it, too, if a scan process died
before its own cleanup. See `lib/usb_power.py`. `uhubctl` must be runnable
without a password — the default Raspberry Pi OS `pi` user already has
`NOPASSWD: ALL`.

> Power-cycling renumbers the serial device (`ttyUSB0` → `ttyUSB1` → …), so the
> LiDAR path is resolved from the stable `/dev/serial/by-id/...` link (set
> `LIDAR.STL27L.PORT_BY_ID` in `config.json`); the raw `PORT` is only a fallback.


## jupyter over remote-ssh
start jupyter for network access:

    jupyter notebook --ip 192.168.1.16 --no-browser PiLiDAR.ipynb
