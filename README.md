# PiLiDAR - DIY 360° 3D Panorama Scanner
## _WORK IN PROGRESS_

## Core Features:
- **LiDAR**: custom serial driver for LDRobot **LD06**, **LD19** or **STL27L**
    - CRC package integrity check
    - [Hardware PWM](https://github.com/Pioreactor/rpi_hardware_pwm) calibrated using curve fitting
    - 2D live visualization and export (numpy or CSV)

- **Panorama**: 6K 360° spherical map
    - stitched from fisheye photos using [**Hugin** Panorama photo stitcher](https://hugin.sourceforge.io/)
    - constant camera exposure by reading EXIF data of automatic
    - constant white balance by iterative optimization of color gains

- **3D Scene**: assembly of 3D scenes from 2D planes based on angle and offsets
    - sampling **vertex colors from panorama**
    - Open3D visualization and export (PCD, PLY or [e57](https://github.com/davidcaron/pye57))
    - aligning multiple scenes using **global registration** and **ICP fine-tuning**
    - **Poisson Surface Meshing** (very slow on Pi4, recommended to run on PC)

## preliminary results
single scans, no registration, no post processing.  
klick the images to open the pointclouds in Sketchfab.

[![Exterior](images/exterior.jpeg)](https://sketchfab.com/models/7997b63a3cb747f99b8f161862318bec/embed?autostart=1&ui_animations=0&ui_stop=0&ui_inspector=0&ui_watermark_link=0&ui_watermark=0&ui_ar=0&ui_help=0&ui_settings=0&ui_vr=0&ui_fullscreen=0&ui_annotations=0)  

*Exterior Scan (colormapped Intensity)*

[![Interior](images/interior.jpeg)](https://sketchfab.com/models/0311c098c57b458abe3a5d3dda9fe92b/embed?autospin=0&autostart=1&ui_animations=0&ui_inspector=0&ui_watermark_link=0&ui_watermark=0&ui_ar=0&ui_help=0&ui_settings=0&ui_vr=0&ui_fullscreen=0&ui_annotations=0)  

*Interior Scan (Vertex Colors)*


## Web Dashboard (v0.9.1-beta)

PiLiDAR includes a comprehensive web-based control center accessible from any device on your network.

### Features
- **Live LiDAR Visualization**: Real-time polar plots with quality metrics
- **One-Click Scanning**: Start complete 3D scans with a single button
- **Configuration Editor**: Modify settings via user-friendly web interface
- **Scan History**: Browse and download previous scans
- **System Monitor**: Track CPU temperature, uptime, and storage

### Access Dashboard
```bash
# Start dashboard (auto-starts on boot if enabled)
python3 tools/web_dashboard.py --port 5000

# Access from any device
http://192.168.0.70:5000
```

### Start a Full Scan
1. Navigate to dashboard at `http://<pi-ip>:5000`
2. Click **"🚀 Start Full Scan"** button
3. Confirm scan workflow
4. Monitor real-time progress:
   - Camera panorama capture (if enabled)
   - 180° LiDAR scanning
   - Panorama stitching
   - 3D point cloud processing
5. View results automatically (~3-5 minutes total)

### Features Detail

#### Live LiDAR View
- Interactive polar plots with zoom/pan
- Distance range slider (1-10m)
- Real-time quality metrics:
  - Point density (pts/degree)
  - Angular coverage (%)
  - Noise level (mm std dev)
  - Overall quality score (0-100)
- 3D coordinate system visualization
- Auto-updating every 50ms (20 FPS)
- SSH-compatible (no X11 required)

#### Configuration Editor
- Web-based JSON editor with validation
- 60+ parameter tooltips
- Axis inversion toggles (X/Y/Z)
- Y/Z offset calibration wizard
- Reset to defaults button
- Real-time config save

#### Scan Automation
- Non-blocking subprocess execution
- Real-time status updates (every 2 seconds)
- Progress indicators for each workflow step
- Automatic motor power management
- Error recovery with detailed messages
- Automatic result archival

### API Endpoints
```bash
# Start scan programmatically
curl -X POST http://localhost:5000/api/scan/start

# Check scan status
curl http://localhost:5000/api/scan/status

# Get system info
curl http://localhost:5000/api/system
```

For detailed documentation, see:
- **Scan Automation**: [`SCAN_AUTOMATION.md`](SCAN_AUTOMATION.md)
- **Quick Start Guide**: [`QUICKSTART_SCAN.md`](QUICKSTART_SCAN.md)


## Hardware Specs:

- LDRobot LD06, LD19 or STL27L LiDAR
- Raspberry Pi HQ Camera with ArduCam M12 Lens [(M25156H18, p.7)](https://www.arducam.com/doc/Arducam_M12_Lens_Kit_for_Pi_HQ_Camera.pdf)
- Raspberry Pi 4 or **Raspberry Pi 5** (fully compatible)
- NEMA17 42-23 stepper with A4988 driver

- Power Supply: 
    - v1: 2x _18650_ Batteries (7.2V) with step-down converter
    - v2: 10.000 mAh USB Powerbank with step-up converter


![PiLiDAR v1](images/pilidar_covershot.jpg)
*Rev. 1 using 2x 18650 Batteries and Buck Converter*


![PiLiDAR v2](images/pilidar_covershot_v2.jpg)
*Rev. 2 using 10.000 mAh Powerbank and Boost Converter*


### stepper driver, motor and gearbox

- A4988 bipolar stepper driver ([tutorial](https://www.youtube.com/watch?v=PMS5jY7RTjo))  
- NEMA17 42x42x23 bipolar stepper ([17HE08-1004S](https://www.omc-stepperonline.com/e-series-nema-17-bipolar-1-8deg-17ncm-24-07oz-in-1a-42x42x23mm-4-wires-17he08-1004s), 17 Ncm torque)
- 3D-printed planetary reduction gearbox (see [FDM / 3D printing](#fdm--3d-printing))


### LDRobot LiDAR Specs

![LD06 vs. STL27L](images/lidar_comparison.jpg)
*angular resolution of LD06 (left) vs. STL27L (right)*

LD06: 
- sampling frequency: 4500 Hz
- baudrate 230400
- [Sales page](https://www.inno-maker.com/product/lidar-ld06/)
- [mechanical Datasheet](https://www.inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf)
- [Protocol Description](https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf)

STL27L:
- sampling frequency: 21600 Hz
- baudrate 921600
- [datasheet](https://github.com/May-DFRobot/DFRobot/blob/master/SEN0589_Datasheet.pdf)
- [wiki](https://www.waveshare.com/wiki/DTOF_LIDAR_STL27L)
- ROS2 driver [git](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2?tab=readme-ov-file#Instructions)

Scan duration:
12s initialisation
17s shooting 4x photos
1:24m scanning 0.167° x 0.18°
37s stitching, cleanup


## Sensor Position Configuration

To generate correct 3D point cloud output, the physical positions of the LiDAR and camera must be defined in `config.json`. The coordinate system uses a Z-up convention where:
- **X-axis**: points forward (LiDAR scanning direction)
- **Y-axis**: points to the left
- **Z-axis**: points up (rotation axis of the stepper motor)

![Coordinate System](images/CAD_v2.jpg)
*CAD model showing sensor arrangement*

### Required Parameters in config.json

#### 1. LiDAR Y-Offset (`3D.Y_OFFSET`)
The horizontal distance from the rotation axis (stepper motor shaft) to the LiDAR sensor in millimeters.
- **Positive values**: LiDAR is positioned towards the left of the rotation axis
- **Negative values**: LiDAR is positioned towards the right of the rotation axis

```json
"3D": {
    "Y_OFFSET": -37.5,
    ...
}
```
*Example: The LiDAR is mounted 37.5mm to the right of the rotation axis*

#### 2. LiDAR Z-Offset (`3D.Z_OFFSET`)
The vertical distance from the panorama camera optical center to the LiDAR sensor in millimeters.
- **Positive values**: LiDAR is positioned above the camera
- **Negative values**: LiDAR is positioned below the camera

```json
"3D": {
    ...
    "Z_OFFSET": -41.9,
    ...
}
```
*Example: The LiDAR is mounted 41.9mm below the camera optical center*

#### 3. LiDAR Angular Offset (`LIDAR.LIDAR_OFFSET_ANGLE`)
A small rotational correction for the LiDAR axis in degrees. This compensates for mechanical assembly imperfections.
- Adjust this value if the 3D scan appears slightly rotated relative to the expected orientation

```json
"LIDAR": {
    ...
    "LIDAR_OFFSET_ANGLE": -1.05,
    ...
}
```
*Example: A -1.05° correction is applied to compensate for mounting angle deviation*

#### 4. Vertex Color Rotation (`VERTEXCOLOUR.Z_ROTATE`)
A rotational offset in degrees to align the panorama image with the 3D point cloud when sampling vertex colors.
- Adjust this value if the colors from the panorama don't align correctly with the 3D geometry

```json
"VERTEXCOLOUR": {
    "SCALE": 0.5,
    "Z_ROTATE": 0.0
}
```
*Example: No additional rotation is needed (0.0°)*

### How to Measure Sensor Positions

1. **Measure Y_OFFSET**: 
   - Measure the horizontal distance from the center of the stepper motor shaft to the center of the LiDAR sensor
   - Use a negative value if the LiDAR is on the right side of the rotation axis

2. **Measure Z_OFFSET**:
   - Measure the vertical distance from the camera lens optical center to the LiDAR sensor plane
   - Use a negative value if the LiDAR is below the camera

3. **Calibrate LIDAR_OFFSET_ANGLE**:
   - Take a test scan of a scene with clear vertical lines (e.g., door frames, corners)
   - If vertical features appear tilted, adjust this value until they are straight

4. **Calibrate Z_ROTATE**:
   - Take a test scan with both LiDAR and camera enabled
   - If the vertex colors are misaligned (shifted horizontally), adjust this value
   - Each degree of rotation corresponds to approximately 10 pixels at 3600px panorama width

### Example Configuration

```json
{
    "LIDAR": {
        "LIDAR_OFFSET_ANGLE": -1.05,
        ...
    },
    "3D": {
        "Y_OFFSET": -37.5,
        "Z_OFFSET": -41.9,
        ...
    },
    "VERTEXCOLOUR": {
        "SCALE": 0.5,
        "Z_ROTATE": 0.0
    }
}
```


## wiring

![breadboard version 2](images/pilidar_breadboard.jpg)
*Breadboard Rev. 2*


### LD06 / STL27L:
- UART Tx (yellow)
- PWM (white)
- GND (black)
- VCC 5V (red)

### Raspberry Pi:
- LD06 UART0 Rx: GP15
- LD06 PWM0: GP18
- Power Button: GP03
- Scan Button: GP17
- A4988 direction: GP26, step: GP19
- A4988 microstepping mode: GP5, GP6, GP13


## Setup

### Power Button (Wakeup & Shutdown)
- Wakeup is hardwired to Pin 3
- enable gpio-shutdown

    
        echo "dtoverlay=gpio-shutdown" >> /boot/firmware/config.txt 

- if necesessary:  
    
        sudo nano /etc/systemd/logind.conf
        HandlePowerKey=poweroff

### enable i2c-GPIO for GY-521 Accelerometer

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

### Power LED and CPU fan

    # CPU fan at lower temp
    echo "dtoverlay=gpio-fan,gpiopin=4,temp=45000" >> /boot/firmware/config.txt
    

    # Power LED Heartbeat:
    echo "dtparam=pwr_led_trigger=timer" >> /boot/firmware/config.txt

### Scan Button: register GPIO interrupt
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
    ExecStart=/usr/bin/python3 /home/pi/PiLiDAR/gpio_interrupt.py
    Restart=no

    [Install]
    WantedBy=multi-user.target

reload daemon, enable and start service:

    sudo systemctl daemon-reload
    sudo systemctl enable pilidar.service
    sudo systemctl start pilidar.service

check service if necessary:

    sudo systemctl status pilidar.service


### set Permission for UART on Raspberry Pi
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

#### new solution: grant permissions to the serial port using udev rules 

(TODO: check and remove old!)
- forget about `visudo` and the subprocess call above.
- Open a terminal and run the following command: `sudo nano /etc/udev/rules.d/50-ttyS0.rules`
- Write the following line in the file and save it: `KERNEL=="ttyS0",GROUP="dialout",MODE="0660"`
- Run the following command to check if your user is a member of the dialout group: `groups`
- If you see `dialout` in the output, you are already a member of the group. If not, run the following command to add your user to the group: `sudo usermod -a -G dialout pi`
- Run the following command to reload the udev rules: `sudo udevadm control --reload-rules`
- Unplug and replug the serial device, or reboot the system, to apply the changes.

### Hardware PWM on Raspberry Pi

**Important:** PWM channel mapping differs between Pi 4 and Pi 5!

#### Raspberry Pi 4 and earlier
enable GPIO_18 (PWM0) and GPIO_19 (PWM1):
    
    echo "dtoverlay=pwm-2chan" >> /boot/firmware/config.txt 

#### Raspberry Pi 5
On Pi 5, GPIO18/19 are on channels 2/3 (not 0/1):
- GPIO12 → channel 0
- GPIO13 → channel 1  
- GPIO18 → channel 2
- GPIO19 → channel 3

Enable PWM for GPIO18/19:

    echo "dtoverlay=pwm-2chan" >> /boot/firmware/config.txt 

The PiLiDAR software automatically detects Pi 5 and uses the correct channel mapping.

#### Verify PWM is enabled
check if PWM kernel module is loaded:

    lsmod | grep pwm

Install [RPi Hardware PWM library](https://github.com/Pioreactor/rpi_hardware_pwm):

    pip install rpi-hardware-pwm


### Raspberry Pi 5 Specific Notes

The project is fully compatible with Raspberry Pi 5. Key differences handled automatically:

1. **GPIO Library**: Uses `gpiozero` which works on both Pi 4 and Pi 5 (uses `lgpio` backend on Pi 5)
2. **PWM Channels**: Automatic channel mapping adjustment for Pi 5
3. **GPIO Chip**: Pi 5 uses a different GPIO chip (RP1), handled transparently by gpiozero

Install required packages on Pi 5:

    pip install gpiozero rpi-lgpio rpi-hardware-pwm


### Panorama Stitching
install Hugin with enblend plugin

    sudo apt-get install hugin-tools enblend



### power switching the USB port

using [uhubctl](https://www.baeldung.com/linux/control-usb-power-supply) cli tool. install:  
    
    sudo apt-get install uhubctl

list all available hubs and devices

    sudo uhubctl

powering Raspberry Pi's USB-3-Ports (Hub 2) off / on

    sudo uhubctl -l 2 -a off
    sudo uhubctl -l 2 -a on


### jupyter over remote-ssh 
start jupyter for network access:

    jupyter notebook --ip 192.168.1.16 --no-browser PiLiDAR.ipynb


## FDM / 3D printing

### 3D model files:
- ! Housing and additional parts (obj and 3mf)  in [the PiLiDAR-Hardware Repo](https://github.com/PiLiDAR/PiLiDAR-Hardware/tree/main/FDM) !

- M12 to C-Mount lens adapter ([thingiverse.com](https://www.thingiverse.com/thing:4444398))

- NEMA17 planetary reduction gearbox ([printables.com](https://www.printables.com/de/model/782336-nema17-planetary-gearbox-fixed))

![CAD model](images/CAD_v2.jpg)
*Housing CAD model Rev. 2*




![3D printing](images/FDM.jpg)
*FDM printing the old front panel (Rev. 1) in PETG*


## Serial Protocol
### LD06
baudrate 230400, data bits 8, no parity, 1 stopbit  
sampling frequency 4500 Hz, scan frequency 5-13 Hz, distance 2cm - 12 meter, ambient light 30 kLux

total package size: 48 Byte, big endian.
- starting character：Length 1 Byte, fixed value 0x54, means the beginning of data packet;
- Data Length: Length 1 Byte, the first three digits reserved, the last five digits represent the number of measured points in a packet, currently fixed value 12;
- speed：Length 2 Byte, in degrees per second;
- Start angle: Length: 2 Byte; unit: 0.01 degree;
- Data: Length 36 Byte; containing 12 data points with 3 Byte each: 2 Byte distance (unit: 1 mm), 1 Byte luminance. For white objects within 6m, the typical luminance is around 200.
- End Angle: Length: 2 Byte; unit: 0.01 degree；
- Timestamp: Length 2 Bytes in ms, recount if reaching to MAX 30000；
- CRC check: Length 1 Byte

The Angle value of each data point is obtained by linear interpolation of the starting angle and the ending angle.  
The calculation method of the angle is as following:

    step = (end_angle – start_angle)/(len – 1)  
    angle = start_angle + step*i  

len is the length of the packet, and the i value range is [0, len].


## remote Open3D Visualization

using ~~[_Web Visualizer_](https://www.open3d.org/docs/release/tutorial/visualization/web_visualizer.html)~~ [Plotly](https://plotly.com/python/) to display 3D pointclouds works great in Jupyter.  

Plotly seems to render client-sided, unlike Open3D Web Visualizer which renders host-sided and streams jpg sequences, which strains the Pi's both CPU and WIFI.


## Dumping Scans to USB Storage

1. Clone the Repo and run the installer:
    ```
    cd /home/pi/PiLiDAR
    git clone https://github.com/LaserBorg/usb_dump --depth 1
    cd usb_dump && chmod +x install.sh && ./install.sh "$(pwd)"
    ```
2. Create the config file:
    ```
    echo '{"source_directories": ["/home/pi/PiLiDAR/scans"], "target_root_directory": null}' > usbdump.json
    ```


### Troubleshoot USB_dump:
-  Check the log file:
    ```
    tail -f /tmp/usbdump.log
    ```

- to uninstall the service, run
    ```
    chmod +x uninstall.sh && ./uninstall.sh
    ```

- if the mount point is still persistend after being removed, just delete them.
    ```
    sudo rm -rf /media/pi/<your device name>
    ```


## Troubleshooting

### Windows Serial Driver
get [CP210x_Universal_Windows_Driver.zip](https://files.waveshare.com/upload/6/63/CP210x_Universal_Windows_Driver.zip) here:  
https://www.waveshare.com/wiki/DTOF_LIDAR_STL27L#Software_Download

### GPIO on Raspberry Pi 5
PiLiDAR now uses `gpiozero` library which is compatible with both Raspberry Pi 4 and 5.
The `gpiozero` library automatically uses the correct backend:
- **Pi 4**: Uses RPi.GPIO or pigpio backend
- **Pi 5**: Uses lgpio backend (required due to new RP1 GPIO chip)

If you encounter GPIO issues on Pi 5, ensure you have the correct packages:

    pip install gpiozero rpi-lgpio

### Legacy RPi.GPIO RuntimeError (Pi 4 with Bookworm)
If using older code with RPi.GPIO on Bookworm, the deprecated sysfs GPIO interface was removed.  
Use [LGPIO](https://pypi.org/project/rpi-lgpio/) as described [here](https://raspberrypi.stackexchange.com/questions/147332/rpi-gpio-runtimeerror-failed-to-add-edge-detection):

    sudo apt remove python3-rpi.gpio
    sudo apt update

    sudo apt install python3-rpi-lgpio

    # or in an env without system packages:
    pip3 install rpi-lgpio

LGPIO creates temp-files ([issue](https://github.com/joan2937/lg/issues/12)) like ".lgd-nfy0". The pilidar.service sets LG_WD=/tmp to handle this.


### poor performance of VS Code on Raspberry Pi

disable hardware acceleration for VS Code ([source](https://code.visualstudio.com/docs/setup/raspberry-pi))

    Preferences: Configure Runtime Arguments  
    Set "disable-hardware-acceleration": true

### pye57 on Raspberry Pi
there is no wheel for arm64. build requires libxerces:

    sudo apt install libxerces-c-dev
    pip install pye57

### add WIFI via SSH

[tutorial](https://u-labs.de/portal/raspberry-pi-wlan-verbindung-nachtraeglich-einrichten-oder-aendern-so-geht-es-grafisch-konsole/):

    sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

    # make sure country code is set:
    country=DE

add entry to wpa_supplicant.conf

    sudo wpa_passphrase "YOUR_SSID" "YOUR_PASSWORD" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf



## references:

inspirations
- [LIDAR_LD06_python_loder](https://github.com/henjin0/LIDAR_LD06_python_loder) and [Lidar_LD06_for_Arduino](https://github.com/henjin0/Lidar_LD06_for_Arduino) by Inoue Minoru ("[henjin0](https://github.com/henjin0)")
- [ShaunPrice's](https://github.com/ShaunPrice/360-camera) StereoPi-supporting fork of [BrianBock's](https://github.com/BrianBock/360-camera) 360-camera script (Article on [Medium](https://medium.com/stereopi/stitching-360-panorama-with-raspberry-pi-cm3-stereopi-and-two-fisheye-cameras-step-by-step-guide-aeca3ff35871))

another Lidar implementation in Python
- [pyLIDAR](https://github.com/Paradoxdruid/pyLIDAR)

hardware PWM using [GPIOZero](https://gpiozero.readthedocs.io/en/stable/migrating_from_rpigpio.html#pwm-pulse-width-modulation)

ICP implementations:
- Aeva [Doppler-ICP](https://github.com/aevainc/Doppler-ICP/blob/main/README.md)
- Photogrammetry & Robotics Bonn [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Lidar-Visualizer](https://github.com/PRBonn/lidar-visualizer)

3D Demo Data for global registration, ICP, meshing etc.:
- [BunnyMesh.ply](https://github.com/isl-org/open3d_downloads/releases/download/20220201-data/BunnyMesh.ply) from [20220201-data](https://github.com/isl-org/open3d_downloads/releases/tag/20220201-data)
- [DemoICPPointClouds.zip](https://github.com/isl-org/open3d_downloads/releases/download/20220301-data/DemoICPPointClouds.zip) from [20220301-data](https://github.com/isl-org/open3d_downloads/releases/tag/20220301-data)

Using a MOSFET for switching: [tutorial](https://elinux.org/RPi_GPIO_Interface_Circuits#Using_a_FET)

A4988 Enable, Sleep and Reset [tutorial](https://www.youtube.com/watch?v=PMS5jY7RTjo)
