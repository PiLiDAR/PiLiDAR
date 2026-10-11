# Serial Protocol & Data Handling

## LD06 / STL27L Packet Format
baudrate 230400 (LD06) / 921600 (STL27L), data bits 8, no parity, 1 stopbit
sampling frequency 4500 Hz (LD06) / 21600 Hz (STL27L), scan frequency 5-13 Hz, distance 2cm - 12 meter, ambient light 30 kLux

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
