# Troubleshooting & References

## Troubleshooting

### Windows Serial Driver
get [CP210x_Universal_Windows_Driver.zip](https://files.waveshare.com/upload/6/63/CP210x_Universal_Windows_Driver.zip) here:
https://www.waveshare.com/wiki/DTOF_LIDAR_STL27L#Software_Download

### RPi.GPIO RuntimeError: Failed to add edge detection
The Bookworm kernel removed the deprecated sysfs GPIO interface that `RPi.GPIO`
needs. PiLiDAR no longer uses `RPi.GPIO` at all — `gpio_interrupt.py`,
`lib/a4988_driver.py` and the relay setup in `lib/config.py` are all built on
the [lgpio](https://pypi.org/project/lgpio/) C-API (installed in the venv via
`requirements.txt`), which talks to the modern `gpiochip` character devices:

    pip install lgpio

If a legacy script still fails with `ModuleNotFoundError: No module named 'RPi'`,
it is importing `RPi.GPIO` — port it to lgpio (see `lib/a4988_driver.py` for the
output-pin pattern: `gpiochip_open` → `gpio_claim_output` → `gpio_write`) or run
it through the `rpi-lgpio` compatibility shim.

lgpio creates temp-files ([issue](https://github.com/joan2937/lg/issues/12)) like ".lgd-nfy0". gpio-interrupt.py executes 'export LG_WD=/tmp' to set it's CWD.


### poor performance of VS Code on Raspberry Pi

disable hardware acceleration for VS Code ([source](https://code.visualstudio.com/docs/setup/raspberry-pi))

    Preferences: Configure Runtime Arguments
    Set "disable-hardware-acceleration": true

### pye57 on Raspberry Pi
there is no wheel for arm64. build requires libxerces:

    sudo apt install libxerces-c-dev
    pip install pye57

### add WIFI via SSH

[tutorial](https://u-labs.de/portal/raspberry-pi-wlan-verbindung-nachtrglich-einrichten-oder-aendern-so-geht-es-grafisch-konsole/):

    sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

    # make sure country code is set:
    country=DE

add entry to wpa_supplicant.conf

    sudo wpa_passphrase "YOUR_SSID" "YOUR_PASSWORD" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf


## references

see [references.md](references.md)
