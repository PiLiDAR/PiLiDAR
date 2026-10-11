# Hardware: Bill of Materials, Wiring & LiDAR Specs

## Bill of Materials

- Lidar:
  - One of:
    - LDRobot LD06: $80 ([link](https://www.aliexpress.us/item/3256803352905216.html))
    - LDRobot LD19: $70 ([link](https://www.amazon.com/DTOF-D300-Distance-Obstacle-Education/dp/B0B1V8D36H))
    - LDRobot STL27L: $160 ([link](https://www.dfrobot.com/product-2726.html))
- Camera and Lens:
  - Raspberry Pi HQ Camera with ArduCam M12 Lens: $60 ([M25156H18, p.7](https://www.arducam.com/doc/Arducam_M12_Lens_Kit_for_Pi_HQ_Camera.pdf)) ([link](https://www.arducam.com/doc/Arducam_M12_Lens_Kit_for_Pi_HQ_Camera.pdf))
- Raspberry Pi 4: $50
- NEMA17 42-23 stepper with A4988 driver: $10 ([link](https://www.amazon.com/SIMAX3D-Nema17-Stepper-Motor/dp/B0CQLFNSMJ))
- Power Supply:
  - One of:
    - [v1]: 2x _18650_ Batteries (7.2V) with step-down converter
    - [v2]: 10.000 mAh USB Powerbank with step-up converter

*Total:* \~ $200 - $280 (in April of 2025) (not including power supply)

(links are only for example, not necessarily the recommended way to purchase)

![PiLiDAR v1](../images/pilidar_covershot.jpg)
*Rev. 1 using 2x 18650 Batteries and Buck Converter*


![PiLiDAR v2](../images/pilidar_covershot_v2.jpg)
*Rev. 2 using 10.000 mAh Powerbank and Boost Converter*


## stepper driver, motor and gearbox

- A4988 bipolar stepper driver ([tutorial](https://www.youtube.com/watch?v=PMS5jY7RTjo))
- NEMA17 42x42x23 bipolar stepper ([17HE08-1004S](https://www.omc-stepperonline.com/e-series-nema-17-bipolar-1-8deg-17ncm-24-07oz-in-1a-42x42x23mm-4-wires-17he08-1004s), 17 Ncm torque)
- 3D-printed planetary reduction gearbox (see [FDM / 3D printing](#fdm--3d-printing))


## LDRobot LiDAR Specs

![LD06 vs. STL27L](../images/lidar_comparison.jpg)
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


## wiring

![breadboard version 2](../images/pilidar_breadboard.jpg)
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

> **LiDAR speed (PWM) control is LD06-only.** The LD06 was wired directly to the
> Pi's UART0 + PWM0 (GP15/GP18), so hardware PWM on GP18 controlled its spin
> speed (`LIDAR.LD06.PWM_COEFFS` = the curve-fit coefficients; `GPIO_SERIAL.ENABLE`
> gates it). The **STL27L uses its own USB controller board**, which has no PWM
> input — it spins at its fixed rate, so PWM control does not apply
> (`STL27L.PWM_COEFFS` is `null`).


## FDM / 3D printing

### 3D model files:
- ! Housing and additional parts (obj and 3mf)  in [the PiLiDAR-Hardware Repo](https://github.com/PiLiDAR/PiLiDAR-Hardware/tree/main/FDM) !

- M12 to C-Mount lens adapter ([thingiverse.com](https://www.thingiverse.com/thing:4444398))

- NEMA17 planetary reduction gearbox ([printables.com](https://www.printables.com/de/model/782336-nema17-planetary-gearbox-fixed))

![CAD model](../images/CAD_v2.jpg)
*Housing CAD model Rev. 2*




![3D printing](../images/FDM.jpg)
*3D printing the old front panel (Rev. 1) in PETG*
