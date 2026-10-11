# References

## inspirations
- [LIDAR_LD06_python_loder](https://github.com/henjin0/LIDAR_LD06_python_loder) and [Lidar_LD06_for_Arduino](https://github.com/henjin0/Lidar_LD06_for_Arduino) by Inoue Minoru ("[henjin0](https://github.com/henjin0)")
- [ShaunPrice's](https://github.com/ShaunPrice/360-camera) StereoPi-supporting fork of [BrianBock's](https://github.com/BrianBock/360-camera) 360-camera script (Article on [Medium](https://medium.com/stereopi/stitching-360-panorama-with-raspberry-pi-cm3-stereopi-and-two-fisheye-cameras-step-by-step-guide-aeca3ff35871))

## another Lidar implementation in Python
- [pyLIDAR](https://github.com/Paradoxdruid/pyLIDAR)

## hardware PWM
using [GPIOZero](https://gpiozero.readthedocs.io/en/stable/migrating_from_rpigpio.html#pwm-pulse-width-modulation)

## ICP implementations
- Aeva [Doppler-ICP](https://github.com/aevainc/Doppler-ICP/blob/main/README.md)
- Photogrammetry & Robotics Bonn [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Lidar-Visualizer](https://github.com/PRBonn/lidar-visualizer)

## 3D Demo Data
for global registration, ICP, meshing etc.:
- [BunnyMesh.ply](https://github.com/isl-org/open3d_downloads/releases/download/20220201-data/BunnyMesh.ply) from [20220201-data](https://github.com/isl-org/open3d_downloads/releases/tag/20220201-data)
- [DemoICPPointClouds.zip](https://github.com/isl-org/open3d_downloads/releases/download/20220301-data/DemoICPPointClouds.zip) from [20220301-data](https://github.com/isl-org/open3d_downloads/releases/tag/20220301-data)

## hardware
- Using a MOSFET for switching: [tutorial](https://elinux.org/RPi_GPIO_Interface_Circuits#Using_a_FET)
- A4988 Enable, Sleep and Reset [tutorial](https://www.youtube.com/watch?v=PMS5jY7RTjo)
