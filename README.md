# Stream Chasers Aircraft Design and Build
* DAS and FPV system integration
* Telemetry recording, payload drop, FPV, payload landing location overlay.
* UI constructed with Glade

## Prerequisites
* ROS
* MAVROS
* OpenCV2
* GTK+ 3
* PX4 MODIFIED Firmware - Make sure to add extras.txt in /etc of the uSD card. The actuator control works outside the OFFBOARD mode as long as the FCU is armed.

## Test environment
Ubuntu 17.10 + ROS Lunar 1.13.5 (ROS Lunar does NOT support Ubuntu 17.10 yet. I had installed ROS before I upgraded to 17.10 from 17.04.)

## TODO:
* Brush up the UI.
* Modifiable CSV file, crosshair, FCU adresses

## How do I start?
* Plug in 3DR Radio or Pixhawk via USB
* If connected via 3DR Radio:
  * $ roslaunch das_fpv 3dr_radio.launch
* If connected via USB:
  * roslaunch das_fpv usb.launch
* Issues still exist dealing with ^C, so just do what you need to do to kill the process...
