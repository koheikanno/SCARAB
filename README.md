# Stream Chasers Aircraft Design and Build
* DAS and FPV system integration
* Telemetry recording, payload drop, FPV, payload landing location overlay.
* UI constructed with Glade
* Since I started using git halfway through the development, the master branch is a mess.

## Prerequisites
* ROS
* MAVROS
* OpenCV2
* GTK+ 3

## Test environment
Ubuntu 17.10 + ROS Lunar 1.13.5 (ROS Lunar does NOT support Ubuntu 17.10 yet. I had installed ROS before I upgraded to 17.10 from 17.04.)

## TODO:
* Test the AUX1 PWM output with the modified [PX4 Firmware](https://github.com/koheikanno/Firmware) for payload drops.
* Brush up the UI.
* Open and close FPV window from the UI
* Create a .launch file.
* Clean up for readability
* Clean up master branch
* Classify stray functions
* Modifiable CSV file, crosshair, FCU adresses
* Check or create a new ROS package

## How do I start?
* Plug in 3DR Radio or Pixhawk via USB
* Modify px4.launch if necessary
  * $ roscd mavros/launch
  * $ sudo nano px4.launch
  * In the line <arg name = "fcu_url" default="/dev/ttyUSB0:57600" />
    * If connected to Pixhawk directly via USB, leave it as is.
    * If connected to Pixhawk via 3DR Radio, change ttyUSB0:57600 to ttyACM0:57600
  * ^X to close nano. Make sure to save the buffer
* $ roslaunch mavros px4.launch
* Run src/das_fpv/src/telemetry.py for now. Proper ROS package to be added.
