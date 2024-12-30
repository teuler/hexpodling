# hexpodling

"hexpodling" is a work-in-progress, 3-degrees of freedom (3DOF) hexapod robot based on the [Vorpal](https://vorpalrobotics.com/wiki/index.php/Vorpal_The_Hexapod) from Vorpal Robotics. The original Vorpal is remote-controlled and 2DOF; it is aimed at teaching in schools. 

The main differences between the original and the version described here are:
* The legs contain an additional segment, hence 3DOF
* Based on [Raspberry Pi Pico microcontrollers](https://www.raspberrypi.com/products/raspberry-pi-pico/) instead of the original's Arduino Nano, with one Pico controlling the movements (server) and one for the "higher" functions (client)
* Programmed in [PicoMite MMBasic](https://geoffg.net/picomite.html) by Peter Mather and Geoff Graham
* Supports current sensing (load) for up to 8 servos, e.g. to detect if the robot is stuck

Here is a [video](https://youtu.be/h_NlpfQ0yDU) demonstrating its movements (remote-controlled).  
For the details, see [Wiki](https://github.com/teuler/hexpodling/wiki).

### Release Notes

* 2025-01-01 - After a long pause
  * Switch to MMBasic
  * New client board featuring two Pico RP2040 microcontrollers
  * Remote control (to test movements) direcly w/ a wireless XBox controller
* 2020-12-27 - Updates - Still work in progress
  * Wiki pages 'Mechanics' and 'Boards' added
  * Code updated (now also some initial code for the client microcontroller)
  * Both boards - for the "walk engine" and for the "behavior engine" - revised (including bug fixes)
* 2020-06-27 - Initial release  
