# hexpodling

"hexpodling" is a work-in-progress, 3-degrees of freedom (3DOF) hexapod robot based on the [Vorpal](https://vorpalrobotics.com/wiki/index.php/Vorpal_The_Hexapod) from Vorpal Robotics. The original Vorpal is remote-controlled and 2DOF; it is aimed at teaching in schools. 

The main differences between the original and the version described here are:
* The legs contain an additional segment, hence 3DOF
* Based on an ESP32 microcontroller ([TinyPICO](https://www.tinypico.com/)) instead of the original's Arduino Nano. 
* Programmed in [MicroPython](http://micropython.org/)
* Supports current sensing (load) for up to 8 servos, e.g. to detect if the robot is stuck
* Accepts an additional controller board for autonomous behaviour
    
Please find the details, see [Wiki](https://github.com/teuler/hexpodling/wiki).

### Release Notes

* 2020-06-27 - Initial release
* 2020-12-27 - Updates - Still work in progress
  * Wiki pages 'Mechanics' and 'Boards' added
  * Code updated (now also some initial code for the client microcontroller)
  * Both boards - for the "walk engine" and for the "behavior engine" - revised (including bug fixes)
