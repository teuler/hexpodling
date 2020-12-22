# hexbotling

"hexbotling" is a work-in-progress, 3-degrees of freedom (3DOF) hexapod robot based on the [Vorpal](https://vorpalrobotics.com/wiki/index.php/Vorpal_The_Hexapod) from Vorpal Robotics. The original Vorpal is remote-controlled and 2DOF; it is aimed at teaching in schools. 

The main differences between the original and the version described here are:
* The legs contain an additional segment, hence 3DOF
* Based on an ESP32 microcontroller ([TinyPICO](https://www.tinypico.com/)) instead of the original's Arduino Nano. 
* Programmed in [MicroPython](http://micropython.org/)
* Supports current sensing (load) for up to 8 servos, e.g. to detect if the robot is stuck
* Accepts an additional controller board for autonomous behaviour

The "hexbotling" is driven by two microcontrollers communicating via a serial connection: 
* A [TinyPICO](https://www.tinypico.com/) ESP32 microcontroller implements the "walk engine", which controls the servos, sequences leg movements and gaits (inverse kinematics), and manages the power. The "walk engine" accepts simple commands via an onboard serial port or via BLE. With the latter, the robot can be remote-controlled. 
* A [Feathers2](https://feathers2.io/) ESP32-S2 microcontroller implements the second controller board ("behaviour engine") for autonomous behaviour. It has connectors for various sensors.
    
Please find the details in the [WiKi](https://github.com/teuler/hexbotling/wiki).

### Release Notes

* 2020-06-27 - Initial release
