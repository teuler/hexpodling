This folder contains python code that runs on a PC, e.g. to communicate with a hexbotling:

- `hexapod_remote_ble.py` - Allows remote controlling the robot using a standard USB/wireless game controller. It translates the game controller actions into 
   walk/movement commands and sends them via a Bluetooth (BLE) link to the TinyPICO ESP32 on the robot. This program requires the Python packages 
   [`numpy`](https://numpy.org/), [`pygame`](https://www.pygame.org), and [`bleak`](https://github.com/hbldh/bleak).
   
The subfolder `hexbotling` contains the code that runs on the roboter.

The folder `modules` contains additional libraries (e.g. `joystick.py`)

> Note: _The robot code relies on the drivers in [`robotling_lib`](https://github.com/teuler/robotling_lib), which has to be clones into the `hexbotling` 
  folder on the development PC._

