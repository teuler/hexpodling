This folder contains the current MMBasic code:

- `hps_calibrate.bas` - A tool to calibrate the servo positions of the legs
- `hps_cli.bas` - The main program for the Pico that serves as client (it controls the robot) 
- `hps_srv.bas` - The main program for the Pico that runs the server; it controls the movements of the robot (lets it walk), collects some sensor data about the robots orientation, the voltages of servo and logic batteries, etc.
- `lib_bno055.bas` - "Driver" for the BNO055 9-axis absolute-orientation sensor with sensor fusion.
- `lib_mcp3208.bas` - "Driver" for the MCP3208 chip, an 8-channel 12-bit A/D converter with SPI interface
- `lib_esp32cam.bas` - "Driver" for the ESP32-cam / VL53L5CX time-of-flight sensor
- `lib_keyboard.bas` - Keyboard routines for `hps_cli.bas`
- `lib_maestro.bas` - Routines for servo control via the Pololu 18-channel Maestro board
- `cal_load.dat` - calibration data for the first 4 servo load channels 

## Server
To upload the software to the server Pico:
1. Install the [PicoMite firmware](https://geoffg.net/Downloads/picomite/PicoMite_Firmware.zip) (V6.00.01, `PicoMiteRP2040V6.00.01.uf2` or the RP2350 variant)
2. Load programs onto the internal drive:
   ```
   xmodem r "hps_srv.bas"
   xmodem r "lib_bno055.bas"
   xmodem r "lib_mcp3208.bas"
   xmodem r "lib_maestro.bas"
   xmodem r "hps_calibrate.bas"
   xmodem r "cal_load.dat"
   ```
3. Install libraries:
   ```
   library delete
   load "lib_bno055.bas"
   library save
   load "lib_mcp3208.bas"
   library save
   load "lib_maestro.bas"
   library save
   ```
4. Set the following options:
   ```
   OPTION COLOURCODE ON
   OPTION DISPLAY 64, 80   
   OPTION AUTORUN ON
   OPTION HEARTBEAT OFF
   OPTION CPUSPEED 252000
   ```
5. Now load the main program `hps_srv.bas` and run it.
     
## Client
To upload the software to the client Pico:
1. Install the [PicoMite firmware](https://geoffg.net/Downloads/picomite/PicoMite_Firmware.zip) (V6.00.01, `PicoMiteRP2040USBV6.00.01.uf2` or the RP2350 variant). The USB variant is needed to allow the robot to be controlled by a USB-gamepad.

> Note: _After installing the USB firmware, the Pico console cannot be longer connected via the USB port, because it enters host-mode. For connecting the serial console, use pins GP8 and GP9 (COM2, 115200). For details, see MMBasic manual._

2. Load programs onto the internal drive:
   ```
   xmodem r "hps_cli.bas"
   xmodem r "lib_keyboard.bas"
   xmodem r "lib_esp32cam.bas"
   ```
3. Install libraries:
   ```
   library delete
   load "lib_keyboard.bas"
   library save
   ```
4. Set the following options (note that the SPI and LCDPANEL options are only needed if connecting a TFT display):
   ```
   OPTION SERIAL CONSOLE COM2,GP8,GP9
   OPTION LCD SPI GP18,GP19,GP16
   OPTION COLOURCODE ON
   OPTION HEARTBEAT OFF
   OPTION CPUSPEED 252000
   OPTION DISPLAY 64, 80
   OPTION LCDPANEL ST7789_320BUFF, PORTRAIT,GP22,GP15,GP17,GP14
   OPTION AUTORUN ON
   OPTION PICO OFF
   ```
   Calling `option list` should also give the follwing options automatically:
   ```
   OPTION SERIAL CONSOLE COM2,GP8,GP9
   OPTION KEYBOARD US
   ```
   
5. Now load the main program `hps_cli.bas` and run it.
