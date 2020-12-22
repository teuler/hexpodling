# ----------------------------------------------------------------------------
# behave_engine.py
# Definition of the class `BehaveEngine`, which subsumises all functions of
# the hexapod client board
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-09-04, v1
# ----------------------------------------------------------------------------
import gc
from micropython import const
from robotling_board_version import BOARD_VER
import robotling_lib.robotling_board as rb
from robotling_lib.robotling_base import RobotlingBase

from hexa_global import *
from hexa_config_vorpal import HexaConfig
from hexapod import Hexapod

from robotling_lib.platform.platform import platform
if platform.languageID == platform.LNG_MICROPYTHON:
  import robotling_lib.platform.esp32.board_huzzah32 as board
  from robotling_lib.platform.esp32.busio import UART, I2CBus
  from robotling_lib.platform.esp32.dio import DigitalOut
  from robotling_lib.platform.esp32.aio import AnalogIn
  import time
elif platform.languageID == platform.LNG_CIRCUITPYTHON:
  import board
  from robotling_lib.platform.circuitpython.busio import UART, I2CBus
  from robotling_lib.platform.circuitpython.dio import DigitalOut
  from robotling_lib.platform.circuitpython.aio import AnalogIn
else:
  print("ERROR: No matching hardware libraries in `platform`.")

__version__      = "0.1.0.0"

# ----------------------------------------------------------------------------
class BehaveEngine(RobotlingBase):
  """Behave engine's main class.

  Objects:
  -------
  - onboardLED     : on(), off()
  - yellowLED      : on(), off()

  Methods:
  -------
  - update()
    Update onboard devices (Neopixel, analog sensors, etc.). Call frequently
    to keep sensors updated and NeoPixel pulsing!

  Properties:
  ----------
  - logicBattery_V : battery voltage [V]
  - version_as_int : Software version
  """

  def __init__(self):
    """ Initialize onboard components
    """
    si = platform.sysInfo
    print("BehaveEngine (board v{0:.2f}, software v{1}) w/{2} {3} ({4})"
          .format(BOARD_VER/100, __version__, platform.language, si[2], si[0]))
    print("Initializing ...")

    # Get the robot's configuration
    self.Cfg = HexaConfig()

    # Initialize base object
    super().__init__(neoPixel=False, MCP3208=True)
    print("[{0:>12}] {1:35}".format("GUID", self.ID))

    # Initialize some variables
    self._devices = self.Cfg.DEVICES
    self._uartServ = None
    self._BNO055 = None
    self.Display = None
    self.TRMini = None

    # Initialize on-board (feather) hardware
    self.onboardLED = DigitalOut(rb.BLUE_LED, value=False)
    self.yellowLED = DigitalOut(rb.YELLOW_LED, value=False)
    self._adc_battery = AnalogIn(rb.ADC_BAT) if rb.ADC_BAT else None

    # Get software I2C bus (#0)
    """
    self._I2C = I2CBus(rb.I2C_FRQ, rb.SCL, rb.SDA, code=0)
    self._nI2CDev = len(self._I2C.deviceAddrList)
    """

    # If a framebuffer display is requested, initialize it now before memory
    # is too fragmented for the framebuffer
    if "ssd1327_128x128" in self.Cfg.DEVICES and self._nI2CDev > 0:
      from robotling_lib.driver.ssd1327 import SSD1327_I2C
      self.Display = SSD1327_I2C(128, 128, self._I2C)
      self.Display.fill(0)
      self.Display.set_text_color(15)
      self.Display.println("Hexapod BE v{0}".format(__version__[:3]))
      self.Display.println("Initializing ...")

    # Initialize other devices
    gc.collect()
    if "bno055" in self.Cfg.DEVICES and self._nI2CDev > 0:
      # Connect to bno055
      from robotling_lib.driver.bno055 import BNO055
      self._BNO055 = BNO055(self._I2C)

    if "tera_evomini" in self.Cfg.DEVICES:
      # Connect to TeraRanger Evo Mini via UART
      from robotling_lib.sensors.teraranger_evomini import TeraRangerEvoMini
      self.TRMini = TeraRangerEvoMini(id=rb.UART2_CH, tx=rb.TX2, rx=rb.RX2)

    if "wlan" in self._devices:
      # Connect to WLAN, if not already connected
      self.connectToWLAN()

    # Battery status
    if self._adc_battery:
      v = self.logicBattery_V
      errC = ErrCode.Ok if v > BATT_LOGIC_THRES_V else ErrCode.LowBattery
      toLog("Logic battery = {0:.2f} V".format(v), err=errC, green=True)

    # Serial connection to server
    if "uart_client" in self._devices:
      # Create an UART for a serial connection to the server
      self._uartServ = UART(rb.UART_CH, baudrate=rb.BAUD, tx=rb.TX, rx=rb.RX)
      assert self._uartServ, "Initializing server UART failed"
      s = "#{0}, tx={1} rx={2}, {3} Bd".format(rb.UART_CH, rb.TX,rb.RX, rb.BAUD)
      toLog(s, sTopic="uart(client)", green=True)

    # First update of robot state representation
    self.HPR = Hexapod()
    self.updateRepresentation(full=True)

    # Done
    print("... done.")
    if self.Display:
      self.Display.println("... done.")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def version_as_int(self):
    return int(__version__.replace(".", ""))

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def updateRepresentation(self, full=False):
    """ Update the robot's representation object
    """
    self.HPR.logicBattery_mV[self.HPR.CLI] = int(self.logicBattery_V *1000)
    if full:
      self.HPR.softwareVer[self.HPR.CLI] = self.version_as_int
      self.HPR.memory_kB[self.HPR.CLI] = self.memory

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def powerDown(self):
    """ Close connections, etc.
    """
    '''
    if self._uartServ:
      self._uartServ.deinit()
    if self.TRMini:
      self.TRMini.deinit()
    '''
    if self.Display:
      self.Display.deinit()
    super().powerDown()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def logicBattery_V(self):
    """ Logic battery voltage in [V]
    """
    if self._adc_battery:
      return rb.battery_convert(self._adc_battery.value)
    else:
      return 0

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def update(self):
    """ Update onboard devices ...
    """
    super().updateStart()
    self._pulsePixel()
    # ****************
    # ****************
    # ****************
    super().updateEnd()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def printReport(self):
    """ Prints a report on memory usage and performance
    """
    super().printReport()
    if self._adc_battery:
      print("Battery    : logic {0:.2f}V".format(self.logicBattery_V))
    print("---")

# ----------------------------------------------------------------------------
