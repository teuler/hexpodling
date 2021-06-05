# ----------------------------------------------------------------------------
# walk_engine.py
# Definition of the class `WalkEngine`, which subsumises all functions of the
# hexapod server board
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-01-06, v1
# 2021-01-08, v1.1, - New revision of server board (v0.3)
#                   - I2C compass added (BNO055)
#                   - Servo load processing added
# ----------------------------------------------------------------------------
import array
from micropython import const
from robotling_board_version import BOARD_VER
import robotling_lib.robotling_board as rb
import robotling_lib.misc.ansi_color as ansi
from robotling_lib.motors.servo_manager import ServoManager
from robotling_lib.robotling_base import RobotlingBase
from robotling_lib.misc.helpers import timed_function

from hexa_global import *
from hexa_config_vorpal import HexaConfig
from server.hexa_gait_generator import HexaGaitGenerator
from hexapod import Hexapod

from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  from tinypico import get_battery_voltage
  import robotling_lib.platform.esp32.board_tinypico as board
  from robotling_lib.platform.esp32.busio import UART, I2CBus
  import robotling_lib.platform.esp32.aio as aio
  import robotling_lib.platform.esp32.dio as dio
  import time
  I2C_SCAN = True
else:
  print("ERROR: No matching hardware libraries in `platform`.")

__version__      = "0.1.1.0"

# ----------------------------------------------------------------------------
class WalkEngine(RobotlingBase):
  """Walk engine's main class.

  Objects:
  -------
  - yellowLED      : on(), off()
  - power5V        : on(), off()
  - Buzzer         : freq_Hz(), beep(freq=440)
  - Potentiometer  : value
  - SM             : Servo manager
  - Compass        : Compass (BNO055)

  Methods:
  -------
  - setPosture(post=POST_RELAXED, force_linear=False)
    Assume a predefined posture (see POST_xxx)
  - moveServos(sbits=0xFFF, pos=[], dt=1000, linear=True, wait=False)
    Move servos to standard or to user-defined position(s)

  - powerDown()
    Switch off servos, close connections, etc.

  - update()
    Update onboard devices (Neopixel, analog sensors, etc.). Call frequently
    to keep sensors updated and NeoPixel pulsing!

  Properties:
  ----------
  - dialPosition   : Return position of potentiometer as `DialState.xxx` value
  - servoPower     : Switch servo power on or off
  - servoBattery_V : Servo battery in [V]
  - logicBattery_V : Logics battery in [V]
  - version_as_int : Software version
  """
  COX_NEUTRAL          = const(0)    # Special leg and hip angles ...
  FEM_NEUTRAL          = const(0)
  FEM_SIT              = const(-55)  #-40
  TIB_NEUTRAL          = const(0)
  TIB_SIT              = const(0)    # 35
  TIB_RELAXED          = const(-90)

  POST_NEUTRAL         = const(0)    # Postures
  POST_RELAXED         = const(1)
  POST_SITTING         = const(2)
  POST_WALK            = const(3)

  def __init__(self):
    """ Initialize onboard components
    """
    print("WalkEngine (board v{0:.2f}, software v{1}) w/ MicroPython {2} ({3})"
          .format(BOARD_VER/100, __version__, platform.sysInfo[2],
                  platform.sysInfo[0]))
    toLog("Initializing ...", head=False)
    self.yellowLED = dio.DigitalOut(rb.YELLOW_LED, True)
    super().__init__(MCP3208=True)
    toLog(self.ID, sTopic="GUID")
    #print("[{0:>12}] {1:35}".format("GUID", self.ID))

    # Get the robot's configuration
    self._isServoPowerOn = False
    self.Cfg = HexaConfig()
    verbose = self.Cfg.VERBOSE
    nSrv = self.Cfg.SERVO_COUNT
    self.loadLims = self.Cfg.get_servo_load_ranges()

    # Initialize some variables
    self._devices = self.Cfg.DEVICES
    self.Compass = None
    self._uart = None
    self._ble = None
    self._bsp = None

    # Initialize remaining on-board hardware
    self.Buzzer = dio.Buzzer(rb.BUZZER)
    self.Buzzer.mute = self.Cfg.NO_BUZZER
    self.Potentiometer = aio.AnalogIn(rb.ADC_POT)
    self._adc_battery = aio.AnalogIn(rb.ADC_BAT)
    self._MCP3208.channelMask = self.Cfg.SERVO_LOAD_MASK

    # Get I2C bus
    self._I2C = I2CBus(freq=rb.I2C_FRQ, scl=rb.SCL, sda=rb.SDA, scan=I2C_SCAN)
    self._nI2CDev = len(self._I2C.deviceAddrList) if I2C_SCAN else -1

    # Initialize other devices
    if "bno055" in self.Cfg.DEVICES and self._nI2CDev != 0:
      # Connect to bno055
      from robotling_lib.sensors.compass_bno055 import Compass
      self.Compass = Compass(self._I2C)

    # Create servos
    toLog("Initializing servo and power management ...", head=False)
    if "minMaestro18" in self.Cfg.DEVICES:
      # Initialize servo driver
      from robotling_lib.motors.servos_mini_maestro_18 import MiniMaestro18
      self.ServoCtrl = MiniMaestro18(ch=rb.UART2_CH, _tx=rb.TX2, _rx=rb.RX2)
    else:
      raise NotImplementedError("No compatible servo controller defined")

    # Create servo manager and servos ...
    self.SM = ServoManager(nSrv, verbose=self.Cfg.VERBOSE > 1)
    self._Servos = []
    ranges_us = self.Cfg.get_servo_ranges_us()
    ranges_dg = self.Cfg.SERVO_RANGES
    directions = self.Cfg.SERVO_DIR
    for i in range(nSrv):
      self._Servos.append(self.ServoCtrl.channels[i])
      self._Servos[i].change_range(ranges_us[i], ranges_dg[i], directions[i])
      self._Servos[i].change_behavior(127, 0)
      self.SM.add_servo(i, self._Servos[i])
    _ = self.ServoCtrl.channels[0].get_error()
    self._SPos = array.array('i', [0] *nSrv)
    self._SIDs = array.array('b', [i for i in range(nSrv)])
    toLog("Servo manager ready", green=True)

    # Battery status
    v = self.servoBattery_V
    errC = ErrCode.Ok if v > BATT_SERVO_THRES_V else ErrCode.LowBattery
    toLog("Servo battery = {0:.2f} V".format(v), err=errC, green=True)
    v = self.logicBattery_V
    errC = ErrCode.Ok if v > BATT_LOGIC_THRES_V else ErrCode.LowBattery
    toLog("Logic battery = {0:.2f} V".format(v), err=errC, green=True)

    # Serial connection to client
    toLog("Initializing client interface(s) ...", head=False)
    if "uart_client" in self._devices:
      # Create an UART for a serial connection to the client
      self._uart = UART(rb.UART_CH, baudrate=rb.BAUD, tx=rb.TX, rx=rb.RX)
      self._uart.init(bits=8, parity=None, stop=1)
      s = "#{0}, tx/rx={1}/{2}, {3} Bd".format(rb.UART_CH, rb.TX,rb.RX, rb.BAUD)
      toLog(s, sTopic="uart(server)")

    if "ble_client" in self._devices:
      # Activate bluetooth and create a simple BLE UART for remote control
      from bluetooth import BLE
      from robotling_lib.remote.ble_peripheral import BLESimplePeripheral
      self._ble = BLE()
      self._bsp = BLESimplePeripheral(self._ble, BLE_UART_DEVICE_NAME)

    # Create gait generator instance for kinematics
    toLog("Initializing walk engine ...", head=False)
    self.GGN = HexaGaitGenerator(self.Cfg, self.yellowLED, self.SM, verbose)

    # Move servos in "relaxed" position
    self.setPosture(POST_SITTING, force_linear=True)
    self._isServoPowerOn = True

    # First update of robot state representation
    self.HPR = Hexapod()
    self.updateRepresentation(force_all=True)

    # Done
    self.Buzzer.beep()
    self.yellowLED.off()
    toLog("... done.", head=False)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def version_as_int(self):
    return int(__version__.replace(".", ""))

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def updateRepresentation(self, force_all=False):
    """ Update the robot's representation object. If `force_all` is True,
        all variables are updated, including the ones that are frequently
        updated outside this call.
    """
    t = time.ticks_ms()

    dt = time.ticks_diff(t, self.HPR._lastUpdPower)
    if force_all or dt >= self.Cfg.HRP_POWER_MS:
      # Parameters (e.g. battery) that do not need to be updated so frequently
      self.HPR.servoPower = self._isServoPowerOn
      self.HPR.servoBattery_mV = int(self.servoBattery_V *1000)
      self.HPR.logicBattery_mV = int(self.logicBattery_V *1000)
      self.HPR._lastUpdPower = t

    if force_all:
      # Parameters that are usually updated outside (e.g. in `update()`)
      self.HPR.dialState = self.dialPosition
      self.HPR.dialStateChanged = True
      if self.Compass:
        self.HPR.headPitchRoll_deg = self.Compass.get_heading_3d()[1:]
      # ...

      # Parameters (e.g. software version) that need to be updated rarely
      self.HPR.softwareVer[self.HPR.SRV] = self.version_as_int
      self.HPR.memory_kB[self.HPR.SRV] = self.memory

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def setPosture(self, post=POST_RELAXED, force_linear=False):
    """ Move servos to standard or to user-defined position(s)
    """
    n3 = int(self.Cfg.SERVO_COUNT/3)
    seq = []
    if post == POST_RELAXED:
      seq.append((self._SIDs, [COX_NEUTRAL]*n3 +[FEM_NEUTRAL]*n3 +
                              [TIB_RELAXED]*n3, 1000, 0))
    elif post == POST_NEUTRAL:
      seq.append((self._SIDs, [COX_NEUTRAL]*n3 +[FEM_NEUTRAL]*n3 +
                              [TIB_NEUTRAL]*n3, 1000, 0))
    elif post == POST_SITTING:
      seq.append((self._SIDs, [COX_NEUTRAL]*n3 +[FEM_SIT]*n3 +
                              [TIB_SIT]*n3, 1500, 1))
    elif post == POST_WALK:
      seq.append((self._SIDs, [COX_NEUTRAL]*n3 +[self.Cfg.FEM_STRT_ANG]*n3 +
                              [self.Cfg.TIB_STRT_ANG]*n3, 1500, 1))
    for entr in seq:
      # Move into position of sequence and wait, either via spin, if already
      # setup, or simply by sleeping ...
      spos = array.array('i', entr[1])
      self.SM.move(entr[0], spos, entr[2], force_linear or entr[3])
      if self._spin_period_ms > 0:
        self.spin_while_moving()
      else:
        time.sleep_ms(entr[2] +100)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def powerDown(self):
    """ Switch off servos, close connections, etc.
    """
    #self.setPosture(post=POST_SITTING)
    self.servoPower = False
    self.SM.deinit()
    self.ServoCtrl.deinit()
    if self._uart:
      self._uart.deinit()
    if self._ble:
      self._bsp.deinit()
      self._ble.active(False)
    self.Buzzer.warn()
    self.Buzzer.beep()
    super().powerDown()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def servoPower(self):
    return self._isServoPowerOn

  @servoPower.setter
  def servoPower(self, state):
    self._isServoPowerOn = state
    if not state:
      self.SM.turn_all_off()

  @property
  def dialPosition(self):
    d = self.Potentiometer.value
    if d < 20:
      return DialState.STOP
    elif d < 300:
      return DialState.AD
    elif d < 1000:
      return DialState.TS
    elif d < 2300:
      return DialState.DEMO
    elif d > 4050:
      return DialState.RC
    else:
      return DialState.NONE

  @property
  def servoBattery_V(self):
    """ Servo battery voltage in [V]
        = value/4096 *3.3 *(22+10)/10
        w/ correction factor accounting for "real" resistors
    """
    return self._adc_battery.value *0.002773438 *1.066

  @property
  def logicBattery_V(self):
    """ Logic battery voltage in [V]
    """
    return get_battery_voltage()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def update(self):
    """ Update onboard devices ...
    """
    super().updateStart()
    self._pulsePixel()

    # Read compass data
    if self.Compass:
      self.HPR.headPitchRoll_deg = self.Compass.get_heading_3d()[1:]

    # Process servo load and get foot position
    # (_MCP3208.update() is already called in RobotlingBase.updateStart)
    for i in range(self.Cfg.SERVO_LOAD_CHANS):
      self.HPR.servoLoad[i] = int((self._MCP3208.data[i] -self.loadLims[i][0])
                                  /self.loadLims[i][2] *100)
    self.HPR.legYPos[0:] = array.array("h", self.GGN._xyzCurrFootPos.transpose()[1])
    super().updateEnd()

  def spin_while_moving(self, t_spin_ms=50):
    """ Call spin frequently while waiting for the current move to finish
    """
    while self.SM.is_moving:
      self.spin_ms(t_spin_ms)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def printReport(self):
    """ Prints a report on memory usage and performance
    """
    super().printReport()
    print("Batteries  : servo: {0:.2f}V, logic {1:.2f}V"
          .format(self.servoBattery_V, self.logicBattery_V))
    print("---")

# ----------------------------------------------------------------------------
