# ----------------------------------------------------------------------------
# walk_engine.py
# Definition of the class `WalkEngine`, which subsumises all functions of the
# hexapod board.
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, v1
# ----------------------------------------------------------------------------
import array
from micropython import const
from carrier_version_server import BOARD_VER
import robotling_lib.robotling_board as rb
from robotling_lib.motors.servo_manager import ServoManager
from robotling_lib.misc.helpers import timed_function, TimeTracker

from hexa_global import *
from hexa_config_vorpal import HexaConfig
from server.hexa_gait_generator import HexaGaitGenerator
from hexapod import Hexapod

from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  from tinypico import get_battery_voltage
  import robotling_lib.platform.esp32.board_tinypico as board
  import robotling_lib.platform.esp32.busio as busio
  import robotling_lib.platform.esp32.dio as dio
  import robotling_lib.platform.esp32.aio as aio
  from robotling_lib.driver.dotstar import DotStar
  from machine import Pin
  import time
else:
  print("ERROR: No matching hardware libraries in `platform`.")

__version__      = "0.1.0.0"

# ----------------------------------------------------------------------------
class WalkEngine(object):
  """Walk engine's main class.

  Objects:
  -------
  - greenLED       : on(), off()
  - power5V        : on(), off()
  - Buzzer         : freq_Hz(), beep(freq=440)
  - Potentiometer  : value
  - SM             : Servo manager

  Methods:
  -------
  - setPosture(post=POST_RELAXED, force_linear=False)
    Assume a predefined posture (see POST_xxx)
  - moveServos(sbits=0xFFF, pos=[], dt=1000, linear=True, wait=False)
    Move servos to standard or to user-defined position(s)
  - runServoCalibration()
    Start the servo manager's interactive servo calibration routine

  - powerDown()
    Switch off servos, close connections, etc.

  - update()
    Update onboard devices (Neopixel, analog sensors, etc.). Call frequently
    to keep sensors updated and NeoPixel pulsing!
  - spin_ms(dur_ms=0, period_ms=-1, callback=None)
    Instead of using a timer that calls `update()` at a fixed frequency (e.g.
    at 20 Hz), one can regularly, calling `spin()` once per main loop and
    everywhere else instead of `time.sleep_ms()`. For details, see there.
  - spin_while_moving(t_spin_ms=50)
    Call spin frequently while waiting for the current move to finish

  Properties:
  ----------
  - dialPosition   : Return position of potentiometer as `DialState.xxx` value
  - servoPower     : Switch servo power on or off
  - dotStarPower   : Turns power to DotStar LED on TinyPICO on or off
  - servoBattery_V : Servo battery in [V]
  - logicBattery_V : Logics battery in [V]
  - version_as_int : Software version
  - memory         : Returns allocated and free memory as tuple
  """
  MIN_UPDATE_PERIOD_MS = const(20)  # Minimal time between update() calls
  APPROX_UPDATE_DUR_MS = const(8)   # Approx. duration of the update/callback
  HEARTBEAT_STEP_SIZE  = const(5)   # Step size for pulsing NeoPixel

  COX_NEUTRAL          = const(0)   # Special leg and hip angles ...
  FEM_NEUTRAL          = const(0)
  FEM_SIT              = const(-55) #-40
  TIB_NEUTRAL          = const(0)
  TIB_SIT              = const(0)   # 35
  TIB_RELAXED          = const(-90)

  POST_NEUTRAL         = const(0)   # Postures
  POST_RELAXED         = const(1)
  POST_SITTING         = const(2)
  POST_WALK            = const(3)

  def __init__(self):
    """ Initialize onboard components
    """
    print("WalkEngine (board v{0:.2f}, software v{1}) w/ MicroPython {2} ({3})"
          .format(BOARD_VER/100, __version__, platform.sysInfo[2],
                  platform.sysInfo[0]))
    print("Initializing ...")

    # Get the current time in seconds
    self._run_duration_s = 0
    self._start_s = time.time()

    # Get the robot's configuration
    self._isServoPowerOn = False
    self.Cfg = HexaConfig()
    verbose = self.Cfg.VERBOSE
    nSrv = self.Cfg.SERVO_COUNT

    # Initialize some variables
    self._devices = self.Cfg.DEVICES
    self._uart = None
    self._ble = None
    self._bsp = None
    self.ID = platform.GUID
    print("[{0:>12}] {1:35}".format("GUID", self.ID))

    # Initialize on-board hardware
    self.greenLED = dio.DigitalOut(rb.GREEN_LED)
    self.Buzzer = dio.Buzzer(rb.BUZZER)
    self.Potentiometer = aio.AnalogIn(rb.ADC_POT)
    self._adc_battery = aio.AnalogIn(rb.ADC_BAT)

    self._stateDS = True
    self.dotStarPower = self._stateDS
    self._SPI_DS = busio.SPIBus(rb.SPI_FRQ, rb.DS_CLOCK, rb.DS_DATA, rb.MISO)
    self._DS = DotStar(0,0, 1, brightness=0.5, spi=self._SPI_DS)
    self._DS[0] = 0
    self._iColor = 0
    self._DS_RGB = bytearray([0]*3)
    self._DS_curr = array.array("i", [0,0,0])
    self._DS_step = array.array("i", [0,0,0])
    self._DS_pulse = False

    # Create servos
    if "minimaestro18" in self.Cfg.DEVICES:
      # Initialize servo driver
      from robotling_lib.motors.servos_mini_maestro_18 import MiniMaestro18
      self.ServoCtrl = MiniMaestro18(ch=rb.UART2_CH, _tx=rb.TX2, _rx=rb.RX2)
    else:
      raise NotImplementedError("No compatible servo controller defined")

    # Create servo manager and servos ...
    self.SM = ServoManager(nSrv, verbose=False)
    self._Servos = []
    ranges_us = self.Cfg.get_servo_ranges_us()
    ranges_dg = self.Cfg.SERVO_RANGES
    directions = self.Cfg.SERVO_DIR
    for i in range(nSrv):
      self._Servos.append(self.ServoCtrl.channels[i])
      self._Servos[i].change_range(ranges_us[i], ranges_dg[i], directions[i])
      self._Servos[i].change_behavior(127, 0)
      self.SM.add_servo(i, self._Servos[i])
    self._SPos = array.array('i', [0] *nSrv)
    self._SIDs = array.array('b', [i for i in range(nSrv)])
    toLog("Servo manager ready")

    # Battery status
    v = self.servoBattery_V
    errC = ErrCode.Ok if v > BATT_SERVO_THRES_V else ErrCode.LowBattery
    toLog("Servo battery = {0:.2f} V".format(v), err=errC, green=True)
    v = self.logicBattery_V
    errC = ErrCode.Ok if v > BATT_LOGIC_THRES_V else ErrCode.LowBattery
    toLog("Logic battery = {0:.2f} V".format(v), err=errC, green=True)

    if "wlan" in self._devices:
      # Connect to WLAN, if not already connected
      self.connectToWLAN()

    if "uart_client" in self._devices:
      # Create an UART for a serial connection to the client
      self._uart = busio.UART(rb.UART_CH, baudrate=rb.BAUD, tx=rb.TX, rx=rb.RX)
      s = "#{0}, tx/rx={1}/{2}, {3} Bd".format(rb.UART_CH, rb.TX,rb.RX, rb.BAUD)
      toLog(s, sTopic="uart")
    if "ble_client" in self._devices:
      # Activate bluetooth and create a simple BLE UART for remote control
      from bluetooth import BLE
      from robotling_lib.remote.ble_peripheral import BLESimplePeripheral
      self._ble = BLE()
      self._bsp = BLESimplePeripheral(self._ble, BLE_UART_DEVICE_NAME)

    # Initialize spin function-related variables
    self._spin_period_ms = 0
    self._spin_t_last_ms = 0
    self._spin_callback = None
    self._spinTracker = TimeTracker()

    # Create gait generator instance for kinematics
    self.GGN = HexaGaitGenerator(self.Cfg, self.greenLED, self.SM, verbose)

    # Move servos in "relaxed" position
    self.setPosture(POST_SITTING, force_linear=True)
    self._isServoPowerOn = True

    # First update of robot state representation
    self.HPR = Hexapod()
    self.updateRepresentation(full=True)

    # Done
    self.Buzzer.beep()
    print("... done.")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def version_as_int(self):
    return int(__version__.replace(".", ""))

  @property
  def memory(self):
    import gc
    gc.collect()
    return (gc.mem_alloc(), gc.mem_free())

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def updateRepresentation(self, full=False):
    """ Update the robot's representation object
    """
    self.HPR.dialState = self.dialPosition
    self.HPR.servoPower = self._isServoPowerOn
    self.HPR.servoBattery_mV = int(self.servoBattery_V *1000)
    self.HPR.logicBattery_mV = int(self.logicBattery_V *1000)
    if full:
      self.HPR.softwareVer = self.version_as_int
      self.HPR.memory_kB = self.memory

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
    self.setPosture(post=POST_SITTING)
    self.dotStarPower = False
    self.servoPower = False
    if self._uart:
      self._uart.deinit()
    if self._ble:
      self._bsp.deinit()
      self._ble.active(False)
    self.Buzzer.warn()
    self.Buzzer.beep()
    self._run_duration_s = time.time() -self._start_s

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def dotStarPower(self):
    return self._stateDS

  @dotStarPower.setter
  def dotStarPower(self, state):
    if state:
      Pin(rb.DS_POWER, Pin.OUT, None)
      Pin(rb.DS_POWER).value(False)
    else:
      Pin(rb.DS_POWER, Pin.IN, Pin.PULL_HOLD)
    Pin(rb.DS_CLOCK, Pin.OUT if state else Pin.IN)
    Pin(rb.DS_DATA, Pin.OUT if state else Pin.IN)
    time.sleep(.035)
    self._stateDS = state

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
    self._spinTracker.reset()
    # ...
    self._pulseDotStar()
    if self._spin_callback:
      self._spin_callback()
    self._spinTracker.update()

  def spin_ms(self, dur_ms=0, period_ms=-1, callback=None):
    """ If not using a Timer to call `update()` regularly, calling `spin()`
        once per main loop and everywhere else instead of `time.sleep_ms()`
        is an alternative to keep the robotling board updated.
        e.g. "spin(period_ms=50, callback=myfunction)"" is setting it up,
             "spin(100)"" (~sleep for 100 ms) or "spin()" keeps it running.
    """
    if self._spin_period_ms > 0:
      p_ms = self._spin_period_ms
      p_us = p_ms *1000
      d_us = dur_ms *1000

      if dur_ms > 0 and dur_ms < (p_ms -APPROX_UPDATE_DUR_MS):
        time.sleep_ms(int(dur_ms))

      elif dur_ms >= (p_ms -APPROX_UPDATE_DUR_MS):
        # Sleep for given time while updating the board regularily; start by
        # sleeping for the remainder of the time to the next update ...
        t_us  = time.ticks_us()
        dt_ms = time.ticks_diff(time.ticks_ms(), self._spin_t_last_ms)
        if dt_ms > 0 and dt_ms < p_ms:
          time.sleep_ms(dt_ms)

        # Update
        self.update()
        self._spin_t_last_ms = time.ticks_ms()

        # Check if sleep time is left ...
        d_us = d_us -int(time.ticks_diff(time.ticks_us(), t_us))
        if d_us <= 0:
          return

        # ... and if so, pass the remaining time by updating at regular
        # intervals
        while time.ticks_diff(time.ticks_us(), t_us) < (d_us -p_us):
          time.sleep_us(p_us)
          self.update()

        # Remember time of last update
        self._spin_t_last_ms = time.ticks_ms()

      else:
        # No sleep duration given, thus just check if time is up and if so,
        # call update and remember time
        d_ms = time.ticks_diff(time.ticks_ms(), self._spin_t_last_ms)
        if d_ms > self._spin_period_ms:
          self.update()
          self._spin_t_last_ms = time.ticks_ms()

    elif period_ms > 0:
      # Set up spin parameters and return
      self._spin_period_ms = period_ms
      self._spin_callback = callback
      self._spinTracker.reset(period_ms)
      self._spin_t_last_ms = time.ticks_ms()

    else:
      # Spin parameters not setup, therefore just sleep
      time.sleep_ms(dur_ms)

  def spin_while_moving(self, t_spin_ms=50):
    """ Call spin frequently while waiting for the current move to finish
    """
    while self.SM.is_moving:
      self.spin_ms(t_spin_ms)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def startPulseDotStar(self, iColor):
    """ Set color of DotStar and enable pulsing; return previous color index
    """
    rgb = self._DS.getColorFromWheel(iColor)
    iColPrev = self._iColor
    self._iColor = iColor

    if (rgb != self._DS_RGB) or not(self._DS_pulse):
      # New color and start pulsing
      c = self._DS_curr
      s = self._DS_step
      c[0] = rgb[0]
      s[0] = int(rgb[0] /self.HEARTBEAT_STEP_SIZE)
      c[1] = rgb[1]
      s[1] = int(rgb[1] /self.HEARTBEAT_STEP_SIZE)
      c[2] = rgb[2]
      s[2] = int(rgb[2] /self.HEARTBEAT_STEP_SIZE)
      self._DS_RGB = rgb
      self._DS[0] = rgb
      self._DS.show()
      self._DS_pulse = True
      self._DS_fact = 1.0
    return iColPrev

  def _pulseDotStar(self):
    """ Update pulsing, if enabled
    """
    if self._DS_pulse:
      rgb = self._DS_RGB
      for i in range(3):
        self._DS_curr[i] += self._DS_step[i]
        if self._DS_curr[i] > (rgb[i] -self._DS_step[i]):
          self._DS_step[i] *= -1
        if self._DS_curr[i] < abs(self._DS_step[i]):
          self._DS_step[i] = abs(self._DS_step[i])
        if self._DS_fact < 1.0:
          self._DS_curr[i] = int(self._DS_curr[i] *self._DS_fact)
      self._DS[0] = self._DS_curr
      self._DS.show()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def connectToWLAN(self):
    """ Connect to WLAN if not already connected
    """
    if platform.ID in [platform.ENV_ESP32_UPY, platform.ENV_ESP32_TINYPICO]:
      import network
      from NETWORK import my_ssid, my_wp2_pwd
      if not network.WLAN(network.STA_IF).isconnected():
        sta_if = network.WLAN(network.STA_IF)
        if not sta_if.isconnected():
          print('Connecting to network...')
          sta_if.active(True)
          sta_if.connect(my_ssid, my_wp2_pwd)
          while not sta_if.isconnected():
            self.greenLED.on()
            time.sleep(0.05)
            self.greenLED.off()
            time.sleep(0.05)
          print("[{0:>12}] {1}".format("network", sta_if.ifconfig()))

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def printReport(self):
    """ Prints a report on memory usage and performance
    """
    used, free = self.memory
    total = free +used
    print("Memory     : {0:.0f}% of {1:.0f}kB heap RAM used."
          .format(used/total*100, total/1024))
    print("Batteries  : servo: {0:.2f}V, logic {0:.2f}V"
          .format(self.servoBattery_V, self.logicBattery_V))
    avg_ms = self._spinTracker.meanDuration_ms
    dur_ms = self._spinTracker.period_ms
    print("Performance: spin: {0:6.3f}ms @ {1:.1f}Hz ~{2:.0f}%"
          .format(avg_ms, 1000/dur_ms, avg_ms /dur_ms *100))
    print("---")

# ----------------------------------------------------------------------------
