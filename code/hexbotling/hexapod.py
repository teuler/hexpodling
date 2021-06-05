# ----------------------------------------------------------------------------
# hexapod.py
# Representation of the robot
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-08-14, First version
# 2021-01-15, Servo load, foot position and compass data added
# ----------------------------------------------------------------------------
try:
  ModuleNotFoundError
except NameError:
  ModuleNotFoundError = ImportError
try:
  # Micropython imports
  from micropython import const
  from hexa_global import *
  from robotling_lib.misc.helpers import timed_function
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  from hexbotling.hexa_global import *

import array
import robotling_lib.misc.rmsg as rmsg
from hexa_config_vorpal import HexaConfig

# ----------------------------------------------------------------------------
class Hexapod(object):
  """Hexapod representation class"""

  SRV = const(0)
  CLI = const(1)

  def __init__(self):
    """ Initialize representation
    """
    self.Cfg = HexaConfig()

    # Software version and free memory
    self.softwareVer = [0, 0]
    self.memory_kB = [0, 0]

    # State of dial and if it has just changed
    self.dialState = DialState.NONE
    self.dialStateChanged = True

    # Voltage of batteries, if servo power is on
    self.servoPowerState = False
    self.servoBattery_mV = 0
    self.logicBattery_mV = 0
    self._lastUpdPower = 0

    # Servo load and foot position
    self.servoLoad = array.array("h", [0]*self.Cfg.SERVO_LOAD_CHANS)
    self.legYPos = array.array("h", [0]*LEG_COUNT)

    # Heading, pitch and roll
    self.headPitchRoll_deg = array.array("f", [0, 0, 0])

    # General status of robot
    self.hexState = HexState.Undefined

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def asDict(self):
    """ Return object as dictionary
    """
    d = dict()
    d["state"] = {"dialState": self.dialState,
                  "hexState": self.hexState}
    d["power"] = {"servoPowerState": self.servoPowerState,
                  "servoBattery_mV": self.servoBattery_mV,
                  "logicBattery_mV": self.logicBattery_mV}
    d["euler"] = {"Heading_deg": self.headPitchRoll_deg[0],
                  "Pitch_deg": self.headPitchRoll_deg[1],
                  "Roll_deg": self.headPitchRoll_deg[2]}
    d["engine"] = {"servoLoad": self.servoLoad,
                   "footYPos": self.legYPos}
    d["softwareVer"] = self.softwareVer
    d["memory_kB"] = self.memory_kB
    return d

  #@timed_function
  def fromMsg(self, msg):
    """ Populate object from STA message
    """
    if msg.token == rmsg.TOK_STA:
      p = rmsg.TOK_addrPSetStart +rmsg.TOK_offsVals
      d = msg._msg
      self.hexState = d[p+0]
      self.dialState = d[p+2]
      self.servoPower = d[p+3]
      self.servoBattery_mV = d[p+4]
      self.logicBattery_mV = d[p+5]
      self.headPitchRoll_deg = array.array("f", d[p+6:p+9])
      self.servoLoad = d[p+9:p+13]
      self.legYPos = d[p+13:p+19]

# ----------------------------------------------------------------------------
