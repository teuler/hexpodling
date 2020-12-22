# ----------------------------------------------------------------------------
# hexapod.py
# Representation of the robot
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-08-14, First version
#
# ----------------------------------------------------------------------------
try:
  ModuleNotFoundError
except NameError:
  ModuleNotFoundError = ImportError
try:
  # Micropython imports
  from micropython import const
  from hexa_global import *
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  from hexbotling.hexa_global import *

import robotling_lib.misc.rmsg as rmsg

# ----------------------------------------------------------------------------
class Hexapod(object):
  """Hexapod representation class"""

  SRV = const(0)
  CLI = const(1)

  def __init__(self):
    """ Initialize representation
    """
    self.dialState = DialState.NONE
    self.dialStateChanged = True

    self.servoPower = False
    self.servoBattery_mV = 0
    self.logicBattery_mV = [0, 0]
    self.serverLogicBattery_mV = 0

    self.softwareVer = [0, 0]
    self.memory_kB = [0, 0]

    self.hexState = HexState.Undefined
    # ...

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def asDict(self):
    """ Return object as dictionary
    """
    d = dict()
    d["State"] = {"dialState": self.dialState,
                  "hexState": self.hexState}
    d["Power"] = {"servoPower": self.servoPower,
                  "servoBattery_mV": self.servoBattery_mV,
                  "logicBattery_mV": self.logicBattery_mV}
    d["softwareVer"] = self.softwareVer
    d["memory_kB"] = self.memory_kB
    return d

  def fromMsg(self, msg):
    """ Populate object from STA message
    """
    if msg.token == rmsg.TOK_STA:
      d = msg._data[0]
      self.hexState = d[0]
      self.dialState = d[2]
      self.servoPower = d[3]
      self.servoBattery_mV = d[4]
      self.logicBattery_mV[SRV] = d[5]

# ----------------------------------------------------------------------------
