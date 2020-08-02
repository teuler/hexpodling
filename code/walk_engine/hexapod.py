# ----------------------------------------------------------------------------
# hexapod.py
# Definition of the class `HexaPod`, derived from `WalkEngine`
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, First version
#
# ----------------------------------------------------------------------------
import ujson
import misc.rmsg as rmsg
from hexa_global import *
from walk_engine import WalkEngine
from misc.helpers import timed_function

from platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  import time
else:
  print("ERROR: No matching hardware libraries in `platform`.")

# ----------------------------------------------------------------------------
class HexaPod(WalkEngine):
  """Hexapod class"""

  def __init__(self):
    super().__init__()

    # Initialize
    self._isVerbose = True
    self._currDialPos = self.DIAL_NONE
    self._dialChanged = True
    self._hexState = HexState.Undefined

    # Telemetry
    self.Tele = None
    '''
    if "wlan" in self._devices:
      from remote.telemetry import Telemetry
      self.greenLED.on()
      self.Tele = Telemetry(self.ID)
      if self.Tele.connect():
        pass
      self.greenLED.off()
    '''
    # Create message objects for serial commands, if valid UART is defined
    if self._uart:
      self.mIn = rmsg.RMsgUART(self._uart)
      self.mOut = rmsg.RMsgUART(self._uart, isClient=False)

    # Done
    self.isRunning = True
    toLog("Hexapod ready.")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def housekeeper(self, info=None):
    """ Does the housekeeping:
        - Changes also color of NeoPixel depending on the robot's state
        - ...
    """
    if self.isRunning:
      # Check if dial position has changed
      d = self.dialPosition
      if d is not self._currDialPos:
        self._currDialPos = d
        self._dialChanged = True

    # ******************************
    # ******************************
    # ******************************
    # CHECK SENSORS AND STUFF
    # ******************************
    # ******************************
    # ******************************

    # Change DotStar according to state
    # TODO ...
    self.startPulseDotStar(100)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  #@timed_function
  def onSerialCommand(self):
    """ Handle new serial command
    """
    try:
      self.greenLED.on()
      self.mOut.clear()
      isDone = False

      if not self.mIn.error == rmsg.Err.Ok:
        toLog("Serial message error", ErrCode.Cmd_ParsingErr)

      elif self.mIn.token == rmsg.TOK_VER:
        # Version information requested
        self.mOut.token = rmsg.TOK_VER
        self.mOut.add_data("V", [self.version_as_int])
        self.mOut.add_data("M", [self.memory[1]])
        self.mOut.send()
        isDone = True

      elif self.mIn.token == rmsg.TOK_XP0:
        # Move all legs to default positions
        self.mOut.token = rmsg.TOK_ACK
        self.mOut.add_data("C", [rmsg.TOK_XP0])
        self.mOut.send()
        self.assumePosture(self.POST_NEUTRAL)
        isDone = True

      elif self.mIn.token == rmsg.TOK_STA:
        # Status request
        # TODO
        print("STA")
        isDone = True

      elif self.mIn.token == rmsg.TOK_GG0:
        # Start or stop the gait generator
        # >GG0 M=a,m G=g;
        # TODO: Use other parameters, such as gait
        if self.mIn[0,0] > 0:
          self.GGN.start()
        else:
          self.GGN.stop()
        isDone = True

      elif self.mIn.token == rmsg.TOK_GGP:
        # Change walk parameters of the gait generator (GGN), positions etc.
        # >GGP B=bs,px,pz,bx,by,bz T=bo,lh,tx,tz,ty;
        data = self.mIn._data
        bo_y = data[1][0]
        bs_y = data[0][0]
        bp_x_z = np.array([data[0][1], 0, data[0][2]])
        br_xyz = np.array([data[0][3], data[0][4], data[0][5]])
        lh = data[1][1]
        tl_x_z = np.array([data[1][2], 0, data[1][3]])
        tr_y = data[1][4]
        self.GGN.Input.set_pos(bo_y, bs_y, bp_x_z, br_xyz, lh, tl_x_z, tr_y)
        isDone = True

      elif self.mIn.token == rmsg.TOK_GGQ:
        # Change only most important walk parameters and request status quickly
        # >GGQ T=bo,lh,tx,tz,ty D=ds A=ta;
        # <STA ...;
        data = self.mIn._data
        bo_y = data[0][0]
        lh = data[0][1]
        tl_x_z = np.array([data[0][2], 0, data[0][3]])
        tr_y = data[0][4]
        ds = data[1][0]
        self.GGN.Input.set_pos_timing(bo_y, lh, tl_x_z, tr_y, ds)
        isDone = True

      else:
        toLog("Command unknown", ErrCode.CmdUnknown)
        w.Buzzer.warn()
    finally:
      if isDone:
        self.mIn.clear()
        if self.mOut.token is not rmsg.TOK_NONE and self._isVerbose :
          toLog("Reply `{0}`".format(self.mOut.out_message_str))
      self.greenLED.off()
      return isDone

# ----------------------------------------------------------------------------
