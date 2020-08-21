# ----------------------------------------------------------------------------
# hexapod_server.py
# Definition of the class `HexaPodServer`, derived from `WalkEngine`
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, First version
#
# ----------------------------------------------------------------------------
import ujson
from hexa_global import *
from server.walk_engine import WalkEngine
import robotling_lib.misc.rmsg as rmsg
from robotling_lib.misc.helpers import timed_function

from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  import time
else:
  print("ERROR: No matching hardware libraries in `platform`.")

# ----------------------------------------------------------------------------
class HexaPodServer(WalkEngine):
  """HexapodServer class"""

  def __init__(self):
    super().__init__()

    # Initialize
    self._isVerbose = True
    self.HPR.dialState = DialState.NONE
    self.HPR.dialStateChanged = True
    self.HPR.hexState = HexState.Undefined

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
    self._lastBLEConnectState = False
    '''
    self.mIn = None
    self.mOut = None
    self.recreate_message_objects()
    '''
    if self._uart:
      self._mInUART = rmsg.RMsgUARTMPy(self._uart)
      self._mOutUART = rmsg.RMsgUARTMPy(self._uart, isClient=False)
    if self._bsp:
      self._mInBLE = rmsg.RMsgBLEMPy(self._bsp)
      self._mOutBLE = rmsg.RMsgBLEMPy(self._bsp, isClient=False)
    self.switch_msg_objects()

    # Done
    self.isRunning = True
    toLog("Hexapod ready.")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def multiple_port_types(self):
    return self._bsp is not None and self._uart is not None

  def switch_msg_objects(self):
    """
    """
    if self._bsp and self._bsp.is_connected:
      if self._lastBLEConnectState == False:
        self.mIn = self._mInBLE
        self.mOut = self._mOutBLE
        self._lastBLEConnectState = True
    else:
      self.mIn = self._mInUART
      self.mOut = self._mOutUART
      self._lastBLEConnectState = False


  '''
  def recreate_message_objects(self):
    """ Depending on what devices are defined in the configuration and if
        a bluetooth connection is active, create the appropriate message
        objects for receiving commands
    """
    assert self._bsp or self._uart
    if self._bsp is None or (self._bsp and not self._bsp.is_connected):
      # Normal UART is the only option
      self._lastBLEConnectState = False
      if self._uart:
        if self.mIn:
          if self.mIn.port_type == rmsg.PortType.UART_MPY:
            # Nothing to do
            return
          else:
            # Delete the previous message objects
            self.mIn.deinit()
            self.mOut.deinit()
        # Create new message objects of the UART type
        self.mIn = rmsg.RMsgUARTMPy(self._uart)
        self.mOut = rmsg.RMsgUARTMPy(self._uart, isClient=False)
    else:
      # BLE is available and connected
      if not self._lastBLEConnectState:
        # BLE was not connected before
        self._lastBLEConnectState = True
        if self.mIn:
          if self.mIn.port_type == rmsg.PortType.BLE_MPY:
            # Nothing to do
            return
          else:
            # Delete the previous message objects
            self.mIn.deinit()
            self.mOut.deinit()
        # Create new message objects of the BLE-UART type
        self.mIn = rmsg.RMsgBLEMPy(self._bsp)
        self.mOut = rmsg.RMsgBLEMPy(self._bsp, isClient=False)
    '''

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def housekeeper(self, info=None):
    """ Does the housekeeping:
        - Changes also color of NeoPixel depending on the robot's state
        - ...
    """
    if self.isRunning:
      # Check if dial position has changed
      d = self.dialPosition
      if d is not self.HPR.dialState:
        self.HPR.dialState = d
        self.HPR.dialStateChanged = True

    # ******************************
    # ******************************
    # ******************************
    # CHECK SENSORS AND STUFF
    # ******************************
    # ******************************
    # ******************************

    # Change DotStar according to state
    if self.HPR.hexState == HexState.Stop:
      ds = DSTAR_COL_STOPPED
    elif self.HPR.hexState == HexState.Adjust:
      ds = DSTAR_COL_ADJUST
    elif self.HPR.hexState == HexState.WalkEngineEngaged:
      if self._bsp and self._bsp.is_connected:
        ds = DSTAR_COL_REMOTE
      else:
        ds = DSTAR_COL_AUTONOME
    else:
      ds = DSTAR_COL_NONE
    self.startPulseDotStar(ds)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
        self.mOut.add_data("V", [self._HPR.softwareVer])
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
        self.updateRepresentation()
        self.mOut.token = rmsg.TOK_STA
        s = [self.HPR.hexState, self.GGN.state, self.HPR.dialState,
             1 if self.HPR.servoPower else 0,
             self.HPR.servoBattery_mV, self.HPR.logicBattery_mV]
        self.mOut.add_data("S", s)
        # ...
        self.mOut.send()
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
