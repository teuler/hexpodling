# ----------------------------------------------------------------------------
# hexapod_server.py
# Definition of the class `HexaPodServer`, derived from `WalkEngine`
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, First version
# 2020-12-21, Moved pulsing functionality to `robotling_base`
# ----------------------------------------------------------------------------
from hexa_global import *
from server.walk_engine import WalkEngine
import robotling_lib.misc.rmsg as rmsg
from robotling_lib.misc.helpers import timed_function
from robotling_lib.platform.platform import platform
'''
if platform.ID == platform.ENV_ESP32_TINYPICO:
  import time
else:
  print("ERROR: No matching hardware libraries in `platform`.")
'''
# ----------------------------------------------------------------------------
class HexaPodServer(WalkEngine):
  """HexapodServer class"""

  def __init__(self):
    super().__init__()

    # Initialize
    self._isVerbose = self.Cfg.VERBOSE > 0
    self._isRunning = False
    self.HPR.dialState = DialState.NONE
    self.HPR.dialStateChanged = True
    self.HPR.hexState = HexState.Undefined
    self.Tele = None

    # Create message objects for serial commands
    self._lastBLEConnectState = False
    self._clearCmdBuffer = False
    assert self._uart or self._bsp, "No message interface to client defined"
    self.switch_msg_objects(isFirst=True)

    # Setup and start spin function
    self.spin_ms(period_ms=self.Cfg.TM_PERIOD, callback=self.housekeeper)
    self.spin_ms(100)

    # Done
    self._isRunning = True
    toLog("Hexapod (server) ready.")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def loop(self):
    """ Main loop
    """
    try:
      round = 0
      isHandled = False
      print("Entering loop ...")

      if self.Cfg.DEBUG_LINK:
        self.servoPower = True
        self.GGN.start()
        self.HPR.hexState = HexState.WalkEngineEngaged

      try:
        while True:
          try:
            # Check if dial has changed
            if self.HPR.dialStateChanged and not self.Cfg.DEBUG_LINK:
              self.HPR.dialStateChanged = False

              # Handle new dial selection
              if self.HPR.dialState in [0,3]:
                self.Buzzer.beep(220, 20)

                if self.HPR.dialState == DialState.STOP:
                  # Stop GGN, assume sitting position and power-down
                  self.GGN.stop()
                  self.setPosture(post=self.POST_SITTING)
                  self.servoPower = False
                  self.HPR.hexState = HexState.Stop

                elif self.HPR.dialState == DialState.AD:
                  '''
                  # Power up and assume a neutral position
                  r.servoPower = True
                  r.setPosture(post=r.POST_NEUTRAL)
                  r._hexState = HexState.Adjust
                  '''
                  pass

                elif self.HPR.dialState == DialState.DEMO:
                  # Activate GGN
                  self.servoPower = True
                  self.GGN.start()
                  self.HPR.hexState = HexState.WalkEngineEngaged

                toLog("Dial=={0} -> {1}"
                      .format(self.HPR.dialState,
                              HexStateStr[self.HPR.hexState]))

            # If walk engine engaged, check for new command and handle it
            if self.HPR.hexState == HexState.WalkEngineEngaged:
              if self.mMsg.receive():
                isHandled = self.onSerialCommand()
                if not isHandled:
                  toLog("Command not handled", err=ErrCode.Cmd_NotHandled)
              elif not self.mMsg.error == rmsg.Err.Ok:
                toLog("Error parsing command", err=ErrCode.Cmd_ParsingErr)
                self.Buzzer.warn()

            # Check if client connection has changed (i.e. internal <-> BLE);
            # (Do this only every cople of seconds and only if more than one
            #  port is available)
            if self.multiple_ports and round % self.Cfg.CHECK_BLE_ROUNDS == 0:
              self.switch_msg_objects()

          finally:
            # Keep GGN running and make sure the robot's housekeeping gets
            # updated once per loop
            if self.HPR.hexState == HexState.WalkEngineEngaged:
              self.GGN.spin()
            self.spin_ms()
            round += 1

      except KeyboardInterrupt:
        self._isRunning = False
        print("Loop stopped.")

    finally:
      # Power down ...
      self.powerDown()
      self.printReport()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def housekeeper(self, info=None):
    """ Does the housekeeping
    """
    if self._isRunning:
      # Check if dial position has changed
      d = self.dialPosition
      if d is not self.HPR.dialState:
        self.HPR.dialState = d
        self.HPR.dialStateChanged = True

      # Read servo load
      # TODO: Read servo load from board sensors

    # Change RGB pixel according to state
    col = HexStateCol[self.HPR.hexState]
    rmt = self._bsp and self._bsp.is_connected
    if self.HPR.hexState == HexState.WalkEngineEngaged and rmt:
      col = PIXEL_COL_BLUETOOTH
    self.startPulsePixel(col)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def onSerialCommand(self):
    """ Handle new serial command
    """
    try:
      self.greenLED.on()
      self.mRpl.reset()
      isDone = False

      if not self.mMsg.error == rmsg.Err.Ok:
        toLog("Serial message error", ErrCode.Cmd_ParsingErr)

      elif self.mMsg.token == rmsg.TOK_VER:
        # Version information requested
        self.mRpl.token = rmsg.TOK_VER
        self.mRpl.add_data("V", [self._HPR.softwareVer])
        self.mRpl.add_data("M", [self.memory[1]])
        self.mRpl.send(tout_ms=0)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_XP0:
        # Move all legs to default positions
        self.mRpl.token = rmsg.TOK_ACK
        self.mRpl.add_data("C", [rmsg.TOK_XP0])
        self.mRpl.send(tout_ms=0)
        self.assumePosture(self.POST_NEUTRAL)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_STA:
        # Status request
        self.updateRepresentation()
        self.mRpl.token = rmsg.TOK_STA
        s = [self.HPR.hexState, self.GGN.state, self.HPR.dialState,
             1 if self.HPR.servoPower else 0,
             self.HPR.servoBattery_mV, self.HPR.logicBattery_mV[self.HPR.SRV]]
        self.mRpl.add_data("S", s)
        # ...
        self.mRpl.send(tout_ms=0)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_GG0:
        # Start or stop the gait generator
        # >GG0 M=a,m G=g;
        # TODO: Use other parameters, such as gait
        if self.mMsg[0,0] > 0:
          self.GGN.start()
        else:
          self.GGN.stop()
        isDone = True

      elif self.mMsg.token == rmsg.TOK_GGP:
        # Change walk parameters of the gait generator (GGN), positions etc.
        # >GGP B=bs,px,pz,bx,by,bz T=bo,lh,tx,tz,ty;
        data = self.mMsg._data
        bo_y = data[1][0]
        bs_y = data[0][0]
        bp_x_z = np.array([data[0][1], 0, data[0][2]])
        br_xyz = np.array([data[0][3], data[0][4], data[0][5]])
        lh = data[1][1]
        tl_x_z = np.array([data[1][2], 0, data[1][3]])
        tr_y = data[1][4]
        self.GGN.Input.set_pos(bo_y, bs_y, bp_x_z, br_xyz, lh, tl_x_z, tr_y)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_GGQ:
        # Change only most important walk parameters and request status quickly
        # >GGQ T=bo,lh,tx,tz,ty D=ds A=ta;
        # <STA ...;
        data = self.mMsg._data
        bo_y = data[0][0]
        lh = data[0][1]
        tl_x_z = np.array([data[0][2], 0, data[0][3]])
        tr_y = data[0][4]
        ds = data[1][0]
        self.GGN.Input.set_pos_timing(bo_y, lh, tl_x_z, tr_y, ds)
        isDone = True

      else:
        toLog("Command unknown", ErrCode.CmdUnknown)
        self.Buzzer.warn()
    finally:
      if isDone:
        self.mMsg.reset(clearBuf=self._clearCmdBuffer)
        if self.mRpl.token is not rmsg.TOK_NONE: # and self._isVerbose:
          toLog("Reply `{0}`".format(self.mRpl.out_message_str))
      self.greenLED.off()
      return isDone

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def multiple_ports(self):
    return self._bsp is not None and self._uart is not None

  def switch_msg_objects(self, isFirst=False):
    """ Switch to BLE messaging (remote controlled) if BLE becomes available,
        otherwise stick with UART messaging
    """
    if self._bsp and self._bsp.is_connected:
      if self._lastBLEConnectState == False:
        self.mMsg = rmsg.RMsgBLEMPy(self._bsp, typeMsgOut=rmsg.MSG_Server)
        self.mRpl = rmsg.RMsgBLEMPy(self._bsp, typeMsgOut=rmsg.MSG_Server)
        self._lastBLEConnectState = True
        self._clearCmdBuffer = True
        toLog("Remote control mode via BLE ...")
    elif self._lastBLEConnectState or isFirst:
      self.mMsg = rmsg.RMsgUARTMPy(self._uart, typeMsgOut=rmsg.MSG_Server)
      self.mRpl = rmsg.RMsgUARTMPy(self._uart, typeMsgOut=rmsg.MSG_Server)
      self._lastBLEConnectState = False
      self._clearCmdBuffer = False
      toLog("Autonomous mode ...")

# ----------------------------------------------------------------------------
