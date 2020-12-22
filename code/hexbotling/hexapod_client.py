# ----------------------------------------------------------------------------
# hexapod_server.py
# Definition of the class `HexaPodClient`, derived from `xxx`
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-09-04, First version
#
# ----------------------------------------------------------------------------
from hexa_global import *
from client.behave_engine import BehaveEngine
import robotling_lib.misc.rmsg as rmsg
import robotling_lib.misc.ansi_color as ansi
from robotling_lib.platform.platform import platform
'''
if platform.languageID == platform.LNG_MICROPYTHON:
  import time
elif platform.languageID == platform.LNG_CIRCUITPYTHON:
  from robotling_lib.platform.circuitpython import time
else:
  print("ERROR: No matching hardware libraries in `platform`.")
'''
# ----------------------------------------------------------------------------
class HexaPodClient(BehaveEngine):
  """HexapodClient class"""

  def __init__(self):
    super().__init__()

    # **********************
    # **********************
    # **********************
    self._currCol = PIXEL_COL_ORANGE
    # **********************
    # **********************
    # **********************

    # Initialize
    self._isRunning = False
    self._serverConnected = False

    # Create message objects for serial commands
    assert self._uartServ, "No message interface to server defined"
    self.mCmd = rmsg.RMsgUARTMPy(self._uartServ, typeMsgOut=rmsg.MSG_Client)
    self.mDta = rmsg.RMsgUARTMPy(self._uartServ, typeMsgOut=rmsg.MSG_Server)

    # Setup and start spin function
    self.spin_ms(period_ms=self.Cfg.TM_PERIOD, callback=self.housekeeper)
    self.spin_ms(100)

    # **********************
    # **********************
    # **********************
    self.startPulsePixel(self._currCol)
    # **********************
    # **********************
    # **********************

    # Done
    self._isRunning = True
    toLog("Hexapod (client) ready.", color=ansi.CYAN)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def loop(self):
    """ Main loop
    """
    try:
      round = 0
      print("Entering loop ...")

      try:
        while True:
          try:
            if not self._serverConnected:
              # Ping the server until it responds
              if round % 5000 == 0:
                print("Pinging server ....")
                if self.sendSTA():
                  toLog("Connected to server.", color=ansi.GREEN)
            else:
              if round % 15000 == 0:
                self.sendSTA()
              pass
              # **********************
              # **********************
              # **********************
              # **********************

            # **************************
            # **************************
            # **************************
            if round % 15000 == 0 and self._BNO055:
              print(self._BNO055.euler)
              print(self._BNO055.temperature)
            if round % 2000 == 0 and self.TRMini:
              self.TRMini.update(raw=False)
              print(self.TRMini.distance, self.TRMini.invalids)
            # **************************
            # **************************
            # **************************
            if round % 10000 == 0:
              self._MCP3208.channelMask = 0x0F
              self._MCP3208.update()
              print(self._MCP3208.data)
            # **************************
            # **************************
            # **************************

          finally:
            # Make sure the robot's housekeeping gets updated once per loop
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
      # TODO
      pass

    # Change RGB pixel according to state
    if self._serverConnected:
      col = PIXEL_COL_GREEN
    else:
      col = PIXEL_COL_ORANGE
    if not col == self._currCol:
      self.startPulsePixel(col)
      self._currCol = col
    # **************************
    # **************************
    # **************************

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def sendSTA(self):
    """ Request status
    """
    self.mCmd.reset(rmsg.TOK_STA)
    ok = self._sendCommand()
    if ok:
      # Save status in the internal robot representation
      self.HPR.fromMsg(self.mDta)
    self._serverConnected = ok
    return ok

  def _sendCommand(self):
    """ Send command, assuming that `self.mCmd` has been filled.
        Returns `True` if a reply was received (in `self.mDta`)
    """
    res = ""
    try:
      # send command
      self.onboardLED.on()
      res = self.mCmd.send()
      if res:
        # Reply received, convert and save it
        self.mDta.from_string(res)
        print("<-", self.mDta, "(", res, ")")
    finally:
      self.onboardLED.off()
      return len(res) > 0

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def onSerialData(self):
    """ Handle new serial data
    """
    return True
    '''
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
    '''

# ----------------------------------------------------------------------------
