# ----------------------------------------------------------------------------
# hexapod_server.py
# Definition of the class `HexaPodServer`, derived from `WalkEngine`
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-01-06, First version
# 2020-12-21, Moved pulsing functionality to `robotling_base`
# 2021-01-17, Servo load measurement and calibration added
# ----------------------------------------------------------------------------
import array
from hexa_global import *
from server.walk_engine import WalkEngine
import robotling_lib.misc.rmsg as rmsg
from robotling_lib.misc.helpers import timed_function

from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  import time
else:
  print(ansi.RED +"ERROR: This is MicroPython-only." +ansi.BLACK)

# ----------------------------------------------------------------------------
class HexaPodServer(WalkEngine):
  """HexapodServer class"""

  def __init__(self):
    super().__init__()

    # Initialize
    self._isVerbose = self.Cfg.VERBOSE > 0
    self._isRunning = False
    self._isCalibrating = False
    self.HPR.dialState = DialState.NONE
    self.HPR.dialStateChanged = True
    self.HPR.hexState = HexState.Undefined
    self.Tele = None

    # Create message objects for serial commands
    self._lastBLEConnectState = False
    self._clearCmdBuffer = False
    assert self._uart or self._bsp, "No message interface to client defined"
    self.init_msg_objects()

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
        # Directly engages walk engine to test the client-server connection
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
              if self.HPR.dialState in [0,1,3]:
                self.Buzzer.beep(220, 20)

                if self.HPR.dialState == DialState.STOP:
                  # Stop GGN, assume sitting position and power-down
                  self.GGN.stop()
                  self.setPosture(post=self.POST_SITTING)
                  self.servoPower = False
                  self.HPR.hexState = HexState.Stop

                elif self.HPR.dialState == DialState.AD:
                  # Power up and assume a neutral position
                  self.servoPower = True
                  self.setPosture(post=self.POST_NEUTRAL)
                  self._hexState = HexState.Adjust

                elif self.HPR.dialState == DialState.DEMO:
                  # Activate GGN
                  self.servoPower = True
                  self.setPosture(post=self.POST_WALK)
                  self.GGN.start()
                  self.HPR.hexState = HexState.WalkEngineEngaged

                toLog("Dial=={0} -> {1}"
                      .format(self.HPR.dialState,
                              HexStateStr[self.HPR.hexState]))

            # If walk engine engaged, check for new command and handle it
            if self.HPR.hexState == HexState.WalkEngineEngaged and self.mMsg:
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
            if self.is_ble_defined and round % self.Cfg.CHECK_BLE_ROUNDS == 0:
              self.check_ble_and_connect()

            # During calibration, collect servo load data
            if self._isCalibrating and round % self.Cfg.CAL_COLCT_ROUNDS == 0:
              if self._cal_n < self.Cfg.CAL_MAX_N:
                self._cal_data[self._cal_n] = np.array(self._MCP3208.data)
                self._cal_n += 1

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

      # Keep robot's representation up to date
      self.updateRepresentation()

    # Change RGB pixel according to state
    col = HexStateCol[self.HPR.hexState]
    rmt = self._bsp and self._bsp.is_connected
    if self.HPR.hexState == HexState.WalkEngineEngaged and rmt:
      col = PIXEL_COL_BLUETOOTH
    self.startPulsePixel(col)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def _ack(self, cmd_tok):
    self.mRpl.token = rmsg.TOK_ACK
    self.mRpl.add_data("C", [cmd_tok])
    self.mRpl.send(tout_ms=0)

  def _sta(self):
    self.mRpl.token = rmsg.TOK_STA
    data = array.array("h", [0]*23)
    data[0] = self.HPR.hexState
    data[1] = self.GGN.state
    data[2] = self.HPR.dialState
    data[3] = 1 if self.HPR.servoPower else 0
    data[4] = self.HPR.servoBattery_mV
    data[5] = self.HPR.logicBattery_mV
    data[6] = int(self.HPR.headPitchRoll_deg[0])
    data[7] = int(self.HPR.headPitchRoll_deg[1])
    data[8] = int(self.HPR.headPitchRoll_deg[2])
    data[9:] = self.HPR.servoLoad
    data[17:] = self.HPR.legYPos
    self.mRpl.add_data("S", data)
    self.mRpl.send(tout_ms=0)

  def onSerialCommand(self):
    """ Handle new serial command
    """
    isDone = False
    #try:
    if True:
      self.yellowLED.on()
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
        self._ack(rmsg.TOK_XP0)
        self.assumePosture(self.POST_NEUTRAL)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_STA:
        # Status request
        self._sta()
        isDone = True

      elif self.mMsg.token == rmsg.TOK_CAL:
        # Start/stop collecting calibration data
        self._ack(rmsg.TOK_CAL)
        self.handleCalibration(self.mMsg[0,0] > 0)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_GG0:
        # Start or stop the gait generator
        # >GG0 M=a,m G=g;
        # TODO: Use other parameters, such as gait
        self._ack(rmsg.TOK_GG0)
        if self.mMsg[0,0] > 0:
          self.GGN.start()
        else:
          self.GGN.stop()
        isDone = True

      elif self.mMsg.token == rmsg.TOK_GGP:
        # Change walk parameters of the gait generator (GGN), positions etc.
        # >GGP B=bs,px,pz,bx,by,bz T=bo,lh,tx,tz,ty;
        self._ack(rmsg.TOK_GGP)
        msg = self.mMsg._msg
        p0 = rmsg.TOK_addrPSetStart +rmsg.TOK_offsVals
        p1 = p0 +msg[p0 -rmsg.TOK_offsNVal] +rmsg.TOK_offsVals
        bo_y = msg[p1]
        bs_y = msg[p0]
        bp_x_z = np.array([msg[p0+1], 0, msg[p0+2]])
        br_xyz = np.array([msg[p0+3], msg[p0+4], msg[p0+5]])
        lh = msg[p1+1]
        tl_x_z = np.array([msg[p1+2], 0, msg[p1+3]])
        tr_y = msg[p1+4]
        self.GGN.Input.set_pos(bo_y, bs_y, bp_x_z, br_xyz, lh, tl_x_z, tr_y)
        isDone = True

      elif self.mMsg.token == rmsg.TOK_GGQ:
        # Change only most important walk parameters and request status quickly
        # >GGQ T=bo,lh,tx,tz,ty D=ds A=ta;
        # <STA ...;
        msg = self.mMsg._msg
        p0 = rmsg.TOK_addrPSetStart +rmsg.TOK_offsVals
        p1 = p0 +msg[p0 -rmsg.TOK_offsNVal] +rmsg.TOK_offsVals
        bo_y = msg[p0]
        lh = msg[p0+1]
        tl_x_z = np.array([msg[p0+2], 0, msg[p0+3]])
        tr_y = msg[p0+4]
        ds = msg[p1]
        self.GGN.Input.set_pos_timing(bo_y, lh, tl_x_z, tr_y, ds)
        self._sta()
        isDone = True

      else:
        toLog("Command unknown", ErrCode.CmdUnknown)
        self.Buzzer.warn()
    #finally:
    if isDone:
      self.mMsg.reset(clearBuf=self._clearCmdBuffer)
      if self.mRpl.token is not rmsg.TOK_NONE and self._isVerbose:
        toLog("Reply `{0}`".format(self.mRpl))
    self.yellowLED.off()
    return isDone

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def is_ble_defined(self):
    return self._bsp is not None

  def init_msg_objects(self):
    """ Initialize message objects
    """
    self.mMsg = None
    self.mRpl = None
    self.check_ble_and_connect()

  def check_ble_and_connect(self):
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
    elif self._uart and (self._lastBLEConnectState or self.mMsg is None):
      self.mMsg = rmsg.RMsgUARTMPy(self._uart, typeMsgOut=rmsg.MSG_Server)
      self.mRpl = rmsg.RMsgUARTMPy(self._uart, typeMsgOut=rmsg.MSG_Server)
      self._lastBLEConnectState = False
      self._clearCmdBuffer = False
      toLog("Autonomous mode ...")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def handleCalibration(self, doCal):
    """ Starts or stops calibration data collection and processes the data
    """
    if self._isCalibrating and not doCal:
      # End calibration and process data ...
      print("Calibration ended, processing ...")
      # ...
      '''
      SERVO_LOAD_CHANS = 4
      CAL_MAX_N = 1000
      CAL_BIN_N = 40
      CAL_LOW_P = 5
      CAL_HIGH_P = 95

      lims = np.zeros((SERVO_LOAD_CHANS, 3), dtype=np.int16)
      data = _cal_data.transpose()

      # For all servo load channels ...
      for iCh in range(SERVO_LOAD_CHANS):
        # Get range of load values
        trace = data[iCh][:_cal_n]
        mx = numerical.max(trace)
        mn = numerical.min(trace)
        rg = mx -mn

        if rg > 0:
          # if range is larger than 0 compute histogram
          binW = rg /(CAL_BIN_N -1)
          hist = np.zeros((CAL_BIN_N), dtype=np.int16)
          for v in trace:
            hist[int(max(0, min(v /binW, CAL_BIN_N-1)))] += 1

          # Determine percentiles
          sum = 0
          auc = numerical.sum(hist)
          lo = auc *CAL_LOW_P /100
          hi = auc *CAL_HIGH_P /100
          lim = np.array([-1, -1, 0])
          for i, n in enumerate(hist):
            sum += n
            if sum > lo and lim[0] < 0:
              lim[0] = (i +1) *binW
            if sum > hi and lim[1] < 0:
              lim[1] = (i +1) *binW
          lim[2] = lim[1] -lim[0]
          lims[iCh] = lim

          print("#{0}: {1:.0f} ... {2:.0f} ({3:.0f})".format(iCh, mn, mx, rg))
          print("    {0} bins of {1:.1f}".format(CAL_BIN_N, binW))
          print("    AUC={0:.1f}, mean={1:.1f}, sd={2:.1f}"
                .format(auc, numerical.mean(trace), numerical.std(trace)))
          print("    {0:.0f}%={1:.1f}, {2:.0f}%={3:.1f}"
                .format(CAL_LOW_P, lim[0], CAL_HIGH_P, lim[1]))
          print("   ", list(hist))

      print(lims)
      ***
      '''
      self._isCalibrating = False

    elif not self._isCalibrating and doCal:
      # Start new calibration
      self._cal_n = 0
      self._cal_data = np.zeros((self.Cfg.CAL_MAX_N, self.Cfg.SERVO_LOAD_CHANS))
      print("Calibration data collection started ...")
      self._isCalibrating = True

# ----------------------------------------------------------------------------
