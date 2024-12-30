# ----------------------------------------------------------------------------
# hexapod_server.py
# Definition of the class `HexaPodClient`, derived from `BehaveEngine`
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-09-04, First version
# 2021-04-28, Make compatible to MicroPython on the FeatherS2
#
# ----------------------------------------------------------------------------
from hexa_global import *
from client.behave_engine import BehaveEngine
import robotling_lib.misc.rmsg as rmsg
import robotling_lib.misc.ansi_color as ansi
from robotling_lib.misc import minigui as gui

from robotling_lib.platform.platform import platform as pf
if pf.languageID == pf.LNG_CIRCUITPYTHON:
  from robotling_lib.platform.circuitpython import time  
elif pf.languageID == pf.LNG_MICROPYTHON:
  import time
else:
  print(ansi.RED +"ERROR: No matching libraries in `platform`." +ansi.BLACK)

# ----------------------------------------------------------------------------
class HexaPodClient(BehaveEngine):
  """HexapodClient class"""

  def __init__(self):
    super().__init__()

    # Initialize
    self._isRunning = False
    self._serverConnected = False
    self._visualize = self.Display is not None and self.Cfg.START_W_DISPLAY
    self._currCol = PIXEL_COL_ORANGE

    # Create message objects for serial commands
    assert self._uartServ, "No message interface to server defined"
    self.mCmd = rmsg.RMsgUARTMPy(self._uartServ, typeMsgOut=rmsg.MSG_Client)
    self.mDta = rmsg.RMsgUARTMPy(self._uartServ, typeMsgOut=rmsg.MSG_Server)

    # Setup and start spin function
    self.spin_ms(period_ms=self.Cfg.TM_PERIOD, callback=self.housekeeper)
    self.spin_ms(100)

    # Done
    self.startPulsePixel(self._currCol)
    toLog("Hexapod (client) ready.", color=ansi.CYAN)
    self._isRunning = True

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def loop(self):
    """ Main loop
    """
    try:
      round = 0
      print("Entering loop ...")
      if self._visualize:
        self.startVisualize()

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
              if round % 10000 == 0:
                self.sendSTA()
              pass
              # **********************
              # **********************
              # **********************
              # **********************

            # **************************
            # **************************
            # **************************
            """
            if round % 1000 == 0 and self.TRMini:
              self.TRMini.update(raw=False)
              distance = self.TRMini.distance
              invalids = self.TRMini.invalids
              print(distance, invalids)
              '''
              s = ""
              for d in data:
                ch1 = "#"
                ch2 = "_"
                if d == 0xFFFF:
                  n = 15
                elif d == 0x0001:
                  n = 15
                  ch1 = "?"
                elif d == 0:
                  n = 0
                  ch2 = "."
                else:
                  n = int(min(15*d/300, 15))
                s += ch1*n +ch2*(15-n) +" "
              print(s)
              '''
            """
            # **************************
            # **************************
            # **************************
            '''
            if round % 10000 == 0:
              self._MCP3208.channelMask = 0x0F
              self._MCP3208.update()
              print(self._MCP3208.data)
            '''
            # **************************
            # **************************
            # **************************

            # Call visualize, if enabled
            if self._visualize and round % 500 == 0:
              self.visualize()

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
      # Update distance sensor(s), if any
      if self.TRMini:
        self.TRMini.update(raw=False)

    # Change RGB pixel according to state
    if self._serverConnected:
      col = PIXEL_COL_GREEN
    else:
      col = PIXEL_COL_ORANGE
    if not col == self._currCol:
      self.startPulsePixel(col)
      self._currCol = col

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def startVisualize(self):
    """ Initialize widgets on the connected OLED display
    """
    # Pass display to GUI and clear it
    gui.Widget.init(self.Display)
    self.Display.clear()
    self.Display.auto_refresh = False

    # Add GUI widgets
    w, h = self.Display.size
    dyLn = h//8
    try:
      # ... a heartbeat indicator
      self._hbeat = gui.WidgetHeartbeat([w-dyLn, 0, dyLn, dyLn])

      # ... battery indicators
      r = [0, 0, w//2, dyLn]
      self._servoBatt = gui.WidgetBattery(r, [0,9], "V", "Servo ")
      r[1] += dyLn
      self._logicBatt = gui.WidgetBattery(r, [0,9], "V", "Logic ")

      # ... a bar graph for each distance sensor
      self._guiDist = []
      for x in [0, w//2]:
        for y in [h//2, 3*h//4]:
          r = [x, y, w//2, h//4]
          self._guiDist.append(gui.WidgetBar(r, [0,50], "cm"))
    finally:
      self.Display.show()

  def visualize(self):
    """ Update widgets on the OLED display
    """
    try:
      self._hbeat.draw()
      self._servoBatt.draw(self.HPR.servoBattery_mV)
      self._logicBatt.draw(self.HPR.serverLogicBattery_mV)
      if self.TRMini:
        dist = self.TRMini.distances
        invl = self.TRMini.invalids
        for id, d in enumerate(dist):
          s = ""
          if d == 0xFFFF:
            s = ">130"
          elif d == 0x0001:
            s = "n/a"
            d = 0
          else:
            d /= 10
          self._guiDist[id].draw(d, s, str(invl[id]))
    finally:
      self.Display.refresh()

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
      if len(res) > 0:
        # Reply received, convert and save it
        if self.mDta.from_hex_string(res) == rmsg.Err.Ok:
          print("<-", self.mDta)
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
