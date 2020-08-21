# ----------------------------------------------------------------------------
# main.py
# Main program; is automatically executed after reboot.
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, First version
#
# ----------------------------------------------------------------------------
from time import ticks_ms
from hexapod_server import *

# ----------------------------------------------------------------------------
def main():
  # Setup the walk engine's spin function
  r.spin_ms(period_ms=r.Cfg.TM_PERIOD, callback=r.housekeeper)
  r.spin_ms(100)

  # Loop ...
  print("Entering loop ...")
  try:
    try:
      round = 0
      isDone = False

      while True:
        try:
          # Check if dial has changed
          if r.HPR.dialStateChanged:
            r.HPR.dialStateChanged = False
            if r.HPR.dialState in [0,3]:
              r.Buzzer.beep(220, 20)
              if r.HPR.dialState == DialState.STOP:
                # Stop GGN, assume sitting position and power-down
                r.GGN.stop()
                r.setPosture(post=r.POST_SITTING)
                r.servoPower = False
                r.HPR.hexState = HexState.Stop
              elif r.HPR.dialState == DialState.AD:
                '''
                # Power up and assume a neutral position
                r.servoPower = True
                r.setPosture(post=r.POST_NEUTRAL)
                r._hexState = HexState.Adjust
                '''
                pass
              elif r.HPR.dialState == DialState.DEMO:
                # Activate GGN
                r.servoPower = True
                r.GGN.start()
                r.HPR.hexState = HexState.WalkEngineEngaged
              toLog("Dial=={0} -> {1}"
                    .format(r.HPR.dialState, HexStateStr[r.HPR.hexState]))

          # Check for new command
          if r.HPR.hexState == HexState.WalkEngineEngaged and r.mIn.receive():
            # Handle command ...
            isDone = r.onSerialCommand()
            if not isDone:
              toLog("Command not handled", ErrCode.Cmd_NotHandled)
          elif not r.mIn.error == rmsg.Err.Ok:
            toLog("Error parsing command", ErrCode.Cmd_ParsingErr)
            r.Buzzer.warn()

          # Check if client connection has changed (i.e. internal <-> BLE);
          # (Do this only every cople of seconds and only if more than one
          #  port is available)
          if r.multiple_port_types and round % r.Cfg.CHECK_BLE_ROUNDS == 0:
            r.switch_msg_objects()
            """
            r.recreate_message_objects()
            """
        finally:
          # Keep GGN running and make sure the robot's housekeeping gets
          # updated once per loop
          if r.HPR.hexState == HexState.WalkEngineEngaged:
            r.GGN.spin()
          r.spin_ms()
          round += 1

    except KeyboardInterrupt:
      r.isRunning = False
      print("Loop stopped.")

  finally:
    # Power down ...
    r.powerDown()
    r.printReport()
    '''
    print(r._run_duration_s, r._run_duration_s/round, round)
    '''

# ----------------------------------------------------------------------------
# Create instance of HexaPod, derived from the WalkEngine class
r = HexaPodServer()

# Call main
if __name__ == "__main__":
  try:
    main()
  except Exception as e:
    # Create an in-memory file-like string object (`sIO`) to accept the
    # exception's traceback (`sys.print_exception` requires a file-like
    # object). This traceback string is then converted into a dictionary
    # to be sent as an MQTT message (if telemetry is activate)
    if r.Tele and r.Tele.connected:
      import sys, uio
      sIO = uio.StringIO()
      sys.print_exception(e, sIO)
      r.Tele.publishDict(KEY_RAW, {KEY_DEBUG: str(sIO.getvalue())})
    # Re-raise the exception such that it becomes also visible in the REPL
    raise e

# ----------------------------------------------------------------------------
