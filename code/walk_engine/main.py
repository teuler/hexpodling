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
from hexapod import *

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
          # Check for new command
          if r.mIn.receive():
            # Handle command ...
            #print(round, r.mIn)
            isDone = r.onSerialCommand()
            if not isDone:
              toLog("Command not handled", ErrCode.Cmd_NotHandled)
          elif not r.mIn.error == rmsg.Err.Ok:
            toLog("Error parsing command", ErrCode.Cmd_ParsingErr)
            r.Buzzer.warn()

        finally:
          # Keep GGN running and make sure the robot's housekeeping gets
          # updated once per loop
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

# ----------------------------------------------------------------------------
# Create instance of HexaPod, derived from the WalkEngine class
r = HexaPod()

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
