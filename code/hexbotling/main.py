# ----------------------------------------------------------------------------
# main.py
# Main program; is automatically executed after reboot.
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, First version
# 2020-09-04, Version for both server and client
# ----------------------------------------------------------------------------
from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  # Only the server board uses a TinyPICO, hence, import and create an
  # instance of the robot server class
  # the server object
  from server.hexapod_server import HexaPodServer
  r = HexaPodServer()
else:
  # Import and create and instance of the robot client class
  from client.hexapod_client import HexaPodClient
  r = HexaPodClient()

# Call main
if __name__ == "__main__":
  try:
    r.loop()
  except Exception as e:
    # Create an in-memory file-like string object (`sIO`) to accept the
    # exception's traceback (`sys.print_exception` requires a file-like
    # object). This traceback string is then converted into a dictionary
    # to be sent as an MQTT message (if telemetry is activate)
    try:
      if r.Tele and r.Tele.connected:
        import sys, uio
        sIO = uio.StringIO()
        sys.print_exception(e, sIO)
        r.Tele.publishDict(KEY_RAW, {KEY_DEBUG: str(sIO.getvalue())})
      # Re-raise the exception such that it becomes also visible in the REPL
      raise e
    except AttributeError:
      raise e

# ----------------------------------------------------------------------------
