# ----------------------------------------------------------------------------
# boot.py
# Runs first on boot
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# ----------------------------------------------------------------------------
# Disable debugging information
try:
  import esp
  esp.osdebug(None)
except ImportError:
  pass

try:
  import supervisor
  supervisor.disable_autoreload()
except ImportError:
  pass

# ----------------------------------------------------------------------------
