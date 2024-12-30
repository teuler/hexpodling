# ----------------------------------------------------------------------------
# hexa_global.py
# General definitions
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-04-02, First version
#
# ----------------------------------------------------------------------------
try:
  ModuleNotFoundError
except NameError:
  ModuleNotFoundError = ImportError
try:
  # Micropython imports
  from micropython import const
  try:
    from ulab import numpy as np
  except ImportError:
    import ulab as np
  shape = lambda x : x.shape()
  import robotling_lib.misc.ansi_color as ansi
  MICROPYTHON = True
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  shape = lambda x : x.shape
  import numpy as np
  MICROPYTHON = False

# ----------------------------------------------------------------------------
# Joint and leg IDs
COX                  = const(0)
FEM                  = const(1)
TIB                  = const(2)

LEG_RM               = const(0)  # Right middle
LEG_RF               = const(1)  # Right front
LEG_LF               = const(2)  # Left front
LEG_LM               = const(3)  # Left middle
LEG_LR               = const(4)  # Left rear
LEG_RR               = const(5)  # Right rear
NO_LEG               = const(6)

LEG_COUNT            = const(6)
R_LEGS               = np.array([LEG_RM, LEG_RF, LEG_RR])
L_LEGS               = np.array([LEG_LF, LEG_LM, LEG_LR])

# Gait generator (GGN) modes
GGN_MODE_translation = const(1)
GGN_MODE_walk        = const(2)
GGN_MODE_singleLeg   = const(3)
GGN_MODE_rotate      = const(4)

# Gait types
GGN_GAIT_default     = const(0)

# Other GGN parameters
GGN_InvalidAngle     = const(-1)

# Inverse kinectics computation results
GGN_IKM_None         = const(0)
GGN_IKM_Solution     = const(1)
GGN_IKM_Warning      = const(10)
GGN_IKM_Error        = const(100)

# GGN state
# (for `idle` and `pending`, CPU time can be used by other routines; for the
#  other states, free CPU time as soon as possible)
class GGNState:
  Idle               = const(0)  # GGN is idle
  IsMoving           = const(1)  # GGN is active and performing a move
  NeedsToCompute     = const(2)  # GGN needs to calculate a set of angles
  UpdateServos       = const(3)  # Move is (almost) finished and servos need
                                 # to be updated
GGNStateStr = dict([
  (GGNState.Idle, "idle"),
  (GGNState.IsMoving, "isMoving"),
  (GGNState.NeedsToCompute, "needsToCompute"),
  (GGNState.UpdateServos, "updateServos")])

# Dial state
class DialState:
  NONE               = const(-1)
  STOP               = const(0)
  AD                 = const(1)
  TS                 = const(2)
  DEMO               = const(3)
  RC                 = const(4)

# ----------------------------------------------------------------------------
# Internal walk engine state
class WEState:
  Standby            = const(0)  # Hardware initialized, GGN off
  Ready              = const(1)  # GGN on but not walking
  Walking            = const(2)  # GGN on and walking
  Error              = const(-1)

WEStateStr = dict([
  (WEState.Standby, "on standby"),
  (WEState.Ready, "ready"),
  (WEState.Walking, "walking"),
  (WEState.Error, "ERROR")])

# HexaPod state
class HexState:
  Undefined          = const(0)
  Stop               = const(1)  # Dial==Stop, Servo power off
  Adjust             = const(2)  # Dial==Adj, POST_NEUTRAL
  WalkEngineEngaged  = const(3)  # Dial==Demo; WEState==Ready or ==Walking
  # ...
  '''
  # *************
  # *************
  Add here also behavioural states?
  # *************
  # *************
  '''
HexStateCol         = bytearray([255, 40, 220, 200, 70])
HexStateStr         = dict([
  (HexState.Undefined, "none"),
  (HexState.Stop, "stopped, servos off"),
  (HexState.Adjust, "adjust, servos neutral"),
  (HexState.WalkEngineEngaged, "walk engine engaged")])

# Other color wheel colors
PIXEL_COL_BLUETOOTH   = 70
PIXEL_COL_BLUE        = 90
PIXEL_COL_ORANGE      = 218
PIXEL_COL_GREEN       = 200
# ...

# ----------------------------------------------------------------------------
# Errors
class ErrCode:
  Cmd_NotHandled     = const(3)
  ServoMoveOverdue   = const(2)
  IKM_Incomplete     = const(1)
  Ok                 = const(0)
  IKM_Failed         = const(-1)
  GGN_Unknown        = const(-2)
  Cmd_Unknown        = const(-3)
  Cmd_ParsingErr     = const(-4)
  LowBattery         = const(-5)
  NotImplemented     = const(-6)
  TookTooLong        = const(-7)
  # ...

# ----------------------------------------------------------------------------
# Other definitions
BATT_SERVO_THRES_V   = 7.2
BATT_LOGIC_THRES_V   = 3.6

BLE_UART_DEVICE_NAME = "hex-uart"

# ----------------------------------------------------------------------------
# MQTT releated definitions
#
# Message keys
KEY_RAW              = "raw"
KEY_DEBUG            = "debug"
KEY_BODY             = "body"
KEY_SHIFT            = "shift"
KEY_SET_MODE         = "mode"
KEY_SET_MODE_REL     = "rel"
KEY_SET_MODE_ABS     = "abs"
KEY_POS              = "pos"
KEY_ROT              = "rot"
KEY_TRAVEL           = "travel"
KEY_DELAY            = "delay"
KEY_CMD              = "cmd"

CMD_BODY_POS_ROT     = "BPR" # body position & rotation
CMD_BODY_Y_SHIFT     = "BYS" # body y shift
CMD_MOVE             = "MOV"
CMD_NOP              = "NOP" # no operation

# ----------------------------------------------------------------------------
def toLog(sMsg, sTopic="", err=ErrCode.Ok, green=False, color=None, head=True):
  """ Print message to history
  """
  c = ""
  if err == ErrCode.Ok:
    s = "INFO" if len(sTopic) == 0 else sTopic
    if green:
      c = ansi.GREEN
  elif err > 0:
    s = "WARNING"
    c = ansi.CYAN
  else:
    s = "ERROR"
    c = ansi.RED
  if color:
    c = color
  if head:
    print(c +"[{0:>12}] {1:35}".format(s, sMsg) +ansi.BLACK)
  else:
    c = ansi.LIGHT_YELLOW if not color else color
    print(c +sMsg +ansi.BLACK)

def toLogArray(a, digits=1):
  """ Print an array to the history
  """
  n,m = shape(a)
  str = "[" if n > 1 else ""
  for i in range(n):
    str += "["
    for j in range(m):
      if n == 1:
        str += "{0:5.{1}1f},".format(a[i], digits)
      else:
        str += "{0:5.{1}f},".format(a[i][j], digits)
    str = str[:-1] +"],"
  str = str[:-1] +"]" if n > 1 else str[:-1]
  print(str)

# ----------------------------------------------------------------------------
