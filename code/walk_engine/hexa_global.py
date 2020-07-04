# ----------------------------------------------------------------------------
# hexa_global.py
# General definitions
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
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
  import ulab as np
  MICROPYTHON = True
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
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
GGNStateStr          = dict([
  (GGNState.Idle, "idle"),
  (GGNState.IsMoving, "isMoving"),
  (GGNState.NeedsToCompute, "needsToCompute"),
  (GGNState.UpdateServos, "updateServos")])

# ----------------------------------------------------------------------------
# General state
class HXAState:
  Standby            = const(0)  # Hardware initialized, GGN off
  Ready              = const(1)  # GGN on but not walking
  Walking            = const(2)  # GGN on and walking
  Error              = const(-1)

HXAStateStr          = dict([
  (HXAState.Standby, "on standby"),
  (HXAState.Ready, "ready"),
  (HXAState.Walking, "walking"),
  (HXAState.Error, "ERROR")])

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
  # ...

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
# Other definitions
ANSI_BLACK           = "\x1b[0m"
ANSI_RED             = "\x1b[91m"
ANSI_RED_BKG         = "\x1b[41m"
ANSI_GREEN           = "\x1b[92m"
ANSI_GREEN_BKG       = "\x1b[42m"
ANSI_CYAN            = "\x1b[96m"
ANSI_CYAN_BKG        = "\x1b[46m"
ANSI_BLUE            = "\x1b[94m"
ANSI_BLUE_BKG        = "\x1b[44m"
ANSI_LILAC           = "\x1b[95m"
ANSI_LILAC_BKG       = "\x1b[45m"

# ----------------------------------------------------------------------------
def toLog(sMsg, sTopic="", err=ErrCode.Ok):
  """ Print message to history
  """
  if err == ErrCode.Ok:
    s = "INFO" if len(sTopic) == 0 else sTopic
    c = ""
  elif err > 0:
    s = "WARNING"
    c = ANSI_CYAN
  else:
    s = "ERROR"
    c = ANSI_RED
  print(c +"[{0:>12}] {1:35}".format(s, sMsg) +ANSI_BLACK)

# ----------------------------------------------------------------------------
