#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# hexa_config_vorpal.py
# Geometrical configuration of the modified Vorpal 3DOF hexapod
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-04-02, First version
# ----------------------------------------------------------------------------
try:
  ModuleNotFoundError
except NameError:
  ModuleNotFoundError = ImportError
try:
  # Micropython imports
  from micropython import const
  from hexa_global import *
  from math import radians, sin, cos
  try:
    from ulab import numpy as np
  except ImportError:
    import ulab as np
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  import numpy as np
  from numpy import radians, sin, cos
  from hexbotling.hexa_global import *

# ----------------------------------------------------------------------------
class HexaConfig(object):

  # pylint: disable=bad-whitespace
  # Period of housekeeping timer (in [ms])
  TM_PERIOD        = const(40)

  # Timeouts for serial server-client connection
  UART_TIME_OUT_MS = const(2)
  # ...

  # How often changes in BLE connection are checked
  # (check when `round % CHECK_BLE_ROUNDS == 0`)
  CHECK_BLE_ROUNDS = const(5000)
  CAL_COLCT_ROUNDS = const(100)

  # Robot representation updates
  # (delays for slowly changing parameters; update every x ms)
  HRP_POWER_MS     = const(1000)
  # ...

  # Flags
  VERBOSE          = const(0)   # The higher, the more information
  NO_BUZZER        = const(1)   # Buzzer on/off
  DEBUG_LINK       = const(0)   # Start in command mode to test server-client
  START_W_DISPLAY  = const(0)   # Start w/ display on; slows everything ...

  # Devices
  # - Servo controller    : "minMaestro18"
  # - Network             : "wlan"
  # - Client              : "uart_client", "ble_client"
  # - Distance sensor     : "tera_evomini"
  # - Compass             : "bno055"
  # - Display             : "ssd1327_128x128"
  DEVICES          = ["minMaestro18", "bno055", "ble_client"]

  # Active servo load sensing channels
  SERVO_LOAD_MASK  = 0x00FF
  SERVO_LOAD_CHANS = 4
  CAL_MAX_N        = 1000

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Servo-related definitions
  SERVO_COUNT        = 18
  SERVO_IDS          = np.array(range(SERVO_COUNT), dtype=np.uint8)
  JOINT_BY_SERVO     = np.array([[2, COX],  # Servo #0 -> leg 2, coxa
                                 [3, COX],  # Servo #1 -> leg 3, coxa
                                 [4, COX],  # ...
                                 [5, COX],
                                 [0, COX],
                                 [1, COX],
                                 [2, FEM],
                                 [3, FEM],
                                 [4, FEM],
                                 [5, FEM],
                                 [0, FEM],
                                 [1, FEM],
                                 [2, TIB],
                                 [3, TIB],
                                 [4, TIB],
                                 [5, TIB],
                                 [0, TIB],
                                 [1, TIB]],
                                dtype=np.uint8)

  SERVO_DIR        = np.array([-1,-1,-1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1, #-1,-1,-1,-1,-1,-1,
                                1, 1, 1, 1, 1, 1])

  SERVO_RANGES     = np.array([[-45, +45], [-45, +45], [-45, +45],
                               [-45, +45], [-45, +45], [-45, +45],
                               [-55, +55], [-55, +55], [-55, +55],
                               [-55, +55], [-55, +55], [-55, +55],
                               [-90, +35], [-90, +35], [-90, +35],
                               [-90, +35], [-90, +35], [-90, +35]])

  # Servo by leg and joint
  # LEG_SERVOS[iLeg][iJoint], with constants from `hexawalker_global.py`
  # $$$ = cLegPins[nDOF][nLegs])
  LEG_SERVOS       = np.array([[ 4,10,16],  # Leg 0 (RM) right-middle
                               [ 5,11,17],  # Leg 1 (RF) right-front
                               [ 0, 6,12],  # Leg 2 (LF) left-front
                               [ 1, 7,13],  # Leg 3 (LM) left-middle
                               [ 2, 8,14],  # Leg 4 (LR) left-rear
                               [ 3, 9,15]], # Leg 5 (RR) right-rear
                                dtype=np.uint8)

  # Body dimensions (in [mm] or [°])
  BODY_R           = const(60)     # body radius
  BODY_COX_ANG     = const(60)     # fixed leg offset angle around body
  COX_OFF_XYZ      = []            # positions of legs relative to body
                                   # $$$ =cOffs[nLegs]
  COX_OFF_ANG      = np.array([0, 60, 60, 0, -60, -60])
                                   # $$$ = cCoxaAngles1[nLegs]
  COX_LEN          = const(48)
  FEM_LEN          = const(47)
  TIB_LEN          = const(84)
  TIB_CORR_ANGLE   = const(50)     # 50, originally 90?

  # Limits within movements are considered finished
  TRAVEL_DEAD_ZONE = const(2)
  TURN_DEAD_ZONE   = const(5)

  # Max. number of blending steps (default is 4)
  MAX_BLEND_STEPS  = const(4)

  # Servo manager: `linear` (1) or `paraboloid` (0) trajectory
  # ****
  # TODO: Fix `paraboloid`; currently it results in shaking ...
  # ****
  MOVE_LINEAR      = const(1)

  # Limits of movement control parameters
  # (Rotations are in [°], travel in [mm], and  delays in [ms])
  BODY_Y_OFFS_MAX  = const(70)
  BODY_Y_SHIFT_LIM = const(64)
  BODY_X_Z_POS_LIM = np.array([15, BODY_Y_OFFS_MAX +BODY_Y_SHIFT_LIM, 15])
  BODY_XYZ_ROT_LIM = np.array([5, 20, 5])
  TRAVEL_X_Z_LIM   = np.array([40, 0, 40]) #25
  TRAVEL_ROT_Y_LIM = const(25)
  LEG_LIFT_MIN     = const(40)
  LEG_LIFT_MAX     = const(80)
  DELAY_INPUT_MIN  = const(0)
  DELAY_INPUT_MAX  = const(255)
  DELAY_SPEED_MAX  = const(2000)

  # Start position of feet
  # (Measured from beginning of coxa; leg coordinate system (?))
  FEM_STRT_ANG     = -20
  TIB_STRT_ANG     = 10
  FEET_INIT_XYZ    = []
  # pylint: enable=bad-whitespace

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def __init__(self):
    # Initialize
    # Coordinate system (from Phoenix code):
    #   x-axis points left
    #   y-axis points down
    #   z-axis points backwards
    #
    # Calculate leg displacements (start of the coxa)
    # (considering the above coordinate system)
    temp = []
    for i in range(6):
      x = -cos(radians(i*self.BODY_COX_ANG)) *self.BODY_R
      z = -sin(radians(i*self.BODY_COX_ANG)) *self.BODY_R
      temp.append([x, 0, z])
    self.COX_OFF_XYZ = np.array(temp)
    self.FEET_INIT_XYZ = self.get_foot_pos(self.FEM_STRT_ANG, self.TIB_STRT_ANG)
    #toLogArray(self.FEET_INIT_XYZ, 0)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def get_foot_pos(self, af_deg, at_deg, ac_deg=None):
    """ Calculate starting foot positions from angles
    """
    ac_deg = self.BODY_COX_ANG if ac_deg is None else ac_deg
    ac = radians(ac_deg)
    af = radians(af_deg)
    at = radians(at_deg)

    pos = np.zeros((6,3), dtype=np.int16)
    for i in range(6):
      rx =  self.FEM_LEN*cos(af) -self.TIB_LEN*sin(at +af) +self.COX_LEN
      y  =  self.FEM_LEN*sin(af) +self.TIB_LEN*cos(at -af)
      x  =  rx *cos(ac *i)
      z  = -rx *sin(ac *i)
      pos[i] = np.array([abs(int(x)), int(y), int(z)])
    return pos

  def get_servo_ranges_us(self):
    """ Return the servo calibration values for the respective servo
        controller
    """
    if "minMaestro18" in self.DEVICES:
      s06_0 = 1640
      s07_0 = 1600
      s08_0 = 1570
      s09_0 = 1360
      s10_0 = 1460
      s11_0 = 1493
      d = 500
      return [# Coxa   -45.. +45
              ( 965, 1812),
              (1001, 1857),
              ( 880+20, 1783+20),
              (1127+40, 1960+40),
              (1030, 1970),
              (1074, 1931),
              # Femur  -55.. +55
              ( s06_0 -d, s06_0 +d),
              ( s07_0 -d, s07_0 +d),
              ( s08_0 -d, s08_0 +d),
              ( s09_0 -d, s09_0 +d),
              ( s10_0 -d, s10_0 +d),
              ( s11_0 -d, s11_0 +d),
               # Tibia  -90.. +35
              ( 782, (1847 -782)/90 *(35 -2) +1847),
              ( 634, (1764 -634)/90 *(35 -7) +1764),
              ( 846, (1850 -846)/90 *(35 +6 -3) +1850),
              ( 943, (1985+35 -943)/90 *(35 -9 +9) +1985+35),
              (1027, (1995-1027)/90 *(35 +4) +1995),
              ( 840, (1900 -840)/90 *(35 -2) +1900)]
    else:
      return []

  def get_servo_load_ranges(self):
    """ Return the servo load calibration ranges
    """
    return np.array([[30, 727, 696],
                     [35, 1167, 1131],
                     [235, 1034, 799],
                     [297, 1147, 850],
                     [0, 0, 0],
                     [0, 0, 0],
                     [0, 0, 0],
                     [0, 0, 0]])

# ----------------------------------------------------------------------------
