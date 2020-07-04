#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# hexa_config_vorpal.py
# Geometrical configuration of the modified Vorpal 3DOF hexapod
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-04-02, First version
#
# ----------------------------------------------------------------------------
import math
try:
  ModuleNotFoundError
except NameError:
  ModuleNotFoundError = ImportError
try:
  # Micropython imports
  from micropython import const
  from hexa_global import *
  import ulab as np
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  import numpy as np
  from walk_engine.hexa_global import *

# ----------------------------------------------------------------------------
class HexaConfig(object):

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
                               -1,-1,-1,-1,-1,-1,
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
  TIB_CORR_ANGLE   = const(50)

  # Limits within movements are considered finished
  TRAVEL_DEAD_ZONE = const(2)
  TURN_DEAD_ZONE   = const(5)

  # Max. number of blending steps (defaul is 4)
  MAX_BLEND_STEPS  = const(4)

  # Limits of movement control parameters
  # (Rotations are in [°], travel in [mm], and  delays in [ms])
  BODY_Y_OFFS_MAX  = const(70)
  BODY_Y_SHIFT_LIM = const(64)
  BODY_X_Z_POS_LIM = np.array([15, BODY_Y_OFFS_MAX +BODY_Y_SHIFT_LIM, 15])
  BODY_XYZ_ROT_LIM = np.array([5, 20, 5])
  TRAVEL_X_Z_LIM   = np.array([25, 0, 25])
  TRAVEL_ROT_Y_LIM = const(25)
  LEG_LIFT_MIN     = const(40)
  LEG_LIFT_MAX     = const(80)
  DELAY_INPUT_MIN  = const(0)
  DELAY_INPUT_MAX  = const(255)
  DELAY_SPEED_MAX  = const(2000)

  # Start position of feet
  # (Measured from beginning of coxa; leg coordinate system (?))
  FEET_INIT_XYZ    = np.array([[+85, 75, 0], [+45, 75,-76], [ 45, 75,-76],
                               [ 85, 75, 0], [ 45, 75, 76], [+45, 75, 76]])
  # Devices
  # - Servo controller    : "pca9685", "minimaestro18"
  # - Network             : "wlan"
  # - Client via UART     : "uart_client"
  DEVICES          = ["minimaestro18", "uart_client"]

  # Period of housekeeping timer
  TM_PERIOD        = const(50)

  # Level of verbosity (the higher the more)
  VERBOSE          = const(0)

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
      x = -math.cos(math.radians(i*self.BODY_COX_ANG)) *self.BODY_R
      z = -math.sin(math.radians(i*self.BODY_COX_ANG)) *self.BODY_R
      temp.append([x, 0, z])
    self.COX_OFF_XYZ = np.array(temp)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def get_servo_ranges_us(self):
    """ Return the servo calibration values for the respective servo
        controller
    """
    if "minimaestro18" in self.DEVICES:
      return [# Coxa   -45.. +45
              ( 965, 1812),
              (1001, 1857),
              ( 880+20, 1783+20),
              (1127+40, 1960+40),
              (1030, 1970),
              (1074, 1931),
              # Femur  -55.. +55
              ( 851+220, 2132+220),
              ( 675+50,  2000+50),
              ( 942+200, 2307+200),
              ( 624+200, 1931+200),
              ( 624+250, 1905+250),
              ( 857+240, 2242+240),
              # Tibia  -90.. +35
             #( 884, (1890 -884)/90 *(35 +8) +1890),
              ( 782, (1847 -782)/90 *(35 -2) +1847),
              ( 634, (1764 -634)/90 *(35 +0 -4) +1764),
              ( 846, (1850 -846)/90 *(35 +6 -5) +1850),
              ( 943, (1985+35 -943)/90 *(35 -9 +9) +1985+35),
              (1027, (1995-1027)/90 *(35 +6) +1995),
              ( 840, (1900 -840)/90 *(35 -2) +1900)]
    else:
      return []

# ----------------------------------------------------------------------------