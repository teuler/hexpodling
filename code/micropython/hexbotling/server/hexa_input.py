#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# hexa_input.py
# Class that comprises the input paramaters that control the gain generator
#
# The MIT License (MIT)
# Copyright (c) 2020-21 Thomas Euler
# 2020-04-02, First version
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
  from robotling_lib.misc.parameter import Parameter
  from ulab import numpy as np
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  from hexbotling.hexa_global import *
  from hexbotling.robotling_lib.misc.parameter import Parameter
  import numpy as np

# ----------------------------------------------------------------------------
class HexaInput(object):
  """ Class that manages the input parameters to control hexapod movement
      and position
  """
  def __init__(self, gait, config):
    self._cfg = config
    self.reset(gait)

  def reset(self, gait):
    """ Initialize input parameters
    """
    v3 = np.zeros(3)
    cfg = self._cfg
    nb = cfg.MAX_BLEND_STEPS

    self.isOn          = False                 # Turns gait generator on/off
    self.doEmergStop   = False                 # Stop walking and all feet down
    self.controlMode   = GGN_MODE_walk         # GGN mode
    self.gaitType      = gait._type            # Currently selected gait

    # Body offset (0=down, 35=default up) and y-shift
    self.bodyYOffs = Parameter(0, [0, cfg.BODY_Y_OFFS_MAX], 0, nb, "mm")
    r = cfg.BODY_Y_SHIFT_LIM
    self.bodyYShift = Parameter(gait.bodyYOffset, [-r, r], unit="mm")

    # Current travel length (x,z), rotation (y), and height
    r = cfg.TRAVEL_X_Z_LIM
    self.x_zTravelLen = Parameter(v3, [-r, r], 0, nb, "mm")
    r = cfg.TRAVEL_ROT_Y_LIM
    self.travelRotY = Parameter(0, [-r, r], 0, nb, "°")
    r = [cfg.LEG_LIFT_MIN, cfg.LEG_LIFT_MAX]
    self.legLiftHeight = Parameter(gait.legLiftHeightDef, r, 0, nb, "mm")

    # Global input for body position (xyzBodyPos[1] is calculated), pitch (x),
    # rotation (y), and roll (z)
    r = cfg.BODY_X_Z_POS_LIM
    self.xyzBodyPos = Parameter(v3, [-r, r], unit="mm")
    r = cfg.BODY_XYZ_ROT_LIM
    self.xyzBodyRot = Parameter(v3, [-r, r], unit="°")

    # Movement speed via adjustible delay and input-depdent delay
    self.delaySpeed_ms = Parameter(80, [0, cfg.DELAY_SPEED_MAX], 0, nb, "ms")
    self.delayInput = Parameter(0, [cfg.DELAY_INPUT_MIN, cfg.DELAY_INPUT_MAX])

    # Target angle [0..359°] when rotating via `travelRotY`
    self.tarAngle_deg = Parameter(0, [GGN_InvalidAngle, 359], unit="°")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def update_blending(self):
    """ Update the parameters with blending
    """
    self.bodyYOffs.update()
    self.x_zTravelLen.update()
    self.travelRotY.update()
    self.legLiftHeight.update()
    self.delaySpeed_ms.update()

  def prepare(self, start):
    """ Prepare to start (`start`=True) or stop walking
    """
    self.isOn = start
    self.doEmergStop = False
    self.controlMode = GGN_MODE_walk

  def set_pos(self, bo_y, bs_y, bp_x_z, br_xyz, lh, tl_x_z, tr_y):
    """ Change body position-related parameters
        with bo_y     bodyYOffs; 0=down, 35=default up
             bs_y     bodyYShift;
             bp_x_z   bodyPos; global body position
             br_xyz   bodyRot; global pitch (x), rotation (y) and roll (z)
             lh       legLiftHeight; current travel height
             tl_x_z   travelLen; current travel length X,Z
             tr_y     travelRotY; current travel rotation Y
        Use `None` for a parameter that should not be changed.
    """
    self.doEmergStop = False
    self.bodyYOffs.val = bo_y
    self.bodyYShift.val = bs_y
    self.xyzBodyPos.val = bp_x_z
    self.xyzBodyRot.val = br_xyz
    self.legLiftHeight.val = lh
    self.x_zTravelLen.val = tl_x_z
    self.travelRotY.val = tr_y

  def set_timing(self, ds, di):
    """ Change motion timing
        with ds       delaySpeed_ms; ddjustible delay in ms
             di       delayInput; delay that depends on the input to get
                      the "sneaking" effect
        Use `None` for a parameter that should not be changed.
    """
    self.doEmergStop = False
    self.delaySpeed_ms.val = ds
    self.delayInput.val = di

  def set_pos_timing(self, bo_y, lh, tl_x_z, tr_y, ds, hd=GGN_InvalidAngle):
    """ Change the most important position and timing parameters
        with bo_y     bodyYOffs; 0=down, 35=default up
             lh       legLiftHeight; current travel height
             tl_x_z   travelLen; current travel length X,Z
             tr_y     travelRotY; current travel rotation Y
             ds       delaySpeed_ms; ddjustible delay in ms
             hd       target heading in [°] (for a turn)
        Use `None` for a parameter that should not be changed.
    """
    self.doEmergStop = False
    self.bodyYOffs.val = bo_y
    self.legLiftHeight.val = lh
    self.x_zTravelLen.val = tl_x_z
    self.travelRotY.val = tr_y
    self.delaySpeed_ms.val = ds
    self.tarAngle_deg.val = hd % 360 if hd is not GGN_InvalidAngle else -1

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def log(self):
    print("    bodyYOffs = ", self.bodyYOffs)
    print("   bodyYShift = ", self.bodyYShift)
    print("legLiftHeight = ", self.legLiftHeight)
    print(" x_zTravelLen = ", self.x_zTravelLen)
    print("   travelRotY = ", self.travelRotY)
    print("   xyzBodyPos = ", self.xyzBodyPos)
    print("   xyzBodyRot = ", self.xyzBodyRot)
    print("delaySpeed_ms = ", self.delaySpeed_ms)
    print("   delayInput = ", self.delayInput)
    print(" tarAngle_deg = ", self.tarAngle_deg)

# ----------------------------------------------------------------------------
