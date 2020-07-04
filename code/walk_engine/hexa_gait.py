#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# hexa_gait.py
# Gait class
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
  from misc.helpers import timed_function
  import ulab as np
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  import numpy as np
  from walk_engine.hexa_global import *

# ----------------------------------------------------------------------------
class HexaGait(object):
  """ Gait description and status parameters
  """
  def __init__(self, type=GGN_GAIT_default):
    # Initialize
    self._type = type
    self.reset()

  def reset(self):
    self.iGaitStep           = 1     # Index of current gait step
    self.iGaitLegIn          = 0     # Input number of the leg
    self.isGaitInMotion      = False # Gait is in motion
    self.isLastLeg           = False # Current leg is last leg of the sequence

    # Relative X,Y,Z positions, y rotations and initial leg positions
    self.xyzPos = np.zeros((LEG_COUNT, 3))
    self.rotY = np.zeros(LEG_COUNT)
    self.legNr = np.zeros(LEG_COUNT)

    if self._type == GGN_GAIT_default:
      # Tripod 6-steps (= gait type 4 in Phoenix code)
      self.nStepsInGait      = 8     # Number of steps in gait
      self.legLiftHeightDef  = 50    # Default travel height
      self.isHalfLiftHeigth  = True  # Outer positions of lifted half height
      self.TLDivFact         = 4     # n steps w/ leg on the floor (walking)
      self.nPosLifted        = 3     # n positions single leg is lifted (1-3)
      self.nomSpeed          = 70    # Nominal speed of the gait
      self.legNr[LEG_LR]     = 5     # Initial leg positions ...
      self.legNr[LEG_RF]     = 1
      self.legNr[LEG_LM]     = 1
      self.legNr[LEG_RR]     = 1
      self.legNr[LEG_LF]     = 5
      self.legNr[LEG_RM]     = 5

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  #@timed_function
  def _generate_leg_gait_seq(self, GGN, iLeg):
    """ Generate the gait sequence for the selected leg
    """
    # Initialize
    inp = GGN._Inp
    isGIM = self.isGaitInMotion
    iStep = self.iGaitStep
    nStep = self.nStepsInGait
    nPosL = self.nPosLifted
    xyzGP = self.xyzPos[iLeg]
    yGR = self.rotY[iLeg]
    legNr = self.legNr[iLeg]
    tdl = self.TLDivFact

    # If gait is not in motion, reset travel parameters
    if not isGIM:
      inp.x_zTravelLen.val = [0,0,0]
      inp.travelRotY.val = 0

    # Leg middle up position
    if ((isGIM and (nPosL == 1 or nPosL == 3) and iStep == legNr) or
        (not isGIM and (iStep == legNr) and
         ((abs(xyzGP[0]) > 2) or (abs(xyzGP[2]) > 2) or (abs(yGR) > 2)))):
      # Up ...
      y = -inp.legLiftHeight.val
      self.xyzPos[iLeg] = np.array([0, -inp.legLiftHeight.val, 0])
      self.rotY[iLeg] = 0

    else:
      # Optional half heigth rear
      if (isGIM and
          ((nPosL == 2 and iStep == legNr) or
           (nPosL == 3 and
            (iStep == legNr -1 or
             iStep == legNr +nStep -1)))):
        lh = inp.legLiftHeight.val
        self.xyzPos[iLeg,0] = -inp.x_zTravelLen.x /2
        self.xyzPos[iLeg,1] = -lh/2 if self.isHalfLiftHeigth else -lh
        self.xyzPos[iLeg,2] = -inp.x_zTravelLen.z /2
        self.rotY[iLeg] = -inp.travelRotY.val /2

      else:
        # Optional half heigth front
        if (isGIM and nPosL >= 2 and
            (iStep == legNr +1 or
             iStep == legNr -(nStep -1))):
          lh = inp.legLiftHeight.val
          self.xyzPos[iLeg,0] = inp.x_zTravelLen.x /2
          self.xyzPos[iLeg,1] = -lh/2 if self.isHalfLiftHeigth else -lh
          self.xyzPos[iLeg,2] = inp.x_zTravelLen.z /2
          self.rotY[iLeg] = inp.travelRotY.val /2
        else:
          # Leg front down position
          if((iStep == (legNr +nPosL) or
              iStep == (legNr -(nStep-nPosL))) and xyzGP[1] < 0):
            self.xyzPos[iLeg,0] = inp.x_zTravelLen.x /2
            self.xyzPos[iLeg,1] = 0
            self.xyzPos[iLeg,2] = inp.x_zTravelLen.z /2
            self.rotY[iLeg] = inp.travelRotY.val /2
          else:
            # Move body forward
            self.xyzPos[iLeg,0] = xyzGP[0] -inp.x_zTravelLen.x /tdl
            self.xyzPos[iLeg,1] = 0
            self.xyzPos[iLeg,2] = xyzGP[2] -inp.x_zTravelLen.z /tdl
            self.rotY[iLeg] -= inp.travelRotY.val /tdl

    # Advance to the next step
    if iLeg == (LEG_COUNT -1):
      self.iGaitStep += 1
      if self.iGaitStep > nStep:
        self.iGaitStep = 1

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def _is_step_complete(self):
    """ Checks if GGN has not finished the last move
    """
    raise NotImplementedError("_is_step_complete")
    '''
    int gaitPeak = 0;
    // Finding any incident of GaitPos/Rot <>0
    //
    for(byte iL=0; iL<nLegs; iL+=1) {
      if(gaitPeak < abs(currGait.gaitPos[iL].x))
        gaitPeak = abs(currGait.gaitPos[iL].x);
      else
      if(gaitPeak < abs(currGait.gaitPos[iL].y))
        gaitPeak = abs(currGait.gaitPos[iL].y);
      else
      if(gaitPeak < abs(currGait.gaitPos[iL].z))
        gaitPeak = abs(currGait.gaitPos[iL].z);
      else
      if(gaitPeak < abs(currGait.gaitRotY[iL]))
        gaitPeak = abs(currGait.gaitRotY[iL]);
    }
    return (gaitPeak > 2);
    '''

# ----------------------------------------------------------------------------
