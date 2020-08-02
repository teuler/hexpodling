#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# hexa_gait_generator.py
# Gait generator (GGN) class
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-04-02, First version
#
# ----------------------------------------------------------------------------
import math
import array
try:
  ModuleNotFoundError
except NameError:
  ModuleNotFoundError = ImportError
try:
  # Micropython imports
  from machine import Timer
  from micropython import const
  from time import ticks_ms
  from hexa_global import *
  from hexa_gait import HexaGait
  from hexa_input import HexaInput
  from misc.helpers import timed_function
  import ulab as np
  from ulab import numerical
  from ulab import vector
except ModuleNotFoundError:
  # Standard Python imports
  const = lambda x : x
  from time import time_ns as ticks_ms
  from walk_engine.hexa_global import *
  from walk_engine.hexa_gait import HexaGait
  from walk_engine.hexa_input import HexaInput
  import numpy as np

# ----------------------------------------------------------------------------
class HexaGaitGenerator(object):
  """ Gait generator (GGN) control and status parameters
  """
  def __init__(self, config, led, servo_man=None, verbose=0):
    # Initialize
    self._isReady = False
    self._verboseLevel = verbose
    self._Cfg = config
    self._SMan = servo_man
    self._LED = led
    self._Gait = HexaGait()
    self._Inp = HexaInput(self._Gait, config)
    self.reset()
    self._isReady = True
    toLog("Gait generator (GGN) initialized.")

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def reset(self):
    """ Reset gait generator
    """
    # Initialize GGN control parameters
    self.isOnPrev         = False          # Previous state of GGN on/off switch
    self.state            = GGNState.Idle  # Current and next state of GGN
    self.nextState        = self.state
    self.moveDur_ms       = 0              # Duration of current, next and
    self.lastMoveDur_ms   = 0              # previous move
    self.nextMoveDur_ms   = 0
    self.calcDur_ms       = 0              # Duration of last GGN calculation
    self.waitUntil_ms     = 0

    # Quality control-related
    self.tMoveStart_ms    = 0              # Time when move last was started
    self.lastMoveLate_ms  = 0              # Delay of the last executed move
    self.lastIKSolRes     = GGN_IKM_None   # Last IK solution results
                                           # =errors*100 +warnings*10 +solutions
    # Reset input parameters
    self._Inp.reset(self._Gait)

    # Set start position of each foot
    self._xyzLegPos = np.array(self._Cfg.FEET_INIT_XYZ)
    self._allLegsDown = True

    # Current leg angles from IK and converted into servo angles
    self._legAngles = np.zeros((LEG_COUNT, 3))
    self._servoAngles = np.zeros((LEG_COUNT, 3))

    # General state and error
    self.walkEngineState = WEState.Standby
    self.lastErrC = ErrCode.Ok
    self._prevStates = [self.state, self.walkEngineState, self.lastErrC]

  def change_gait(self, gaitType=GGN_GAIT_default):
    """ Set or change the gait type
    """
    # TODO: Make sure that robot has stopped
    if gaitType != self._Gait._type:
      self._Gait = HexaGait(gaitType)
      self.reset()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def start(self):
    """ Start GGN
    """
    self._Inp.prepare(True)
    self.state = GGNState.NeedsToCompute
    self.walkEngineState = WEState.Ready
    self.log_status()

  def stop(self):
    """ Stop GGN
    """
    self._Inp.prepare(False)
    self.state = GGNState.Idle
    self.walkEngineState = WEState.Standby
    self.log_status()

  def spin(self):
    """ Call frequently to keep GGN running
    """
    if self.state == GGNState.IsMoving:
      # Still executing the last move
      if self.waitUntil_ms > 0 and ticks_ms() > self.waitUntil_ms:
        # Change state to trigger execution of the next move
        self.state = GGNState.UpdateServos
        self.waitUntil_ms = 0

    if self.state == GGNState.NeedsToCompute:
      # Calculate next move and set timer for move execution
      # -> `IsMoving` or, if delay is too short, `NeedsToCompute`
      self._LED.on()
      self._compute_next_move()
      self._LED.off()

    elif self.state == GGNState.UpdateServos:
      # Previous move is finished, now commit next move
      # -> `NeedsToCompute`
      self._execMove()

    elif self.state == GGNState.Idle:
      # `IsMoving`, Timer -> `UpdateServos`
      pass
    if self._verboseLevel > 0:
      self.log_status()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  #@timed_function
  def _compute_next_move(self):
    """ Compute the next move
    """
    # Initialize
    t1_ms = ticks_ms()
    cfg = self._Cfg
    inp = self._Inp
    gait = self._Gait
    self.lastIKSolRes = GGN_IKM_None
    self.lastErrC = ErrCode.Ok

    # Do some checking ...
    if inp.doEmergStop:
      # TODO
      raise NotImplementedError("Not yet implemented")
    if inp.controlMode is not GGN_MODE_walk:
      # TODO
      raise NotImplementedError("only `GGN_MODE_walk` implemented")

    # Update input parameters with blending
    inp.update_blending()
    y = max(inp.bodyYOffs.val +inp.bodyYShift.val, 0)
    v = inp.xyzBodyPos.val
    inp.xyzBodyPos.val = [v[0], y, v[2]]

    # Check if target angle is set and compass is on, if so, handle it
    if inp.tarAngle_deg != GGN_InvalidAngle and False:
      # TODO
      raise NotImplementedError("`tarAngle_deg` not yet implemented")
      '''
      if(abs(inputGGN.tarAngle_deg -EXB.compassDir) < cTurnDeadZone_deg) {
        // target angle and compass angle are virtually the same, thus reset
        // target angle and stop turning
        inputGGN.tarAngle_deg = GGN_InvalidAngle;
        inputGGN.travelRotY = 0;
        currFlags = (currFlags & ~HXA_flag_isTurnToTarget); }
      else {
        // Set flag to indicate that the target direction is not yet reached
        currFlags = (currFlags | HXA_flag_isTurnToTarget);
      }
      '''
    # Check if all legs are down and if not so, set them to their initial position
    self._allLegsDown = self._are_legs_down()
    if not self._allLegsDown:
      self._xyzLegPos = np.array(self._Cfg.FEET_INIT_XYZ)

    # Calculate update of gate sequence
    gait.isGaitInMotion = self._is_gait_in_motion()
    for iLeg in range(LEG_COUNT):
      gait._generate_leg_gait_seq(self, iLeg)

    # Do IK for all legs
    lgxyz = self._xyzLegPos
    boxyz = inp.xyzBodyPos
    gaxyz = gait.xyzPos
    for iLeg in range(LEG_COUNT):
      s = -1 if iLeg in R_LEGS else 1
      f1 = np.array([ s, 1, 1])
      f2 = np.array([-s, 1, 1])
      f3 = np.array([-s,-1,-1])
      #      right      left
      # f1 = [-1, 1, 1] [ 1, 1, 1]
      # f2 = [ 1, 1, 1] [-1, 1, 1]
      # f3 = [ 1,-1,-1] [-1,-1,-1]
      xyz1 = np.array(f1*lgxyz[iLeg] +f2*boxyz.val +gaxyz[iLeg])
      xyz2 = self._bodyIK(xyz1, gait.rotY[iLeg], iLeg)
      xyz3 = np.array(lgxyz[iLeg] +f1*boxyz.val +f3*xyz2 +f1*(gaxyz[iLeg]))
      self.lastIKSolRes += self._legIK(xyz3, iLeg)

    if self._verboseLevel > 0:
      self.log_IKM_result()

    # Drive servos, if GGN is on
    if inp.isOn:
      if not self.isOnPrev:
        # GGN was just switched on, do whatever is nescessary ...
        # TODO
        toLog("GGN was turned ON")

      # Calculate time the next move will take
      if self._is_gait_in_motion(zFactor=2):
        self.nextMoveDur_ms = gait.nomSpeed
        self.nextMoveDur_ms += inp.delayInput.val*2. +inp.delaySpeed_ms.val
        self.walkEngineState = WEState.Walking
      else:
        # Movement speed if not walking
        self.nextMoveDur_ms = 200 +inp.delaySpeed_ms.val
        self.walkEngineState = WEState.Ready

      # Send new leg positions to the servo manager
      self._startServoUpdate()
      t2_ms = ticks_ms()
      self.calcDur_ms =t2_ms -t1_ms

      if (self.tMoveStart_ms == 0 or
          (self.tMoveStart_ms +self.moveDur_ms) < t2_ms):
        # Is first move or next servo update is overdue, in any case execute
        # the move immediately
        self._execMove()
        self.lastErrC = ErrCode.ServoMoveOverdue
        if self._verboseLevel > 0:
          toLog("Servo move is overdue", err=self.lastErrC)
      else:
        # The last move is not yet finished, we need to wait ...
        # (for this the timer is used, so that other sections of the main
        #  loop get processing time)
        self.state = GGNState.IsMoving
        self.nextState = GGNState.UpdateServos
        dt = max(self.nextMoveDur_ms -self.calcDur_ms, 1)
        self.waitUntil_ms = int(ticks_ms() +dt)

    else:
      # Turn GGN off
      if self.isOnPrev or not self._allLegsDown:
        self.moveDur_ms = 600;
        self._startServoUpdate()
        self._execMove()
      else:
        toLog("GGN was turned OFF")

    self.isOnPrev = inp.isOn
    if self._verboseLevel > 0:
      self.log_status()

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def _execMove(self):
    """ Executes the move based on the servo angles in `self._legAngles`
        and updates the GGN status and timing parameters.
    """
    self._commitServoUpdate(self.moveDur_ms)
    t_ms = ticks_ms()
    self.lastMoveLate_ms = t_ms -self.tMoveStart_ms -self.lastMoveDur_ms
    self.tMoveStart_ms = t_ms
    self.lastMoveDur_ms = self.moveDur_ms
    self.moveDur_ms = self.nextMoveDur_ms
    self.state = GGNState.NeedsToCompute

  def _startServoUpdate(self):
    """ Translate joint angles and prepare servo update
    """
    self._servoAngles = np.array(self._legAngles)
    self._servoAngles[:,2] = -self._servoAngles[:,2]

  #@timed_function
  def _commitServoUpdate(self, dur_ms):
    """ Commit servo update
    """
    if self._SMan:
      # Move servos
      cfg = self._Cfg
      sang = self._servoAngles
      spos = np.zeros(cfg.SERVO_COUNT, dtype=np.int16)
      for iServo, (iLeg, iJoint) in enumerate(cfg.JOINT_BY_SERVO):
        spos[iServo] = int(sang[iLeg][iJoint])
      self._SMan.move(cfg.SERVO_IDS, spos, dur_ms, True)
    if self._verboseLevel > 1:
      print("Joint angles:")
      toLogArray(self._legAngles, digits=0)
      print("Servo angles:")
      toLogArray(self._servoAngles, digits=0)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  #@timed_function
  @micropython.native
  def _bodyIK(self, xyzFoot, yRot, iLeg):
    """ Calculates IK of the body
    """
    cfg = self._Cfg
    inp = self._Inp

    # Calculate total x,z distance between center of body and foot
    xyzTotal = np.array(cfg.COX_OFF_XYZ[iLeg] +xyzFoot)

    # Calculate sin and cos for each rotation
    r_y = np.array([0, yRot, 0])
    rot = (inp.xyzBodyRot.val +r_y) *math.pi /180
    sin_xyz = vector.sin(rot)
    cos_xyz = vector.cos(rot)

    # Calculate rotation matrix:
    x = xyzTotal[0] -( xyzTotal[0] *cos_xyz[1] *cos_xyz[2]
                      -xyzTotal[2] *cos_xyz[2] *sin_xyz[1]
                      +xyzFoot[1]  *sin_xyz[2])
    y = xyzFoot[1]  -( xyzTotal[0] *sin_xyz[1] *sin_xyz[0]
                      -xyzTotal[0] *cos_xyz[1] *cos_xyz[0] *sin_xyz[2]
                      +xyzTotal[2] *cos_xyz[1] *sin_xyz[0]
                      +xyzTotal[2] *cos_xyz[0] *sin_xyz[1] *sin_xyz[2]
                      +xyzFoot[1]  *cos_xyz[2] *cos_xyz[0])
    z = xyzTotal[2] -( xyzTotal[0] *cos_xyz[0] *sin_xyz[1]
                      +xyzTotal[0] *cos_xyz[1] *sin_xyz[2] *sin_xyz[0]
                      +xyzTotal[2] *cos_xyz[1] *cos_xyz[0]
                      -xyzTotal[2] *sin_xyz[1] *sin_xyz[2] *sin_xyz[0]
                      -xyzFoot[1]  *cos_xyz[2] *sin_xyz[0])
    return np.array([x, y, z])

  #@timed_function
  @micropython.native
  def _legIK(self, xyzFoot, iLeg):
    """ Calculates the angles of coxa, femur and tibia (`self._legAngles`)
        for the given foot position (`xyzFoot`) of leg (`iLeg`); returns
        information about IK solution quality. All intermediate angles are
        in radians.
    """
    cfg = self._Cfg
    legA = self._legAngles
    cLen = cfg.COX_LEN
    fLen = cfg.FEM_LEN
    tLen = cfg.TIB_LEN
    tCor = cfg.TIB_CORR_ANGLE

    try:
      # Calculate coxa angle
      # w/ `IKFootPosXZ` the length between coxa and foot
      a = xyzFoot[0]
      b = xyzFoot[2]
      IKFootPosXZ = math.sqrt(a**2 +b**2)
      IKCoxaAngle = math.atan2(b, a)
      legA[iLeg,COX] = math.degrees(IKCoxaAngle) +cfg.COX_OFF_ANG[iLeg]

      # Solving `IKA1`, `IKA2` and `IKSW`
      # w/ `IKA1` the angle between SW line and the ground in radians
      #    `IKSW` the distance between femur axis and tars)
      #    `IKA2` the angle between the line S>W with respect to the femur
      a = xyzFoot[1]
      b = IKFootPosXZ -cLen
      IKSW = math.sqrt(a**2 +b**2)
      IKA1 = math.atan2(b, a)
      tmp1 = fLen**2 -tLen**2 +IKSW**2
      tmp2 = 2*fLen *IKSW
      IKA2 = math.acos(tmp1/tmp2)

      # Calculate femur angle
      legA[iLeg,FEM] = -math.degrees(IKA1 +IKA2) +90

      # Calculate tibia angle
      tmp2 = 2 *fLen *tLen
      a_rad4 = math.acos(tmp1/tmp2)
      legA[iLeg,TIB] = (tCor -math.degrees(a_rad4))

    except ValueError:
      toLog("Domain error in `_legIK` for leg {0}".format(iLeg),
            err=ErrCode.IKM_Incomplete)

    # Return the solution quality
    if IKSW < (fLen +tLen -30):
      return GGN_IKM_Solution
    else:
      if IKSW < (fLen +tLen):
        return GGN_IKM_Warning
      else:
        return GGN_IKM_Error

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def Input(self):
    return self._Inp

  def _are_legs_down(self):
    """ Check if all legs are down
    """
    d = abs((self._xyzLegPos -self._Cfg.FEET_INIT_XYZ).flatten())
    if MICROPYTHON:
      return numerical.sum(d) == 0
    else:
      return np.sum(d) == 0

  def _is_gait_in_motion(self, zFactor=1):
    """ Returns True if one of the travel lengths are outside the range
        considered as the dead zone, that means basically in motion
    """
    dz = self._Cfg.TRAVEL_DEAD_ZONE
    xz = self._Inp.x_zTravelLen.val
    yr = self._Inp.travelRotY.val
    return abs(xz[0]) > dz or abs(xz[2]) > dz or abs(yr *zFactor) > dz

  def _is_gait_at_neutral(self, zFactor=2):
    """ Returns true if all travel lengths are within the range considered as
        the dead zone, that means basically at neutral
    """
    dz = self._Cfg.TRAVEL_DEAD_ZONE
    xz = self._Inp.x_zTravelLen.val
    yr = self._Inp.travelRotY.val
    return abs(xz[0]) < dz and abs(xz[2]) < dz and abs(yr *zFactor) < dz

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def log_IKM_result(self):
    """ Log result of IK computation
    """
    sRes = self.lastIKSolRes
    nErr = sRes // 100
    nWrn = (sRes % 100) //10
    nSol = sRes % 10
    self.lastErrC = ErrCode.Ok
    if nWrn > 0:
      self.lastErrC = ErrCode.IKM_Incomplete
    if nErr > 0:
      self.lastErrC = ErrCode.IKM_Failed
    if self._verboseLevel > 0 and not self.lastErrC == ErrCode.Ok:
      toLog("IKM: {0} error(s), {1} warnings, {2}, solutions"
            .format(nErr, nWrn, nSol), err=self.lastErrC)

  def log_status(self):
    """ Log status
    """
    states = [self.walkEngineState, self.state, self.lastErrC]
    if states != self._prevStates:
      self._prevStates = states
      toLog("HXA: `{0}`, GGN: `{1}`, error: {2}"
            .format(WEStateStr[states[0]], GGNStateStr[states[1]],
                    states[2]), err=self.lastErrC)

# ----------------------------------------------------------------------------
