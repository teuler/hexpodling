# ----------------------------------------------------------------------------
# servo_manager.py
# Class to manage and control a number of servos
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-03, v1
# ----------------------------------------------------------------------------
import array
import time
from misc.helpers import timed_function

from platform.platform import platform
if (platform.ID == platform.ENV_ESP32_UPY or
    platform.ID == platform.ENV_ESP32_TINYPICO):
  import platform.esp32.dio as dio
  from machine import Timer
  from micropython import alloc_emergency_exception_buf
  alloc_emergency_exception_buf(100)
else:
  print("ERROR: No matching hardware libraries in `platform`.")

__version__     = "0.1.0.0"
RATE_MS         = const(20)
_SCALER         = const(16)
_STEP_ARRAY_MAX = const(500)

# ----------------------------------------------------------------------------
class ServoManager(object):
  """Class to manage and control a number of servos"""

  TYPE_NONE       = const(0)
  TYPE_HORIZONTAL = const(1)
  TYPE_VERTICAL   = const(2)
  TYPE_SENSOR     = const(3)

  def __init__(self, nChan, max_steps=_STEP_ARRAY_MAX, verbose=False):
    """ Initialises the management structures
    """
    # To avoid float calculations in the ISR, positions used during a move
    # are given as integers multiplied by a scaler: [us]*_SCALER
    self._isVerbose     = verbose
    self._isMoving      = False
    self._nChan         = max(1, nChan)
    self._Servos        = [None]*nChan                 # Servo objects
    self._servo_type    = array.array('b', [0]*nChan)  # Servo type
    self._servo_number  = array.array('i', [-1]*nChan) # Servo number
    self._servoPos      = array.array('i', [0]*nChan)  # Servo positions [us]
    self._SIDList       = array.array('i', [-1]*nChan) # Servo IDs to move next
    self._targetPosList = array.array('i', [0]*nChan)  # Target positions [us]*
    self._currPosList   = array.array('i', [-1]*nChan) # Current position [us]*
    self._stepSizeList  = array.array('i', [0]*nChan)  # .. step sizes [us]*
    self._stepLists     = []
    for i in range(nChan):
      self._stepLists.append(array.array('i', [0]*max_steps))
    self._nToMove       = 0                            # # of servos to move
    self._dt_ms         = 0                            # Time period [ms]
    self._nSteps        = 0                            # # of steps to move
    self._Timer         = Timer(0)
    self._Timer.init(period=-1)

  def add_servo(self, i, servoObj, pos=0):
    """ Add at the entry `i` of the servo list the servo object, which has to
        define the following functions:
        - `write_us(t_us)`
        - `angle_in_us(value=None)`
        - `off()`
        - `deinit()`
    """
    if i in range(self._nChan):
      self._Servos[i] = servoObj
      self._servoPos[i] = servoObj.angle_in_us()
      self._servo_number[i] = i
      if self._isVerbose:
        print("Add servo #{0:-2.0f}, at {1} us".format(i, self._servoPos[i]))

  def set_servo_type(self, i, type):
    """ Change servo type (see `TYPE_xxx`)
    """
    if i in range(self._nChan) and self._Servos[i] is not None:
      self._servo_type[i] = type

  def turn_all_off(self, deinit=False):
    """ Turn all servos off
    """
    for servo in self._Servos:
      if not servo is None:
        servo.off()
        if deinit:
          servo.deinit()

  def deinit(self):
    """ Clean up
    """
    self._Timer.deinit()
    self.turn_all_off(deinit=True)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @timed_function
  def move_timed(self, servos, pos, dt_ms=0, lin_vel=True):
    self.move(servos, pos, dt_ms)

  @micropython.native
  def move(self, servos, pos, dt_ms=0, lin_vel=True):
    """ Move the servos in the list to the positions given in `pos`.
        If `dt_ms` > 0, then it will be attempted that all servos reach the
        position at the same time (that is after `dt_ms` ms)
    """
    if self._isMoving:
      # Stop ongoing move
      # ...
      self._Timer.init(period=-1)
      self._isMoving = False

    # Prepare new move
    n = 0
    nSteps = dt_ms /RATE_MS
    if nSteps > _STEP_ARRAY_MAX:
      # Too many steps for a paraboloid trajectory
      lin_vel = True
      print("WARNING: {0} is too many steps; going linear".format(int(nSteps)))

    for iS, SID in enumerate(servos):
      if not self._Servos[SID]:
        continue
      self._SIDList[n] = SID
      self._targetPosList[n] = self._Servos[SID].angle_in_us(pos[iS]) *_SCALER
      if nSteps > 0:
        # A time period is given, therefore calculate the step sizes for this
        # servo's move, with ...
        p = self._servoPos[SID] *_SCALER
        dp = self._targetPosList[n] -p
        #print("dp=", dp)
        if lin_vel:
          # ... linear velocity
          s = int(dp /nSteps)
          self._currPosList[n] = p +s
          self._stepSizeList[n] = s
        else:
          # ... paraboloid trajectory
          p_n = nSteps +1
          p_n2 = p_n /2
          p_peak = p_n2**2
          p_func = [-(j +1 -p_n2)**2 +p_peak for j in range(int(nSteps))]
          p_scal = dp /sum(p_func)
          for iv in range(p_n -1):
            self._stepLists[n][iv] = int(p_func[iv] *p_scal)
          self._stepSizeList[n] = 0
          #print(dp, nSteps, p_n, p_scal, sum(self._stepLists[iS]))
          #print(self._stepLists[iS])
      else:
        # Move directly, therefore update already the final position
        self._servoPos[SID] = self._targetPosList[iS] //_SCALER
      n += 1
    self._nToMove = n
    self._dt_ms = dt_ms
    self._nSteps = int(nSteps) -1

    # Initiate move
    if dt_ms == 0:
      # Just move them w/o considering timing
      for iS in range(n):
        p = self._targetPosList[iS] //_SCALER
        self._Servos[self._SIDList[iS]].write_us(p)
      #print("now-targ", self._targetPosList)
    else:
      # Setup timer to keep moving them in the requested time
      self._Timer.init(mode=Timer.PERIODIC, period=RATE_MS, callback=self._cb)
      self._isMoving = True
      #print("timer-targ", self._targetPosList)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  def _cb(self, value):
    if self._isMoving:
      # Update every servo in the list
      for iS in range(self._nToMove):
        SID = self._SIDList[iS]
        if self._nSteps >= 1:
          # Move is ongoing, update servo position ...
          p = self._currPosList[iS] //_SCALER
          self._Servos[SID].write_us(p)
          if self._stepSizeList[iS] == 0:
            # Paraboloid trajectory
            self._currPosList[iS] += self._stepLists[iS][self._nSteps]
            #print("para", self._nSteps, p, self._stepLists[iS][self._nSteps])
          else:
            # Linear trajectory
            self._currPosList[iS] += self._stepSizeList[iS]
            #print("lin ", self._nSteps, p, self._stepSizeList[iS])
        else:
          # Move has ended, therefore set servo to the target position
          tp = self._targetPosList[iS] //_SCALER
          self._servoPos[SID] = tp
          self._Servos[SID].write_us(tp)
      if self._nSteps > 0:
        self._nSteps -= 1
        #print("curr", self._currPosList)
      else:
        # Move is done
        self._isMoving = False
        #print("targ", self._targetPosList)

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  @property
  def is_moving(self):
    """ Returns True if a move is still ongoing
    """
    return self._isMoving

# ----------------------------------------------------------------------------
