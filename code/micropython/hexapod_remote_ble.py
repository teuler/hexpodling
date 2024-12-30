#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------
# hexapod_remote_ble.py
# Receive game controller commands via USB and sends these commands via
# BLE as messages to the hexapod
#
# The MIT License (MIT)
# Copyright (c) 2018-2020 Thomas Euler
# 2020-08-20, v1
#
# ---------------------------------------------------------------------
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import asyncio
import array
import numpy as np
import pygame
import modules.joystick as joy
from hexbotling.hexa_config_vorpal import HexaConfig
from hexbotling.server.hexa_input import HexaInput
from hexbotling.server.hexa_gait import HexaGait
from hexbotling.hexa_global import *
import hexbotling.robotling_lib.misc.rmsg as rmsg
from bleak import BleakClient
from bleak.uuids import uuid16_dict
from bleak.exc import BleakError

HEXAPOD_BLE_ADDRESS = "D8:A0:1D:46:DF:E2"

uuid16_dict         = {v: k for k, v in uuid16_dict.items()}
_DEVICE_NAME_UUID   = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
                      uuid16_dict.get("Device Name"))
_UART_TX            = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
_UART_RX            = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

# ---------------------------------------------------------------------
'''
def parseCmdLn():
  from argparse import ArgumentParser
  parser = ArgumentParser()
  parser.add_argument('-p', '--port', type=str, default=COM_PORT)
  return parser.parse_args()
'''
# ---------------------------------------------------------------------
def getCmdFromJoystickInput():
  """ Generate a new command based on the joystick input, if any
  """
  global isWalkMode, isCalib, bodyYOffsMode, ismCmd

  # Initialize hexapod input parameters
  tl_x_z = np.zeros(3)
  tr_y = 0
  ds = 0
  bp_x_z = np.zeros(3)
  br_xyz = np.zeros(3)
  bo_y = None
  bs_y = None
  lh = None
  params_changed = False
  mCmd.reset()

  # Button `A` switches between walk mode and body mode
  bA = JS.BtnA.pressed
  if bA is not None and bA:
    isWalkMode = not isWalkMode
    print("`A` -> {0} mode".format("Walk" if isWalkMode else "Body"))

  # Button `B` prints help
  bB = JS.BtnB.pressed
  if bB is not None and bB:
    print("`Start`            - Switched GGN on or off")
    print("`A`                - Switches between walk mode and body mode")
    print("`B`                - Prints this help screen")
    print("`X`                - Switches between changing `legLiftHeight` and `bodyYOffs`")
    print("`Y`                - Status requested")
    print("`Hat, L/R`         - -/+ `delaySpeed_ms`")
    print("`Hat, U/D`         - -/+ `legLiftHeight` or `bodyYOffs`")
    print("`Left stick,  L/R` - Move or walk sideways")
    print("`Left stick,  U/D` - Move or walk forward/backward")
    print("`Right stick, L/R` - Turn or tilt body left/right")
    print("`Right stick, U/D` - Nothing or tilt body forward/backward")

  bSL = JS.BtnStickR.pressed
  if bSL is not None and bSL:
    isCalib = not isCalib
    print("`Back` -> {0} calibration".format("Start" if isCalib else "End"))
    mCmd.token = rmsg.TOK_CAL
    mCmd.add_data("S", array.array("h", [int(isCalib)]))
    return mCmd

  # Button `X` switches between changing `legLiftHeight` and `bodyYOffs`
  bX = JS.BtnX.pressed
  if bX is not None and bX:
    bodyYOffsMode = not bodyYOffsMode
    print("`X` -> {0} can be set".format("bodyYOffs" if bodyYOffsMode else "legLiftHeight"))

  # Button `Y` requests the status
  bY = JS.BtnY.pressed
  if bY is not None and bY:
    mCmd.token = rmsg.TOK_STA
    print("`Y` -> Status requested")
    return mCmd

  # Button `Start` switched GGN on or off
  bStart = JS.BtnStart.pressed
  if bStart is not None and bStart:
    Inp.isOn = not Inp.isOn
    mCmd.token = rmsg.TOK_GG0
    mCmd.add_data("M", array.array("h", [int(Inp.isOn), 2]))
    mCmd.add_data("G", array.array("h", [0]))
    return mCmd

  # The left hat controls speed and leg offset
  hatL = JS.HatL.value
  if hatL is not None:
    ds = Inp.delaySpeed_ms.val
    ds += hatL[0] *10
    Inp.delaySpeed_ms.val = ds
    if bodyYOffsMode:
      bo = Inp.bodyYOffs.val
      bo += hatL[1] *5
      params_changed = Inp.bodyYOffs.val != bo
      Inp.bodyYOffs.val = bo
      s = "bodyYOffs={0}".format(bo)
    else:
      lh = Inp.legLiftHeight.val
      lh += hatL[1] *5
      params_changed = Inp.legLiftHeight.val != lh
      Inp.legLiftHeight.val = lh
      s = "legLiftHeight={0}".format(lh)
    print("{0}, delaySpeed_ms={1}".format(s, ds))

  if isWalkMode:
    # Control walking ...
    # >GGQ T=bo,lh,tx,tz,ty D=ds A=ta;
    # <STA ...;
    xyL = JS.StickL.xy
    xyR = JS.StickR.xy
    if xyL is not None or xyR is not None or params_changed:
      # Joystick input has changed
      if xyL is not None:
        tl_x_z[0] = -xyL[0] *Cfg.TRAVEL_X_Z_LIM[0]
        tl_x_z[2] = xyL[1] *Cfg.TRAVEL_X_Z_LIM[2]
      if xyR is not None:
        tr_y = xyR[0] *Cfg.TRAVEL_ROT_Y_LIM
        #ds = max(0, (1 -xyR[1]) *Cfg.DELAY_SPEED_MAX)
      Inp.set_pos_timing(bo_y, lh, tl_x_z, tr_y, None)
      mCmd.token = rmsg.TOK_GGQ
      temp = []
      temp.append(int(Inp.bodyYOffs.val))
      temp.append(int(Inp.legLiftHeight.val))
      temp.append(int(Inp.x_zTravelLen.x))
      temp.append(int(Inp.x_zTravelLen.z))
      temp.append(int(-Inp.travelRotY.val))
      mCmd.add_data("T", array.array("h", temp))
      mCmd.add_data("D", array.array("h", [int(Inp.delaySpeed_ms.val)]))
      mCmd.add_data("A", array.array("h", [-1]))
      return mCmd

  else:
    # Control body posture ...
    # >GGP B=bs,px,pz,bx,by,bz T=bo,lh,tx,tz,ty;
    xyL = JS.StickL.xy
    xyR = JS.StickR.xy
    if xyL is not None or xyR is not None or params_changed:
      # Joystick input has changed
      if xyL is not None:
        bp_x_z[0] = xyL[0] *Cfg.BODY_X_Z_POS_LIM[0]
        bp_x_z[2] = xyL[1] *Cfg.BODY_X_Z_POS_LIM[2]
      if xyR is not None:
        br_xyz[0] = xyR[1] *Cfg.BODY_XYZ_ROT_LIM[0]
        br_xyz[1] = xyR[0] *Cfg.BODY_XYZ_ROT_LIM[1]
      Inp.set_pos(bo_y, bs_y, bp_x_z, br_xyz, lh, [0,0,0], 0)
      mCmd.token = rmsg.TOK_GGP
      temp = []
      temp.append(int(Inp.bodyYShift.val))
      temp.append(int(Inp.xyzBodyPos.x))
      temp.append(int(Inp.xyzBodyPos.z))
      temp.append(int(Inp.xyzBodyRot.x))
      temp.append(int(Inp.xyzBodyRot.y))
      temp.append(int(Inp.xyzBodyRot.z))
      mCmd.add_data("B", array.array("h", temp))
      temp = []
      temp.append(int(Inp.bodyYOffs.val))
      temp.append(int(Inp.legLiftHeight.val))
      temp.append(int(Inp.x_zTravelLen.x))
      temp.append(int(Inp.x_zTravelLen.z))
      temp.append(int(Inp.travelRotY.val))
      mCmd.add_data("T", array.array("h", temp))
      return mCmd
  return mCmd

# ---------------------------------------------------------------------
def uart_data_received(sender, data):
  print("RX> {0}".format(data))

# ---------------------------------------------------------------------
async def run(address):
  """ Run the loop
  """
  global mCmd, mRpl, isReady

  try:
    # Try connecting to robot via BLE
    async with BleakClient(address) as client:
      res = await client.is_connected()
      print("Connected to {0}".format(address))

      # Start responding to incoming messages
      await client.start_notify(_UART_RX, uart_data_received)

      # Loop
      while await client.is_connected() and isReady:
        # Check for pygame events
        for ev in pygame.event.get():
          if (ev.type == pygame.QUIT or
              (ev.type == pygame.JOYBUTTONDOWN and ev.button == joy.BTN_BACK_ID)):
            # Exit programm
            isReady = False
            break

        if isReady:
          # Check for input from joystick
          mCmd = getCmdFromJoystickInput()
          if mCmd.token is not rmsg.TOK_NONE:
            # Send new command
            sCmd = mCmd.to_hex_string()
            print("-> ", mCmd)
            repl = ""
            await client.write_gatt_char(_UART_TX, (sCmd).encode())
            repl = await client.read_gatt_char(_UART_RX)
            if len(repl) > 0:
              mRpl.from_hex_string(repl[1:-1])
              print("<- ", mRpl)

        # Keep events running
        Clock.tick(5)
        await asyncio.sleep(0.010)

      await client.stop_notify(_UART_RX)
      print("ERROR: Connection lost")

  except BleakError as err:
    # Device not found
    print("ERROR: BLE connection failed ({0})".format(err))

# ---------------------------------------------------------------------
if __name__ == '__main__':

  # Initialize variables
  isConnected = False
  isReady = False

  # Import classes to represent hexapod input
  Cfg = HexaConfig()
  Gait = HexaGait()
  Cfg.MAX_BLEND_STEPS = 0
  Inp = HexaInput(Gait, Cfg)
  isWalkMode = False
  isCalib = False
  bodyYOffsMode = True
  mCmd = rmsg.RMsg()
  mRpl = rmsg.RMsg()

  '''
  # Check for command line parameter(s)
  args = parseCmdLn()
  '''
  # Access joystick ...
  try:
    JS = joy.Joystick(0)
    Clock = pygame.time.Clock()
    _ = JS.BtnA.pressed
    print("Joystick found.")
    isReady = True
  except AttributeError:
    print("ERROR: No joystick found")
    exit()

  # Run loop
  try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(HEXAPOD_BLE_ADDRESS))
  except TimeoutError:
    print("ERROR: Connection lost")

  # Disconnect
  print("... done.")

# ---------------------------------------------------------------------
