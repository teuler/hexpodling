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
  global isWalkMode, mCmd

  # Initialize hexapod input parameters
  tl_x_z = np.zeros(3)
  tr_y = 0
  ds = 0
  bp_x_z = np.zeros(3)
  br_xyz = np.zeros(3)
  bo_y = None
  bs_y = None
  lh = None
  mCmd.reset()

  # Button `A` switches between walk mode and body mode
  bA = JS.BtnA.pressed
  if bA is not None and bA:
    isWalkMode = not isWalkMode
    print("`A` -> {0} mode".format("Walk" if isWalkMode else "Body"))

  # Button `Y` request the status
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
    mCmd.add_data("M", [int(Inp.isOn), 2])
    mCmd.add_data("G", [0])
    return mCmd

  # The left hat controls the speed and leg lift
  hatL = JS.HatL.value
  if hatL is not None:
    ds = Inp.delaySpeed_ms.val
    ds += hatL[1] *10
    Inp.delaySpeed_ms.val = ds
    lh = Inp.legLiftHeight.val
    lh += hatL[0] *5
    Inp.legLiftHeight.val = lh

  if isWalkMode:
    # Control walking ...
    # >GGQ T=bo,lh,tx,tz,ty D=ds A=ta;
    # <STA ...;
    xyL = JS.StickL.xy
    xyR = JS.StickR.xy
    if xyL is not None or xyR is not None:
      # Joystick input has changed
      if xyL is not None:
        tl_x_z[0] = xyL[0] *Cfg.TRAVEL_X_Z_LIM[0]
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
      mCmd.add_data("T", temp)
      mCmd.add_data("D", [int(Inp.delaySpeed_ms.val)])
      mCmd.add_data("A", [-1])
      return mCmd

  else:
    # Control body posture ...
    # >GGP B=bs,px,pz,bx,by,bz T=bo,lh,tx,tz,ty;
    xyL = JS.StickL.xy
    xyR = JS.StickR.xy
    if xyL is not None or xyR is not None:
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
      mCmd.add_data("B", temp)
      temp = []
      temp.append(int(Inp.bodyYOffs.val))
      temp.append(int(Inp.legLiftHeight.val))
      temp.append(int(Inp.x_zTravelLen.x))
      temp.append(int(Inp.x_zTravelLen.z))
      temp.append(int(Inp.travelRotY.val))
      mCmd.add_data("T", temp)
      return mCmd
  return mCmd

# ---------------------------------------------------------------------
def uart_data_received(sender, data):
  print("RX> {0}".format(data))

# ---------------------------------------------------------------------
async def run(address, loop):
  """ Run the loop
  """
  global mCmd, isReady

  try:
    # Try connecting to robot via BLE
    async with BleakClient(address, loop=loop) as client:
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
            sCmd = mCmd.to_string()
            print("-> ", sCmd)
            await client.write_gatt_char(_UART_TX, (sCmd +"\n").encode())
            repl = await client.read_gatt_char(_UART_RX)
            if len(repl) > 0:
              print("<- ", repl)

        # Keep events running
        Clock.tick(15)
        await asyncio.sleep(0.010, loop=loop)

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
  mCmd = rmsg.RMsg()

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
  loop = asyncio.get_event_loop()
  loop.run_until_complete(run(HEXAPOD_BLE_ADDRESS, loop))

  # Disconnect
  print("... done.")

# ---------------------------------------------------------------------
