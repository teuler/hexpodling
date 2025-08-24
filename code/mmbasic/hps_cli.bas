' hexapod-client v0.2.04
' ----------------------
' Runs the robot.
' Controls the server (`hps_srv.bas`) via a serial connection.
'
' Notes:
' - To connect wireless x-box-like gamepad, run `gamepad monitor`
'   and make sure that the gamepad is in `U` mode
'
' Copyright (c) 2023-25 Thomas Euler
' MIT Licence
'
' v0.1.11 (2023-12-31)
' - communication to server with basic movement commands
' - support for TOF sensor
' v0.1.12
' - fixed bug in server -> client communication
' v0.1.20
' - body shift and rotation implemented
' v0.1.21
' - added support for gamepad (PS4)
' v0.1.23
' - CMD_SHOW for defined postures and beep sounds implemented
' - gamepad control further completed
' v0.2.00
' - Added the option to use I2C for server-client communication, to free one
'   hardware UART (e.g., for the TOF sensor)
' v0.2.01
' - Small changes to account for Gamepad commands in MMBasic 6.00.xx
' v0.2.03
' - Remove older TOF parts since the sensor will be ESP32-based
' - Prepare for ESP32-Cam sensor
' - `ctrlByKeyboard` routine moved to library
' - "Window" in terminal to display status
' v0.2.04
' - Remove server-client via COM
'
' All boards/firmware versions:
'   OPTION COLORCODE ON
'   OPTION DISPLAY 64,80
'   OPTION CPUSPEED 252000
'   OPTION HEARTBEAT OFF
'
' Options for on-board Pico W (w/ round ST7789 display):
' a) for WebMite firmware
'   OPTION SYSTEM SPI GP18,GP19,GP16
'   OPTION LCDPANEL ST7789_320, P, GP22,GP15,GP17,GP14
'   OPTION WIFI "Elchland3", "xxx"
' b) for USB firmware
'   OPTION SYSTEM SPI GP18,GP19,GP16
'   OPTION LCDPANEL ST7789_320, P, GP22,GP15,GP17,GP14
'   OPTION PICO OFF
'   OPTION BAUDRATE 921600
'   OPTION SERIAL CONSOLE COM2,GP8,GP9
'   OPTION KEYBOARD GR
'
' Robot's coordinate system:
'   x-axis points left
'   y-axis points down
'   z-axis points backwards
'
' Hardware (Client):
' - 240x240 ST7789 display
'[ COM2 @ GP9,GP8 as #1 (to RP2040 w/ VL53L5CX TOF sensor)] - TO BE CHANGED
' - Client-server:
'   (a) COM1 (tx=GP0, rx=GP1) as device #2 -> To connect to server
'   (b) I2C (sda=GP0, scl=GP1) as master, with server being device &H42
' - handshake GP2->GP7 (server), GP3<-GP8 (server)
'
' Requires libraries:
' - `lib_keyboard.bas` (`ctrlByKeyboard` routine)
'
' ---------------------------------------------------------------------------
Option Base 0
Option Explicit
Option Escape
Option Default Float
Option Heartbeat Off

Sub MM.Startup
  Print "`" +MM.Info(Platform) +"` running on " +MM.DEVICE$;
  Print "MMBasic v" +Str$(MM.Info(Version))
  Gamepad monitor
End Sub

' ---------------------------------------------------------------------------
' Initialization of global definitions
GoTo Start
Main:
  ' Debug options:
  ' (DEB_VERB, DEB_INP, DEB_SIM, DEB_COM)
  DEBUG = 0

  ' Configuration
  Dim integer USE_LCD      = 0
  Dim integer USE_WIFI     = 0
  Dim integer USE_GPAD     = 1
  Dim integer USE_CAM      = 0
  Dim integer USE_TERMINAL = 1

  ' Control variables
  Dim integer i, j, ch, running = 1, i_loop = 0
  Dim _t = Timer, t_loop, t_state
  Dim t_info = _t, t_lcd = _t, t_hs = _t

  ' Input representation
  Dim in.bo_y, in.lh, in.ps_x, in.ps_z, in.pr_x, in.pr_y, in.pr_z
  Dim in.tl_x, in.tl_z, in.tr_y, in.ds
  Dim integer in.cmd, in.postureID, in.beepID
  R.resetInputBody

  ' Initialize
  R.init

  ' Start camera/ToF sensor
  /*
  If USE_CAM Then ...
  */

  ' Main loop
  ' ---------
  Do While running
    t_loop = Timer

    ' Handshake w/ server
    If t_hs +750 < t_loop Then
      R.triggerHandshake
      R.updateHandshake
      t_hs = Timer
    EndIf

    ' Read poti (selector)
    R.selector = R.Poti()

    ' Update camera/TOF sensor data
    /*
    If tof.newData Then
      Print tof.iFr
      For i=0 To TOF_FR_DX-1
        For j=0 To TOF_FR_DY-1
          Print Str$(tof.fr(j,i),4);
        Next
        Print
      Next
      Print
      tof.newData = 0
    EndIf
    */

    ' Check for/request server message and handle it ...
    ' (With I2C request state at given intervals if connected)
    If R.connected Then
      If t_state +1000 < t_loop Then ' 250
        COM.handleStateMsg
        t_state = Timer
      EndIf
    EndIf

    ' Handle keyboard input, if any
    ' End program on ESC, send command to server, if one was generated
    If USE_GPAD Then
      ch = ctrlByGamepad()
    Else
      ch = ctrlByKeyboard()
    EndIf
    If ch = 27 Then running = 0 : Exit : EndIf
    If in.cmd > 0 Then COM.sendMsg in.cmd

    ' Reporting ...
    If t_lcd +100 < t_loop Then
      If USE_LCD Then LCD.update
      If i_loop > 1500 And USE_TERMINAL Then Terminal.update
      t_lcd = Timer
    EndIf

    Inc i_loop, 1
  Loop

  ' End program
  Pin(PIN_HS_OUT) = 0
  R.running = 1 : COM.sendMsg CMD_POWER
  R.Shutdown
  End


' ---------------------------------------------------------------------------
Sub _printLn y%, txt$(), state%(), empty%
  ' Print a line at y% in the terminal window, using the text fields
  ' in txt$() and color by state()
  ' (4 fields with 18 characters each)
  Static integer first = 1
  Static string VT$(5) length 14
  If empty% Then Print @(0, y% *12) Space$(80) : Exit Sub : EndIf
  If first Then
    VT$(0) = Chr$(27) +"[m"                     ' Normal
    VT$(1) = Chr$(27) +"[42m" +Chr$(27) +"[97m" ' Ok
    VT$(2) = Chr$(27) +"[43m" +Chr$(27) +"[30m" ' Warning
    VT$(3) = Chr$(27) +"[41m" +Chr$(27) +"[30m" ' Danger
    VT$(4) = Chr$(27) +"[37m"                   ' Disabled
    VT$(5) = Chr$(27) +"[100m"+Chr$(27) +"[97m" ' Active
    first = 0
  EndIf
  Local string s$ = VT$(0)
  Local integer i, n
  For i=0 To 3
    n = 18 -Len(txt$(i))
    Cat s$, VT$(state%(i)) +"|" +txt$(i) +Space$(n) +VT$(0) +" "
  Next
  Print @(0, y% *12) +s$ +VT$(0) +"|"
End Sub


Sub Terminal.update
  ' Prints key data on the robot's state into the terminal (GUI-like)
  Local string tx$(3) length 18
  Local integer sta(3), i, j
  Local float v

  sta(0) = 0
  tx$(0) = "'" +Field$(SEL_STATE$, R.selector +1) +"'"
  sta(1) = 4
  tx$(1) = ""
  sta(2) = Choice(R.connected, 5, 4)
  tx$(2) = Choice(R.connected, "Connected", "Disconnected")
  sta(3) = 5
  tx$(3) = "GGN `" +Field$(GGN_STATE$, R.ggn_state +1) +"`"
  _printLn 0, tx$(), sta()

  ' Battery, IMU etc.
  v = R.servoBattery_V
  sta(0) = Choice(v < 7.5, Choice(v < 7.0, 3, 2), 1)
  tx$(0) = "Batt/Servo = " +Str$(v, 1, 1)+"V"
  v = R.logicBattery_V
  sta(1) = Choice(v < 3.7, Choice(v < 3.6, 3, 2), 1)
  tx$(1) = "Batt/Logic = " +Str$(v, 1, 1)+"V"
  sta(2) = 4
  tx$(2) = ""
  sta(3) = 5
  tx$(3) = "IMU h" +Str$(R.IMU(0),3,0) +" p" +Str$(R.IMU(1),3,0)
  Cat tx$(3), " r" +Str$(R.IMU(2),3,0)
  _printLn 1, tx$(), sta()

  ' Servo load
  For i=0 To SRV_N_LOAD_CH -1
    v = R.load(i)
    j = Min(Max(Int(v /10), 0), 10)
    sta(i) = Choice(v < 30, 2, Choice(v >= 200, 3, 1))
    tx$(i) = Str$(v,4,0) +String$(j, "#")
  Next
  _printLn 2, tx$(), sta()

  _printLn 3, tx$(), sta(), 1
End Sub

' ===========================================================================
' Handling gamepad input
' ---------------------------------------------------------------------------
Function ctrlByGamepad()
  ' Check for gamepad input and handle if, if possible. Returns
  ' key code or 0, if no key was pressed
  Local string key$
  Local integer changed(6), centred(6), i, v1
  Local di
  Static integer _data(6), _mode(1) = (-1,-1)
  If _mode(0) < 0 Then
    Math Set 0, _mode()
  EndIf
  in.cmd = 0
  ctrlByGamepad = 0

  key$ = Inkey$
  If Len(key$) > 0 Then
    ' Return keyboard input
    ctrlByGamepad = Asc(LCase$(key$))
    Exit Function
  EndIf
  If updateGamepad() Then
    ' New gamepad data ...
    If gpad.data(6) And &H0002 Then 'Start/Options
      ' Toggle power up/down
      R.resetInputBody
      R.running = Choice(R.running = 2, 1, 2)
      in.cmd = CMD_POWER
      Exit Function
    EndIf

    If gpad.data(6) And &H0011 Then 'L1 or R1
      ' Trigger posture #1 or #2
      in.cmd = CMD_SHOW
      in.postureID = Choice(gpad.data(6) And &H10, 1, 2)
      Exit Function
    EndIf
    If gpad.data(6) And &H0004 Then 'PS
      in.postureID = 0
      in.beepID = 1
      in.cmd = CMD_SHOW
      Exit Function
    EndIf
    If gpad.data(6) And &H0800 Then 'B/Circle
      ' Toggle move<->posture
      _mode(0) = Choice(_mode(0) = 0, 1, 0)
      Print "Mode -> `"+Field$("Move,Posture", _mode(0)+1, ",")+"`"
    EndIf
    If gpad.data(6) And &H0140 Then 'LL/R
      ' Toggle setting
      Inc _mode(1), Choice(gpad.data(6) And &H40, 1, -1)
      If _mode(1) < 0 Then
        _mode(1) = 2
      ElseIf _mode(1) > 2 Then
        _mode(1) = 0
      EndIf
      Print "Setting -> `"+Field$("Delay,LegLift,BodyY", _mode(1)+1, ",")+"`"
    EndIf

    If gpad.data(6) And &H00A0 Then 'LUp/Down
      Select Case _mode(1)
        Case  0 ' Increase/decrease delay speed
          Inc in.ds, Choice(gpad.data(6) And &H80, -10, 10)
          in.ds = Min(Max(in.ds, 0), DELAY_SPEED_MAX)
          in.cmd = CMD_MOVE
        Case 1 ' leg lift height
           Inc in.lh, Choice(gpad.data(6) And &H80, 5, -5)
           in.lh = Min(Max(in.lh, LEG_LIFT_MIN), LEG_LIFT_MAX)
           in.cmd = CMD_BODY
        Case 2 ' body offset
          Inc in.bo_y, Choice(gpad.data(6) And &H80, 5, -5)
          in.bo_y = Min(Max(in.bo_y, 0), BODY_Y_OFFS_MAX)
          in.cmd = CMD_BODY
      End Select
    EndIf

    If gpad.data(6) And &H2000 Then 'A/Cross
      ' Reset input
      R.resetInputBody
      in.cmd = CMD_BODY
    EndIf
    If gpad.data(6) And &H0400 Then 'Y/Triangle
      ' Log state info
      R.logStateInfo
    EndIf

    If R.running = 2 Then
      ' Check if joystick data has changed
      For i=0 To 5
        changed(i) = Abs(gpad.data(i) -_data(i)) > 2
        centred(i) = Abs(gpad.data(i) -127) <= 5
      Next

      If _mode(0) = 0 Then
        ' `_mode(0)`: 0=normal movements, 1=move/rotate body
        If changed(0) Then 'LX changed
          ' x travel length
          di = -TRAVEL_X_Z_LIM(0) /127
          in.tl_x = Choice(centred(0), 0,Int((gpad.data(0)-127) *di))
          in.cmd = CMD_MOVE
        EndIf
        If changed(1) Then 'LY changed
          ' z travel length
          di = TRAVEL_X_Z_LIM(2) /127
          in.tl_z = Choice(centred(1), 0,Int((gpad.data(1)-127) *di))
          in.cmd = CMD_MOVE
        EndIf
        If changed(2) Then 'RX changed
          ' y travel rotation
          di = -TRAV_ROT_Y_LIM /127
          in.tr_y = Choice(centred(2), 0,Int((gpad.data(2)-127) *di))
          in.cmd = CMD_MOVE
        EndIf
        ' ...

      Else
        If changed(0) Then 'LX changed
          ' x body (lean sideways)
          di = BODY_X_Z_POS_LIM(0) /127
          in.ps_x = Choice(centred(0), 0,Int((gpad.data(0)-127) *di))
          in.cmd = CMD_BODY
        EndIf
        If changed(1) Then 'LY changed
          ' z body (lean forward/backward)
          di = -BODY_X_Z_POS_LIM(2) /127
          in.ps_z = Choice(centred(1), 0,Int((gpad.data(1)-127) *di))
          in.cmd = CMD_BODY
        EndIf
        If changed(2) Then 'RX changed
          ' y body (rotate) -> yaw
          di = -BODY_XYZ_ROT_LIM(1) /127
          in.pr_y = Choice(centred(2), 0,Int((gpad.data(2)-127) *di))
          in.cmd = CMD_BODY
        EndIf
        If changed(3) Then 'RY changed
          ' x body (rotate) -> roll
          di = -BODY_XYZ_ROT_LIM(0) /127
          in.pr_x = Choice(centred(3), 0,Int((gpad.data(3)-127) *di))
          in.cmd = CMD_BODY
        EndIf
      EndIf
    EndIf
    Math Scale gpad.data(), 1, _data()
  EndIf
  ctrlByGamepad = ch
End Function

' ===========================================================================
' Handling keyboard input
' ---------------------------------------------------------------------------
' -> Function `ctrlByKeyboard()` now in `lib_keyboard.bas`

' ===========================================================================
' Start of robot's definitions
' ---------------------------------------------------------------------------
Start:
  ' Set data read pointer here
  StartR: Restore StartR
  Print

  Const R.VERSION$       = "0.2.04"
  Const R.NAME$          = "hexapod|client"
  Option Platform R.Name$

  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' GLOBAL DEFINITIONS
  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
 'Const DEB_SRV          = &H01
  Const DEB_VERB         = &H02
 'Const DEB_GAIT         = &H04
  Const DEB_INP          = &H08
 'Const DEB_IK           = &H10
  Const DEB_COM          = &H40
  Const DEB_SLNT         = &H80
  Dim integer DEBUG      = 0

  ' Gait generator (GGN) states
  Const GGN_IDLE         = 0
  Const GGN_COMPUTE      = 1
  Const GGN_STAND_BY     = 2
  Const GGN_MOVING       = 3
  Const GGN_DO_SERVOS    = 4
  Const GGN_STATE$       = "Idle,Compute,Standby,Moving,DoServos"

  ' Error codes
  Const ERR_OK           = 0
  Const ERR_MOVE_OVERDUE = 1
  Const ERR_STR$         = "Ok,MoveOverdue"

  ' Leg-related definitions
  ' -----------------------
  Const LEG_N            = 6

  Const COX              = 0
  Const FEM              = 1
  Const TIB              = 2

  Const COX_LEN          = 48
  Const FEM_LEN          = 47
  Const TIB_LEN          = 84

  Const FEM2TIB2_DIF     = FEM_LEN^2 -TIB_LEN^2
  Const FEM_2LEN         = FEM_LEN *2
  Const FEMTIB_2LEN      = FEM_LEN *TIB_LEN *2

  Const SRV_N_LOAD_CH    = 4

  ' Body-related definitions
  ' ------------------------
  Const TIB_CORR_ANG     = 50
  Const BODY_R           = 60      ' Body radius [mm]
  Const BODY_COX_ANG     = 60      ' Fixed leg offset angle around body

  Dim integer COX_OFF_ANG(LEG_N-1) ' Leg angles (in degree)
  Data  0, 60,120,180,240,300
  Read COX_OFF_ANG()

  Dim COX_OFF_XYZ(LEG_N-1, 2)      ' Leg positions relative to body
  R._calcLegPos BODY_COX_ANG, BODY_R, COX_OFF_XYZ()

  ' Movement-related definitions
  ' ----------------------------
  ' Limits within movements are considered finished
  Const TRAVEL_DEAD_Z    = 2
 'Const TURN_DEAD_Z      = 5
  Const LEGS_DOWN_DEAD_Z = 10      ' TODO

  ' Limits of movement control parameters
  ' (Rotations are in [degree], travel in [mm],
  '  and  delays in [ms])
  Const BODY_Y_OFFS_MIN  = 0
  Const BODY_Y_OFFS_MAX  = 70
  Const BODY_Y_SHFT_LIM  = 64

  Dim integer BODY_X_Z_POS_LIM(2)
  Data 15, BODY_Y_OFFS_MAX +BODY_Y_SHFT_LIM, 15
  Read BODY_X_Z_POS_LIM()

  Dim integer BODY_XYZ_ROT_LIM(2) = (8, 20, 8)
  Dim integer TRAVEL_X_Z_LIM(2) = (34, 0, 34) ' was 40,0,40

  Const TRAV_ROT_Y_LIM   = 25
  Const LEG_LIFT_MIN     = 20      ' 40
  Const LEG_LIFT_MAX     = 60      ' 80
  Const DELAY_INPUT_MIN  = 0
  Const DELAY_INPUT_MAX  = 255
  Const DELAY_SPEED_MAX  = 2000

  ' Start position of feet
  ' (Measured from beginning of coxa; leg coordinate system (?))
  Const FEM_STRT_ANG     = -20
  Const TIB_STRT_ANG     = 10
  Const COX_OFFS_FACT    = 1.0
  Dim FEET_INIT_XYZ(LEG_N-1, 2)
  R._calcFootPos FEM_STRT_ANG, TIB_STRT_ANG, BODY_COX_ANG, FEET_INIT_XYZ()

  ' Client-server communication
  ' ---------------------------
  ' Both messages start with:
  '   Byte(s)  0...7 : time since start in [ms] (uint32, hexlified)
  '                8 : command code
  '                9 : n data bytes (following)
  ' Client->server continues with:
  '               10 : bodyYOffs
  '               11 : legLiftH
  '            12,13 : x_zTravL; current travel length X,Z
  '               14 : travRotY; current travel rotation Y
  '               15 : delaySpeed; adjustible delay [ms/10]
  '               16 : buzzer signal (0=none, 1=beep, ...)
  '               17 : command extension (e.g., power on or off)
  '            18,19 : x_zBody
  '            20-22 : xyzBodyRot
  '               23 : unused
  ' Server->Client continues with:
  '               10 : servo battery voltage [V*10]
  '               11 : logic battery voltage [V*10]
  '            12,13 : compass heading [degree] =b(12)*10 +b(13)
  '            14,15 : compass pitch and rol [-90,,90, degree]
  '               16 : GGN state
  '          17...24 : servo load for servos #0..7, 0..240 [a.u.]
  '                    WARNING - 1 more than currently reserved
  ' - All single-byte values +COM_DATA_OFFS to avoid that escape characters
  '   (e.g., `\r`) appear in the data section of a package.
  ' - For parameter limits, see `Movement-related definitions`
  '
  Const COM_DATA_OFFS    = 14
  Const COM_N_BYTEVAL    = 16   ' number of single-byte values
  Const COM_MSG_LEN      = 24   ' total length in bytes
  Const COM_I2C_ADDR     = &H42 ' Server address if I2C is used
  Const COM_I2C_SPEED    = 400
  Const COM_I2C_TOUT     = 200  ' ms

  Const CMD_BODY         = 1    ' Commands/message types
  Const CMD_MOVE         = 2
  Const CMD_STOP         = 3
  Const CMD_STATE        = 4
  Const CMD_POWER        = 5
  Const CMD_SHOW         = 6
  Const CMD_STR$         = "Body,Move,Stop,State,Power,Show"

  Const RES_NEW_MSG      = &H01 ' Message handling
  Const RES_HANDLED      = &H02
  Const RES_ERROR        = &H04

  Const HS_TOUT_MS       = 3000 ' Handshake time-out

  Dim integer com.isReady = 0, com.in_res = 0, com.in_t_ms
  Dim integer com.in(COM_N_BYTEVAL -1)

  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' CLIENT-SPECIFIC DEFINITIONS
  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' Pico pins
  Const PIN_POTI         = MM.Info(PinNo GP28)
  /*
  Const PIN_TOF_RX       = MM.Info(PinNo GP9) ' COM2 as #1
  Const PIN_TOF_TX       = MM.Info(PinNo GP8)
  */
  Const PIN_HS_IN        = MM.Info(PinNo GP3)
  Const PIN_HS_OUT       = MM.Info(PinNo GP2)

  /*
  ' TOF sensor definitions
  ' Data format (74 bytes):
  '   bytes  0..7 : frame index (uint32, hexlified)
  '           8,9 : dx, dy (uint8, binary) +DATA_OFFS
  '        10..73 : pixels as distance in [cm] +DATA_OFFS (for 8x8)
  '         74,75 : `\r\n`, which are remmoved by `Line Input`
  '   with DATA_OFFS = 14 to avoid that escape characters (e.g., `\r`)
  '   appear in the data section of a package.
  ' Pixel values of:
  '   255 : invalid measurement
  '
  Const TOF_BAUD         = 115200
  Const TOF_FR_DX        = 8
  Const TOF_FR_DY        = 8
  Const TOF_N_FR         = TOF_FR_DX *TOF_FR_DY
  Const TOF_N_DATA       = 10 +TOF_N_FR
  Const TOF_DATA_OFFS    = 14
  Const TOF_TILT_DEG     = 45

  ' TOF sensor-related variables and setup
  Dim integer tof.dx, tof.dy, tof.len, tof.iFr
  Dim integer tof.fr(TOF_FR_DX-1, TOF_FR_DY-1)
  Dim integer tof.isReady = 0, tof.newData = 0
  */

  ' Poti (selector) related definitions
  Const SEL_NONE         = 0
  Const SEL_STOP         = 1
  Const SEL_ADJUST       = 2
  Const SEL_TEST         = 3
  Const SEL_DEMO         = 4
  Const SEL_RC           = 5
  Const SEL_STATE$       = "None,Stop,Adjust,Test,Demo,RC"

  ' LCD related
  Const C_BKG            = RGB(black)
  Const C_TXT            = RGB(white)
  Const C_WARN           = RGB(orange)
  Const C_OK             = RGB(green)
  Const LCD_N            = 4

  ' Robot's state
  Dim R.servoBattery_V = 0, R.logicBattery_V = 0
  Dim integer R.IMU(2), R.ggn_state = GGN_IDLE, R.selector = SEL_NONE
  Dim integer R.running = 1, R.WiFi = 0, R.connected = 0
  Dim integer R.load(SRV_N_LOAD_CH -1)
  Array Set 0, R.load()

  ' Jump to main
  GoTo Main

' ===========================================================================
' Initialization, shutdown, and status
' ---------------------------------------------------------------------------
Sub R.Init
  ' Prepare handware
  InitR: Restore InitR
  Print R.NAME$+" v"+R.VERSION$
  Print "| running on "+MM.DEVICE$+" MMBasic v"+Str$(MM.Info(VERSION))
  Print "Initializing onboard hardware ..."
  Print "| CPU @ "+Str$(Val(MM.Info(CPUSPEED))/1E6)+" MHz"

  ' If USB version of firmware, check for gamepad ...
  If Instr(MM.DEVICE$, "USB") Then
    ' Gamepad-related variables
    Dim integer gpad.ch = 0, gpad.type = 0, gpad.changed = 0
    Dim integer gpad.data(6), gpad._intDetect = 0
    Print "| Gamepad support enabled. Checking for gamepad ..."
    initGamePad 131 ' 128
    USE_GPAD = gpad.ch > 0
  EndIf

  ' Setup Poti
  SetPin PIN_POTI, AIN

  ' Check for WiFi, if requested
  If USE_WIFI Then R.WiFi = isWiFiOk(1)
  If R.WiFi Then
    Print "| WiFi connected to "+MM.Info(IP Address)
  Else
    Print "| No WiFi"
  EndIf

  ' Setup hardware handshake
  Dim hs.t_last_ms = 0
  SetPin PIN_HS_IN, INTH, R._cbHandshake, PULLDOWN
  SetPin PIN_HS_OUT, DOUT : Pin(PIN_HS_OUT) = 0

  ' Open serial connections server; setup display
  COM.Open
  LCD.power 1

  If DEBUG And DEB_VERB Then
    ' ...
  EndIf
  Print "Ready."
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.Shutdown
  ' Shutdown hardware
  Print "Shutting down ..."
  COM.Close
  LCD.power 0
  Print "Done."
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.logStateInfo
  ' Prints the robot's state info
  Print "State:"
  ' R.IMU(2), R.ggn_state
  Print "| Batteries: servo="+Str$(R.ServoBattery_V,1,2)+"V ";
  Print "logic="+Str$(R.LogicBattery_V,1,2)+"V"
 'Print "| GGN state='";Field$(GGN_STATE$, R.ggn_state +1)+"' ";
  Print "| Selector : '"+Field$(SEL_STATE$, R.selector +1)+"'"
  Print "| IMU      : head="+Str$(R.IMU(0))+" pitch="+Str$(R.IMU(1))+" ";
  Print "roll="+Str$(R.IMU(2))
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._cbHandshake
  ' Interrupt when handshake in-pin goes high (= server signalled)
  hs.t_last_ms = Timer
End Sub


Sub R.triggerHandshake
  ' Initiate handshake with server
  Pulse PIN_HS_OUT, 0.5
End Sub


Sub R.updateHandshake
  ' Sets `R.connected` if last successful handshake was less than
  ' `HS_TOUT_MS` ago
  R.connected = (Timer -hs.t_last_ms) < HS_TOUT_MS
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Function R.Poti()
  ' Return Poti setting
  Local v = Pin(PIN_POTI)
  R.Poti = SEL_NONE
  If v < 0.05 Then R.Poti = SEL_STOP : Exit Function
  If v < 0.40 Then R.Poti = SEL_ADJUST : Exit Function
  If v < 1.00 Then R.Poti = SEL_TEST : Exit Function
  If v < 2.00 Then R.Poti = SEL_DEMO : Exit Function
  If v > 3.10 Then R.Poti = SEL_RC : Exit Function
End Function

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._calcLegPos ac, br, xyz()
  ' Calculate leg positions relative to body
  Local integer i
  Math Set 0, xyz()
  For i=0 To LEG_N-1
    xyz(i,0) = +Cos(i*Rad(ac)) *br
    xyz(i,2) = -Sin(i*Rad(ac)) *br
  Next
End Sub


Sub R._calcFootPos af, at, ac, xyz()
  ' Calculate starting positions for feet (from angles)
  Local float rf = Rad(af), rt = Rad(at), rc = Rad(ac), rx
  Local integer i
  Array Set 0, xyz()
  For i=0 To LEG_N-1
    rx = FEM_LEN *Cos(rf) -TIB_LEN *Sin(rt +rf) +COX_LEN*COX_OFFS_FACT
    xyz(i,0) = rx *Cos(rc *i)
    xyz(i,1) = FEM_LEN *Sin(rf) +TIB_LEN *Cos(rt -rf)
    xyz(i,2) = -rx *Sin(rc *i)
   'Print i,xyz(i,0),xyz(i,1),xyz(i,2)
  Next
End Sub

' ---------------------------------------------------------------------------
Sub R.resetInputBody
  ' Sets input variables (position / body) back to default
  in.bo_y = 0 : in.lh = 48
  in.tl_x = 0 : in.tl_z = 0 : in.tr_y = 0 : in.ds = 150
  in.ps_x = 0 : in.ps_z = 0 : in.pr_x = 0 : in.pr_y = 0 : in.pr_z = 0
  in.postureID = 0
  in.beepID = 0
End Sub

' ===========================================================================
' Display-related
' ---------------------------------------------------------------------------
Sub LCD.power state%
  ' Power screen on/off
  If Not(USE_LCD) Then Exit Sub
  CLS RGB(black)
  Font #4
  'FRAMEBUFFER Create
  'FRAMEBUFFER Write f
  Backlight Choice(state% = 1, 100, 0)
End Sub


Sub LCD.drawBarC i, v,_vmax,_vlim, _x,_y,_dx,_dy, _f$, _a
  Static integer xy(LCD_N,1), dxy(LCD_N,2), a(LCD_N), first=1
  Static vmaxlim(LCD_N,2)
  Static string fstr$(LCD_N) length 10
  Local integer x(15), y(15), n(3)=(4,4,4,4)
  Local integer bc(3)=(C_BKG, C_BKG, C_TXT, C_TXT)
  Local integer dv, j, cx,cy
  If first Then
    Array Set 0, vmaxlim()
    Array Set 0, dxy()
    Array Set 0, a()
    first = 0
  EndIf
  If i < 0  Or i > LCD_N Then Exit Sub
  If dxy(i,0) = 0 Then
    ' Define new indicator
    xy(i,0) = _x
    xy(i,1) = _y
    dxy(i,0) = _dx
    dxy(i,1) = _dy
    vmaxlim(i,0) = _vmax
    vmaxlim(i,1) = _vlim
    fstr$(i) = _f$
    a(i) = _a
  Else
    dv = Int(vmaxlim(i,2))
  EndIf
  vmaxlim(i,2) = Int(dxy(i,0) *v /vmaxlim(i,0))
  bc(2) = Choice(v > vmaxlim(i,1), C_OK, C_WARN)

  ' Draw bar
  If a(i) = 0 Then
    ' Use rectangles (box)
    Box xy(i,0),xy(i,1),dxy(i,0),dxy(i,1),,C_BKG,C_BKG
    Box xy(i,0),xy(i,1),vmaxlim(i,2),dxy(i,1),,bc(2)
    Box xy(i,0)+vmaxlim(i,2)+3,xy(i,1),dxy(i,0)-vmaxlim(i,2)-3,dxy(i,1),,bc(3)
  Else
    ' Use polygon for rotation
    x(0) = xy(i,0)
    y(0) = xy(i,1)
    x(1) = xy(i,0) +dv
    y(1) = y(0)
    x(2) = x(1)
    y(2) = y(0) +dxy(i,1)
    x(3) = x(0)
    y(3) = y(2)
    x(4) = xy(i,0) +dv +3
    y(4) = xy(i,1)
    x(5) = xy(i,0) +dxy(i,0)
    y(5) = y(4)
    x(6) = x(5)
    y(6) = y(4) +dxy(i,1)
    x(7) = x(4)
    y(7) = y(6)
    For j=0 To 7
      x(j+8) = x(j)
      y(j+8) = y(j)
    Next
    x(9) = xy(i,0) +vmaxlim(i,2)
    x(10) = x(9)
    x(12) = xy(i,0) +vmaxlim(i,2) +3
    x(15) = x(12)
    cx = xy(i,0) +dxy(i,0)\2
    cy = xy(i,1) +dxy(i,1)\2
    Math V_Rotate cx,cy, a(i), x(),y(),x(),y()
    Polygon n(), x(),y(), bc()
  EndIf
  If Len(fstr$(i)) = 0 Then Exit Sub
  Text xy(i,0),xy(i,1)+dxy(i,1)+2, Format$(v, fstr$(i)), "LT"
End Sub


Sub LCD.drawLabel _state, x, y, txt$, opt$
  Local integer c = Choice(_state, C_OK, C_WARN)
  Text x,y, txt$, "CB",,,c
  Text x,y+14, Field$(opt$, (_state > 0)+1, ","), "CB",,,C_BKG, c
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub LCD.update
  Static integer x0=MM.HRES\2, y0=MM.VRES\2, dr=x0-1
  Static integer xc,yc, xt(2),yt(2), xh(2),yh(2), rh=50
  Static ah
  Static t_bat=Timer, first = 1
  Local integer y, dx,dy, r, c, gp = MM.Info(USB gpad.ch) >= 128
  Local t, x
  Local String s$

  ' Initialize GUI elements
  If first Then
    dx = 80 : dy = 12
    LCD.drawBarC 0, 0,4.5,3.4, x0-dx\2+1,y0+85, dx-3,dy, "Log %.1fV"
    LCD.drawBarC 1, 0,9.0,7.4, x0-dx\2+1,y0+55, dx-3,dy, "Srv %.1fV"
    xt(0) = x0
    yt(0) = y0 -rh -2
    xt(1) = x0 -8
    yt(1) = y0 -rh +14
    xt(2) = x0 +8
    yt(2) = yt(1)
    first = 0
  EndIf

  t = Timer
  ' Slow changing states (battery status, WiFi)
  If t > t_bat +1000 Then
    LCD.drawBarC 0, R.LogicBattery_V
    LCD.drawBarC 1, R.ServoBattery_V

    LCD.drawLabel R.WiFi, x0-64,y0+70, "WiFi", "_OFF,_ON_"
    LCD.drawLabel R.connected, x0+65,y0+70, "Serv", "_--_,_OK_"
    LCD.drawLabel gp, x0+65,y0+40, "GPad", "_--_,_OK_"

    t_bat = t
  EndIf

  ' Compass
  Circle x0,y0, rh,,,C_TXT
  Circle xc,yc, 16,,,C_BKG
  xc = Int(x0 +rh *R.IMU(2)/90)
  yc = Int(y0 -rh *R.IMU(1)/90)
  Circle xc,yc, 16,,,C_TXT
  Math V_Rotate x0,y0, -ah, xt(),yt(),xh(),yh()
  Triangle xh(0),yh(0),xh(1),yh(1),xh(2),yh(2),C_BKG
  ah = Rad(R.IMU(0))
  Math V_Rotate x0,y0, -ah, xt(),yt(),xh(),yh()
  Triangle xh(0),yh(0),xh(1),yh(1),xh(2),yh(2),C_OK
  Text x0,y0-rh, Str$(R.IMU(0),3,0)+Chr$(96), "CB"

  'Print "GGN state='";Field$(GGN_STATE$, R.ggn_state +1)+"' ";
  'Print "selector='"+Field$(SEL_STATE$, R.selector +1)+"'"
  'Print "IMU heading="+Str$(R.IMU(0),3,0)+" pitch="+Str$(R.IMU(1),3,0)+" ";
  'Print "roll="+Str$(R.IMU(2),3,0)

  'FRAMEBUFFER Copy f,n
  't = Timer -t : Print Str$(t,2,3)
End Sub

' ===========================================================================
' Communication w/ server
' ---------------------------------------------------------------------------
Sub COM.Open
  ' Open communication to server
  Print "Connecting to server ";
  Local string s$
  Print "as I2C device &H" +Hex$(COM_I2C_ADDR, 2) +" ... "
  SetPin GP0, GP1, I2C
  I2C Open COM_I2C_SPEED, COM_I2C_TOUT
  /*
  I2C Check COM_I2C_ADDR
  com.isReady = MM.I2C = 0
  s$ = Choice(com.IsReady, "", "Device not found.")
  */
  com.isReady = 1
  Print "| " +Choice(com.isReady, "Ready.", "Error: " +s$)
End Sub


Sub COM.Close
  ' Close communication to server
  If Not(com.isReady) Then Exit Sub
  I2C Close
  com.isReady = 0
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub COM.sendMsg _cmd
  ' Send command `_cmd` to client
  Static integer first = 1, pbuf, pv
  Static string buf$ length COM_N_BYTEVAL
  Static integer _out(COM_N_BYTEVAL-1), v(COM_N_BYTEVAL/8 -1)
  If first Then
    pbuf = Peek(VarAddr buf$)
    Poke Byte pbuf, COM_N_BYTEVAL
    pv = Peek(VarAddr v())
    first = 0
  EndIf
  If DEBUG And DEB_COM Then Print "Sending ..."

  ' Compile data array
  ' (assumes that the parameters are already range checked)
  '      0 : command code
  '      1 : n data bytes (following), currenly 7
  '      2 : bodyYOffs
  '      3 : legLiftH
  '    4,5 : x_zTravL
  '      6 : travRotY
  '      7 : delaySpeed/10
  '      8 : buzzer signal
  '      9 : command extension
  '  10,11 : x_zBody
  '  12-14 : xyzBodyRot
  Math Set 0, _out()
  _out(0) = _cmd
  _out(1) = 7

  If _cmd = CMD_STATE Then
    If Not(DEBUG And DEB_SLNT) Then Print "|>State"

  ElseIf _cmd = CMD_BODY Then
    ' Body posture
    _out(2)  = in.bo_y -BODY_Y_OFFS_MIN
    _out(3)  = in.lh   -(-LEG_LIFT_MIN)
    _out(10) = in.ps_x -(-BODY_X_Z_POS_LIM(0))
    _out(11) = in.ps_z -(-BODY_X_Z_POS_LIM(2))
    _out(12) = in.pr_x -(-BODY_XYZ_ROT_LIM(0))
    _out(13) = in.pr_y -(-BODY_XYZ_ROT_LIM(1))
    _out(14) = in.pr_z -(-BODY_XYZ_ROT_LIM(2))
    If Not(DEBUG And DEB_SLNT) Then
      Print "|>Body bo_y="+Str$(in.bo_y,2)+" lh="+Str$(in.lh,2);
      Print " ps_x,z="+Str$(in.ps_x,2)+","+Str$(in.ps_z,2);
      Print " pr_x,y,z="+Str$(in.pr_x,2)+","+Str$(in.pr_z,2)+",";
      Print Str$(in.pr_z,2)
    EndIf

  ElseIf _cmd = CMD_MOVE Then
    _out(4) = in.tl_x -(-TRAVEL_X_Z_LIM(0))
    _out(5) = in.tl_z -(-TRAVEL_X_Z_LIM(2))
    _out(6) = in.tr_y -(-TRAV_ROT_Y_LIM)
    _out(7) = Int(in.ds /10)
    If Not(DEBUG And DEB_SLNT) Then
      Print "|>Move tl_x,z="+Str$(in.tl_x,-2)+","+Str$(in.tl_z,-2);
      Print " tr_y="+Str$(in.tr_y,-2)+" ds="+Str$(in.ds,4)
    EndIf

  ElseIf _cmd = CMD_STOP Then
    _out(7) = Int(in.ds /10)
    If Not(DEBUG And DEB_SLNT) Then Print "|>Stop"

  ElseIf _cmd = CMD_POWER Then
    _out(9) = R.running
    If Not(DEBUG And DEB_SLNT) Then Print "|>Power"

  ElseIf _cmd = CMD_SHOW Then
    _out(8) = in.beepID
    _out(9) = in.postureID
    in.postureID = 0
    in.beepID = 0

  Else
    Print "|>Error: Invalid command"
    Exit Sub
  EndIf
  If DEBUG And DEB_COM Then
    Print "|>Parameters (offset w/ limits)"
    Math V_print _out()
  EndIf

  ' Pack single-byte values into a string
  Math Add _out(), COM_DATA_OFFS, _out()
  Memory Pack _out(), v(), COM_N_BYTEVAL, 8
  Memory Copy pv, pbuf+1, COM_N_BYTEVAL

  ' Send message
  Local string s$ = Hex$(Timer, 8) +buf$
  I2C Write COM_I2C_ADDR, 1, COM_MSG_LEN, s$
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub COM.handleStateMsg
  ' Check if a new state message from the server is available, and if so
  ' handle it
  Static integer first = 1, pbuf, pv, v(COM_MSG_LEN/8 -1)
  Static String buf$
  Local integer n, j
  If first Then
    pbuf = Peek(VarAddr buf$)
    pv = Peek(VarAddr v())
    first = 0
  EndIf
  com.in_res = 0

  ' Check if communication link is ready and if there is data waiting
  If Not(com.isReady) Then Exit Sub
  COM.sendMsg CMD_STATE
  I2C Read COM_I2C_ADDR, 0, COM_MSG_LEN, buf$
  n = Choice(MM.I2C = 0, PEEK(BYTE pbuf), 0)

  ' Check data
  If DEBUG And DEB_COM Then Print "Receiving ..."
  com.in_res = RES_NEW_MSG
  If n <> COM_MSG_LEN Then
    ' Error: Length invalid, clear input buffer
    Print "|<Error: Invalid message length (";n;")"
    com.in_res = com.in_res Or RES_ERROR
    Exit Sub
  EndIf

  ' Parse the message, starting with the time of message
  ' (as hexlified uint32)
  com.in_t_ms = Val("&H"+Mid$(buf$,1,8))

  ' Unpack single-byte values into an array
  Memory Copy pbuf+9, pv, COM_N_BYTEVAL
  Memory Unpack v(), com.in(), COM_N_BYTEVAL, 8
  Math Add com.in(), -COM_DATA_OFFS, com.in()
  If DEBUG And DEB_COM Then
    Print "|<Parameters"
    Math V_print com.in()
  EndIf
  If Not(com.in(0) = CMD_STATE) Then
    If DEBUG And DEB_COM Then
      Print "|<Error: CMD_STATE expected"
      com.in_res = com.in_res Or RES_ERROR
    EndIf
    Exit Sub
  EndIf

  ' Extract data array
  '      0 : command code
  '      1 : n data bytes (following), currenly 17
  '      2 : servo battery voltage [V*10]
  '      3 : logic battery voltage [V*10]
  '    4,5 : compass heading [degree]
  '    6,7 : compass pitch and roll [-90..90, degree]
  '      8 : GGN state
  '  9..16 : servo load for servos #0..7, 0..240 [a.u.]
  R.servoBattery_V = com.in(2) /10
  R.logicBattery_V = com.in(3) /10
  R.IMU(0) = com.in(4) *10 +com.in(5)
  R.IMU(1) = com.in(6) -90
  R.IMU(2) = com.in(7) -90
  R.ggn_state = com.in(8)
  For j=0 To SRV_N_LOAD_CH -1
    R.load(j) = com.in(9 +j)
  Next
  com.in_res = com.in_res Or RES_HANDLED
End Sub

' ============================================================================
' Gamepad-related
' ----------------------------------------------------------------------------
' USB device types:
'   0=not in use, 1=keyboard, 2=mouse, 128=ps4, 129=ps3, 130=SNES/Generic
'
' Gamepad data:
'   &H0001 := R1
'   &H0002 := Options/Start
'   &H0004 := PS/Home
'   &H0008 := Share/Select
'   &H0010 := L1
'   &H0020 := pad, down
'   &H0040 := pad, right
'   &H0080 := pad, up
'   &H0100 := pad, left
'   &H0200 := R2 pressed, value in `gpad.data(5)`
'   &H0400 := Triangle
'   &H0800 := Circle
'   &H1000 := Square
'   &H2000 := Cross
'   &H4000 := L2 pressed, value in `gpad.data(4)`
'   &H8000 := touchpad pressed
' ----------------------------------------------------------------------------
Sub initGamepad _type
  ' Look for gamepad `_type` and initialize it, if available
  Local integer i, j
  Local string s$
  gpad.ch = 0
  gpad.type = 0
  Math Set 0, gpad.data()
  For i=1 To 4
    j = MM.Info(USB i)
    If j = _type Then
      Print "| Gamepad of type";j;" found on channel";i
      gpad.ch = i
      gpad.type = _type
      Gamepad interrupt enable i, _cb_gamepad
      Exit Sub
    EndIf
  Next
  Print "| No gamepad of type";_type;" found."
End Sub


Sub _cb_gamepad
  ' Interrupt callback for gamepad
  If gpad.type < 128 Then Exit Sub
  gpad.data(6) = DEVICE(gamepad gpad.ch, b)
  gpad._intDetect = 1
End Sub


Function updateGamepad()
  ' Keeps gamepad data up to date (call frequently)
  Static integer v(6)
  If gpad.type < 128 Then updateGamepad = 0 : Exit Sub : EndIf

  ' Read gamepad sticks etc
  gpad.data(0) = DEVICE(gamepad gpad.ch, lx)
  gpad.data(1) = DEVICE(gamepad gpad.ch, ly)
  gpad.data(2) = DEVICE(gamepad gpad.ch, rx)
  gpad.data(3) = DEVICE(gamepad gpad.ch, ry)
  gpad.data(4) = DEVICE(gamepad gpad.ch, l)
  gpad.data(5) = DEVICE(gamepad gpad.ch, r)

  ' Check if any stick value as substantially changed
  Math C_Sub v(), gpad.data(), v() : v(6) = 0
  gpad.changed = gpad._intDetect Or Abs(Math(Sum v())) > 6 '2
  gpad._intDetect = 0
  updateGamepad = gpad.changed
  Math Scale gpad.data(), 1, v()
End Function

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub checkGamepad
  Local string s$ = ""
  If gpad.data(6) And &H0400 Then s$ = s$ +"Triangle "
  If gpad.data(6) And &H1000 Then s$ = s$ +"Square "
  If gpad.data(6) And &H0800 Then s$ = s$ +"Circle "
  If gpad.data(6) And &H2000 Then s$ = s$ +"Cross "
  If gpad.data(6) And &H0010 Then s$ = s$ +"L1 "
  If gpad.data(6) And &H0001 Then s$ = s$ +"R1 "
  If gpad.data(6) And &H0002 Then s$ = s$ +"Start "
  If gpad.data(6) And &H4000 Then s$ = s$ +"L2 ("+Str$(gpad.data(4))+") "
  If gpad.data(6) And &H0200 Then s$ = s$ +"L2 ("+Str$(gpad.data(5))+") "
  If gpad.data(6) And &H0008 Then s$ = s$ +"Share "
  If gpad.data(6) And &H0004 Then s$ = s$ +"PS "
  If gpad.data(6) And &H0100 Then s$ = s$ +"< "
  If gpad.data(6) And &H0040 Then s$ = s$ +"> "
  If gpad.data(6) And &H0080 Then s$ = s$ +"^ "
  If gpad.data(6) And &H0020 Then s$ = s$ +"v "
  If gpad.data(6) And &H8000 Then s$ = s$ +"Touchpad "
  Print "&B"+Bin$(gpad.data(6), 32)""
  Print s$
End Sub

' ===========================================================================
' Miscellaneous
' ---------------------------------------------------------------------------
Function isWiFiOk(reboot)
  ' Check for WiFi connection, if none and `reboot` = 1, then reboot
  isWiFiOk = 0
  Local integer count = 3
  Do While count >= 3 And MM.Info(IP Address) = "0.0.0.0"
    Print "\&0dWaiting for IP address ("+Str$(count)+" secs) ...";
    Pause 1000
    Inc count, -1
  Loop
  If count < 3 Then Print
  isWiFiOk = MM.Info(IP Address) <> "0.0.0.0"
  If IsWiFiOk Or Not(reboot) Then Exit Function
  Option Autorun On
  CPU Restart
  Print "RESTART"
End Function

' ---------------------------------------------------------------------------
                                                        