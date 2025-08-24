' lib_keyboard.bas v0.1.0
' ----------------------
' Control robot by keyboard via console
' (not a stand-alone library, for `hps_srv.bas`)
'
' Copyright (c) 2023-25 Thomas Euler
' MIT Licence
' ---------------------------------------------------------------------------

print "ctrlByKeyboard routine in library (not standalone)"
print "| Requirements: None"

' ---------------------------------------------------------------------------
Function ctrlByKeyboard()
  ' Check for keyboard input and handle if, if possible. Returns
  ' key code or 0, if no key was pressed
  Local string key$
  Local integer ch = 0
  Local v1
  in.cmd = 0
  key$ = Inkey$
  If Len(key$) > 0 Then
    ' Process keyboard input
    ch = Asc(LCase$(key$))
    Select Case ch
      Case 145     ' F1  - Help
        Print
        Print "  F1          - help"
        Print "Movement\r\n--------"
        Print "  Up,Down     - Forwards/backward (z travel length) "
        Print "  Left,Right  - Sideways (x travel length)"
        Print "  Del,Pg-Down - Turn left/right (y travel rotation)"
        Print "  1,2         - Velocity (delay speed)"
        Print "Body\r\n----"
        Print "  3,4         - y body offset"
        Print "  5,6         - Leg lift height"
        Print "  Q,A         - Lean forward/backward (z body)"
        Print "  W,S         - Pitch (z body rotate)"
        Print "  E,D         - Lean sideways (x body)"
        Print "  R,F         - Roll (x body rotate)"
        Print "  Ins,Pg-Up   - Yaw (y body rotate)"
        Print "  End         - Stop motion "
        Print "  F12         - Power up/down"
        Print
      Case 146     ' F2  - Status
        R.logStateInfo

      Case 49,50   ' 1,2 - delay speed
        If R.running = 2 Then
          Inc in.ds, Choice(ch = 49, -10, 10)
          in.ds = Min(Max(in.ds, 0), DELAY_SPEED_MAX)
          in.cmd = CMD_MOVE
        EndIf
      Case 51,52   ' 3,4 - body offset
        If R.running = 2 Then
          Inc in.bo_y, Choice(ch = 51, -5, 5)
          in.bo_y = Min(Max(in.bo_y, 0), BODY_Y_OFFS_MAX)
          in.cmd = CMD_BODY
        EndIf
      Case 53,54   ' 5,6 - leg lift height
        If R.running = 2 Then
          Inc in.lh, Choice(ch = 53, -5, 5)
          in.lh = Min(Max(in.lh, LEG_LIFT_MIN), LEG_LIFT_MAX)
          in.cmd = CMD_BODY
        EndIf

      Case 128,129,117,106 ' up,down (u,j) - z travel length
        If R.running = 2 Then
          If ch = 117 Then ch = 128
          Inc in.tl_z, Choice(ch = 128, -10, 10)
          v1 = TRAVEL_X_Z_LIM(2)
          in.tl_z = Min(Max(in.tl_z, -v1), v1)
          in.cmd = CMD_MOVE
        EndIf
      Case 130,131,105,107 ' left,right (i,k) - x travel length
        If R.running = 2 Then
          If ch = 105 Then ch = 130
          Inc in.tl_x, Choice(ch = 130, -5, 5)
          v1 = TRAVEL_X_Z_LIM(0)
          in.tl_x = Min(Max(in.tl_x, -v1), v1)
          in.cmd = CMD_MOVE
        EndIf
      Case 127,137,111,108 ' del,pg-down (o,l) - y travel rotation
        If R.running = 2 Then
          If ch = 111 Then ch = 127
          Inc in.tr_y, Choice(ch = 127, -5, 5)
          v1 = TRAV_ROT_Y_LIM
          in.tr_y = Min(Max(in.tr_y, -v1), v1)
          in.cmd = CMD_MOVE
        EndIf

      Case 113,97   ' q,a - z body (lean forward/backward)
        If R.running = 2 Then
          Inc in.ps_z, Choice(ch = 113, 5, -5)
          v1 = BODY_X_Z_POS_LIM(2)
          in.ps_z = Min(Max(in.ps_z, -v1), v1)
          in.cmd = CMD_BODY
        EndIf
      Case 119,115  ' w,s - z body (rotate) -> pitch
        If R.running = 2 Then
          Inc in.pr_z, Choice(ch = 119, -5, 5)
          v1 = BODY_XYZ_ROT_LIM(2)
          in.pr_z = Min(Max(in.pr_z, -v1), v1)
          in.cmd = CMD_BODY
        EndIf
      Case 101,100  ' e,d - x body (lean sideways)
        If R.running = 2 Then
          Inc in.ps_x, Choice(ch = 101, -5, 5)
          v1 = BODY_X_Z_POS_LIM(0)
          in.ps_x = Min(Max(in.ps_x, -v1), v1)
          in.cmd = CMD_BODY
        EndIf
      Case 114,102  ' r,f - x body (rotate) -> roll
        If R.running = 2 Then
          Inc in.pr_x, Choice(ch = 114, -5, 5)
          v1 = BODY_XYZ_ROT_LIM(0)
          in.pr_x = Min(Max(in.pr_x, -v1), v1)
          in.cmd = CMD_BODY
        EndIf
      Case 132,136  ' ins,pg_up - y body (rotate) -> yaw
        If R.running = 2 Then
          Inc in.pr_y, Choice(ch = 132, -5, 5)
          v1 = BODY_XYZ_ROT_LIM(1)
          in.pr_y = Min(Max(in.pr_y, -v1), v1)
          in.cmd = CMD_BODY
        EndIf
      Case 134     ' reset body position
        If R.running = 2 Then
          R.resetInputBody
          in.cmd = CMD_BODY
        EndIf

      Case 135,109 ' end (m) - stop all motion
        If R.running = 2 Then
          in.tl_x = 0 : in.tl_z = 0 : in.tr_y = 0
          in.cmd = CMD_STOP
        EndIf
      Case 45      ' `-` - stop turning
        If R.running = 2 Then
          in.tr_y = 0
          in.cmd = CMD_MOVE
        EndIf

      Case 156     ' F12 - Toggle power up/down
        R.resetInputBody
        R.running = Choice(R.running = 2, 1, 2)
        in.cmd = CMD_POWER
      Case 27      ' ESC, handled outside
      Case Else
        Print "ch=";ch
    End Select
  EndIf
  ctrlByKeyboard = ch
End Function

' ---------------------------------------------------------------------------