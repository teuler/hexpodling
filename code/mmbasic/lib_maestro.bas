' lib_maestro.bas v0.1.0
' ----------------------
' Robot commands regarding Maestro servo contoller
' (not a stand-alone library)
'
' Copyright (c) 2023-25 Thomas Euler
' MIT Licence
' ---------------------------------------------------------------------------

print "Maestro servo controller routines in library (not standalone)"
print "| Requirements: Serial port to controller opened at #5"

' ---------------------------------------------------------------------------
' All-servo routines
' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.Angles2Pos a_deg(), p()
  ' Converts all servo positions from angle (in degree) to position (in us)
  ' Note that is `a_deg(iLeg, COX|FEM|TIB)`
  Local integer i, j, k,  a
  For i=0 To LEG_N-1
    For j=0 To 2
      k = leg.srv(j,i)
      a = Min(srv.r_deg(1,k), Max(srv.r_deg(0,k), a_deg(i,j)))
      p(k) = srv.r_us(0,k) +srv.dt_us(k) *(a -srv.r_deg(0,k)) /srv.da_deg(k)
    Next j
  Next i
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._moveAllServos t_us(), dur_ms, n_srv%, wait%, no_move%
  ' Move the first `n_srv%` servos, or, if 0, all `SRV_N` servos to positions
  ' `t_us%()`. If `dur_ms` > 0, then it will attempt to move the servos such
  ' that they arrive at their targets at the same time (after `dur_ms`).
  ' Otherwise, servos are moved immediately.
  ' If `no_move%`, only internal position is updated.
  ' (NO PARAMETER CHECKING, starts with servo #0)
  If DEBUG And DEB_VERB Then Print "R._moveAllServos"
  Static String cmd$ length 40
  Static integer t(SRV_N-1), lhb, caddr = 0
  Static v(SRV_N-1), w(SRV_N-1)
  Local integer i, j, n = Choice(n_srv% = 0, SRV_N, n_srv%)
  Local dt

  ' Prepare if first call
  If caddr = 0 Then
    cmd$ = "\&9F"+Chr$(n)+Chr$(0)+Space$(SRV_N*2)
    caddr = Peek(VARADDR cmd$) +4
  EndIf

  ' Inactivate timer interrupt
  SetTick 0,0,1
  srv.isMoveDone = 1
  srv.iStep = -1

  ' Check if target position is current position
  Math C_Sub t_us(), srv.pCurr(), w()
  Math C_Mult w(), w(), w()
  If Math(Sum w()) < 1 Then Exit Sub

  If no_move% Then
    ' Just update internal servo positions
    Math Scale t_us(), 1, srv.pCurr()
    Exit Sub
  EndIf
  If dur_ms = 0 Then
    ' No move duration is given, just move to target
    Math Scale t_us(), 1, srv.pCurr()
    Math Scale t_us(), 4, t()
  Else
    ' Generate a sequence of steps
    Math C_Sub t_us(), srv.pCurr(), v()
    Math C_Div v(), srv.nStep(), v()
    Math Scale srv.pCurr(), 1, w()
    For i=0 To SRV_N_STEPS-1
      Math C_Add w(), v(), w()
      Math Insert srv.pSteps(), i,, w()
    Next
    Math Slice srv.pSteps(), 0,, w()
    Math Scale w(), 4, t()
    dt = dur_ms /SRV_N_STEPS
    srv.iStep = 1
    srv.nSrv = n
    srv.isMoveDone = 0
  EndIf

  ' Move to position `t()`
  j = caddr
  For i=0 To n-1
    lhb = (((t(i) >> 7) And &H7F) << 8) Or (t(i) And &H7F)
    Poke SHORT j, lhb : Inc j, 2
  Next
  Print #5, cmd$

  ' If step sequence, then start interrupt routine
  If srv.iStep >= 0 Then SetTick dt, R._cbServo, 1
  If wait% Then Do While Lof(#5) < 256 : Pause 1 : Loop
End Sub


Sub R._cbServo
  ' INTERNAL: Timer callback for smooth servo movements
  Static integer t(SRV_N-1), i, j, lhb, caddr = 0
  Static String cmd$ length 40
  Static w(SRV_N-1)
  ' Prepare if first call
  If caddr = 0 Then
    cmd$ = "\&9F"+Chr$(SRV_N)+Chr$(0)+Space$(SRV_N*2)
    caddr = Peek(VARADDR cmd$) +4
  EndIf

  ' Get positions and send to servo controller
  Math Slice srv.pSteps(), srv.iStep,, w()
  Math Scale w(), 4, t()
  j = caddr
  For i=0 To srv.nSrv-1
    lhb = (((t(i) >> 7) And &H7F) << 8) Or (t(i) And &H7F)
    Poke SHORT j, lhb : Inc j, 2
  Next
  Print #5, cmd$

  ' Check if move is done ...
  Inc srv.iStep, 1
  If srv.iStep = SRV_N_STEPS Then
    ' Stop timer and set target position as new current position
    SetTick 0,0,1
    Math Slice srv.pSteps(), SRV_N_STEPS-1,, srv.pCurr()
    srv.isMoveDone = 1
    srv.iStep = -1
  EndIf
End Sub


Sub R.servosOff
  ' Switch all servos off
  Print #5, "\&9F"+Chr$(SRV_N)+Chr$(0)+String$(SRV_N*2, 0)
  Print "| Servos off (&H"+Hex$(R.getServoErr(), 4)+")"
End Sub

' ---------------------------------------------------------------------------
' Single-servo methods
' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.setServo_deg i%, a_deg%, wait%
  ' Set servo `i%` position as angle `a_deg%` in degree
  If i% >= 0 And i% < SRV_N Then
    Local integer _t = R._a2t(i%, a_deg%)
    R._SetServo i%, _t, wait%
  EndIf
End Sub


Sub R.setServo i%, t_us%, wait%
  ' Set servo `i%` position as timing `t_us%` (in us)
  If i% >= 0 And i% < SRV_N Then
    Local integer _t = Min(Max(t_us%, SRV_MIN_US), SRV_MAX_US)
    R._SetServo i%, _t, wait%
  EndIf
End Sub


Sub R._setServo i%, t_us%, wait%
  ' Set servo `i%` position as timing `t_us%` (in us)
  ' If `wait` <> 0, it is waited until the out buffer is empty, NOT until
  ' the servo has reached its position
  ' (NO PARAMETER CHECKING)
  Local integer hb, lb
  Local integer _t = t_us% *4
  hb = (_t And >> 7) And &H7F
  lb = _t And &H7F

  ' Send `set target` command
  If DEBUG And DEB_SRV Then
    Print "Servo #";i%;" to ";Str$(_t /4, 0);" us"
  EndIf
  Print #5, "\&84"+Chr$(i%)+Chr$(lb)+Chr$(hb)
  If wait% Then Do While Lof(#5) < 256 : Pause 1 : Loop
End Sub


Function R._a2t(i%, a_deg%) As integer
  ' Convert for servo `%i` the angle `a_deg%` into the position (in us)
  ' Considers the servo's range in deg and its calibration values
  Local integer t, a = Min(srv.r_deg(1,i%), Max(srv.r_deg(0,i%), a_deg%))
  t = srv.r_us(0,i%) +srv.dt_us(i%) *(a -srv.r_deg(0,i%)) /srv.da_deg(i%)
  R._a2t = t
  If Not(DEBUG And DEB_SRV) Then Exit Function
  Print "Servo #"+Str$(i%)+" "+Str$(a,3,0) +Chr$(186)+" -> ";
  Print Str$(t,4,0)+" us"
End Function

' ---------------------------------------------------------------------------
' Routines to change servo parameters and retrieve information
' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.setServoSpeed i%, speed%
  ' Sets speed of servo `i%`
  Local integer hb, lb
  If i% < 0 Or i% > SRV_N-1 Then Exit Sub
  hb = (speed% >> 7) And &H7F
  lb = speed% And &H7F
  Print #5, "\&87"+Chr$(i%)+Chr$(lb)+Chr$(hb)
End Sub


Sub R.setServoAccel i%, accel%
  ' Sets acceleration of servo `i%`
  Local integer hb, lb
  If i% < 0 And i% > SRV_N-1 Then Exit Sub
  hb = (accel% >> 7) And &H7F
  lb = accel% And &H7F
  Print #5, "\&89"+Chr$(i%)+Chr$(lb)+Chr$(hb)
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Function R.getServoErr() As integer
  ' Returns (and clears) last servo controller error
  Static string res$
  Static integer pRes = Peek(VARADDR res$)
  Local integer i

  ' Send `get errors` command and wait for reply
  If DEBUG And DEB_SRV Then Print "Get errors: ";
  Print #5, "\&A1"
  i = 20
  Do While Loc(#5) = 0 And i > 0
    Pause 2
    Inc i, -1
  Loop
  res$ = Input$(2, #5)

  ' Process reply
  If PEEK(BYTE pRes) > 0 Then
    R.getServoErr = (PEEK(BYTE pRes+1) +(PEEK(BYTE pRes+2) << 8)) And &H1F
    If DEBUG And DEB_SRV Then Print "0x"+Hex$(R.getServoErr, 2)
  Else
    R.getServoErr = -1
    If DEBUG And DEB_SRV Then Print "No reply"
  EndIf
End Function

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub _log.servos
  ' Print current angle (in degree) and position (in us) for each servo
  Local integer i
  For i=0 To SRV_N-1
    Print "Servo #"+Str$(i)+" "+Str$(srv.ang(i),3,0) +Chr$(186)+" -> ";
    Print Str$(srv.pos(i),4,0)+" us"
  Next
End Sub

' ---------------------------------------------------------------------------
