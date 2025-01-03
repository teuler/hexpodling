' lib_mcp3208.bas v0.1.2
' -----------------------
' MCP3208 8-channel ADC driver
' 2024-02-04 - v0.1.1 - Fewer constant declarations to save global
'                       variable slots
' 2024-11-25 - v0.1.2 - Test routine added
' Example:
'   Const PIN_SPI2_RX      = MM.Info(PinNo GP12)
'   Const PIN_SPI2_TX      = MM.Info(PinNo GP11)
'   Const PIN_SPI2_CLK     = MM.Info(PinNo GP10)
'   Const PIN_MCP3208_CS   = MM.Info(PinNo GP13)
'   MCP3208.init PIN_SPI2_RX, PIN_SPI2_TX, PIN_SPI2_CLK
'   Dim v(3)
'   MCP3208.readADCChans 4, v()
'   Math V_print v()
'   MCP3208.close
' ---------------------------------------------------------------------------
Option Base 0
Option Explicit
Option Escape
Option Default Float

' Constants
'Const MCP3208_SPEED      = 2000000
'Const MCP3208_MODE       = 0
'Const MCP3208_BITS       = 8
Dim integer MCP3208.cs    = MM.Info(PinNo GP13)

' ===========================================================================
' MCP3208 8-channel ADC driver
' ---------------------------------------------------------------------------
Sub MCP3208.init rx%, tx%, clk%, cs%
  ' Setup user SPI2
  Print "Initializing MCP3208 via SPI2 ..."
  MCP3208.cs = cs%
  SetPin rx%, tx%, clk%, SPI2
  SetPin MCP3208.cs, DOUT
  Pin(MCP3208.cs) = 0
  Pause 50
  Pin(MCP3208.cs) = 1
  SPI2 Open 2000000, 0, 8 ' 2MHz, mode 0, 8 bits
  Print "| Ready."
End Sub

Function MCP3208.readADC(ch%)
  ' Read channel `ch%`
  Local integer tmp
  Pin(MCP3208.cs) = 0
  SPI2 Write 1, &H06
  SPI2 Write 1, (ch% << 6)
  tmp = (SPI2(0) And &H0F) << 8
  tmp = tmp Or SPI2(0)
  Pin(MCP3208.cs) = 1
  MCP3208.readADC = tmp
End Function

Sub MCP3208.readADCChans n%, dta%()
  ' Read channels 0 to `n%`-1 into `dta%()`
  Local integer i
  For i=0 To n%-1
    Pin(MCP3208.cs) = 0
    SPI2 Write 1, &H06
    SPI2 Write 1, (i << 6)
    dta%(i) = (SPI2(0) And &H0F) << 8
    dta%(i) = dta%(i) Or SPI2(0)
    Pin(MCP3208.cs) = 1
  Next
End Sub

Sub MCP3208.close
  ' Close SPI2 devices
  SPI2 Close
  SetPin MCP3208.cs, OFF
End Sub

' ---------------------------------------------------------------------------
Sub MCP3208.test
  Local integer rx, tx, clk, cs, i
  Local integer v(3)
  rx  = MM.Info(PinNo GP12)
  tx  = MM.Info(PinNo GP11)
  clk = MM.Info(PinNo GP10)
  cs  = MM.Info(PinNo GP13)

  MCP3208.init rx, tx, clk, cs
  For i=0 To 100
    MCP3208.readADCChans 4, v()
    Math V_print v()
    Pause 500
  Next
  'Erase MCP3208.test.rx, MCP3208.test.tx, MCP3208.test.clk
  'Erase MCP3208.test.cs, MCP3208.test.i, MCP3208.test.v()
  MCP3208.close
End Sub

' ---------------------------------------------------------------------------
                                                                                           