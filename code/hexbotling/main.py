# ----------------------------------------------------------------------------
# main.py
# Main program; is automatically executed after reboot.
#
# The MIT License (MIT)
# Copyright (c) 2020 Thomas Euler
# 2020-01-06, First version
# 2020-09-04, Version for both server and client
#
# ----------------------------------------------------------------------------
from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  # Only the server board uses a TinyPICO, hence, import and create an
  # instance of the robot server class
  # the server object
  from hexapod_server import HexaPodServer
  r = HexaPodServer()
else:
  # Import and create and instance of the robot client class
  from hexapod_client import HexaPodClient
  r = HexaPodClient()

# Call main
if __name__ == "__main__":
  try:
    r.loop()
  except Exception as e:
    '''
    # Create an in-memory file-like string object (`sIO`) to accept the
    # exception's traceback (`sys.print_exception` requires a file-like
    # object). This traceback string is then converted into a dictionary
    # to be sent as an MQTT message (if telemetry is activate)
    if r.Tele and r.Tele.connected:
      import sys, uio
      sIO = uio.StringIO()
      sys.print_exception(e, sIO)
      r.Tele.publishDict(KEY_RAW, {KEY_DEBUG: str(sIO.getvalue())})
    # Re-raise the exception such that it becomes also visible in the REPL
    '''
    raise e

"""
from robotling_lib.sensors.teraranger_evomini import TeraRangerEvoMini as TREM
import board
import time

tr = TREM(0, tx=board.IO33, rx=board.IO5)
for i in range(500):
  tr.update()
  print(tr.distance, tr.invalids)
  time.sleep(0.5)
"""
"""
try:
  from time import ticks_ms, ticks_diff
except ImportError:
  import time as _time
  def ticks_ms():
    return int(_time.monotonic() *1000)
  def ticks_diff(ticks1, ticks2):
    return ticks1 -ticks2

try:
  from machine import freq
except ImportError:
  from microcontroller import cpu
  def freq():
    return cpu.frequency
import gc

def pi(places=100):
  # 3 + 3*(1/24) + 3*(1/24)*(9/80) + 3*(1/24)*(9/80)*(25/168)
  # The numerators 1, 9, 25, ... are given by (2x + 1) ^ 2
  # The denominators 24, 80, 168 are given by (16x^2 -24x + 8)
  extra = 8
  one = 10 ** (places+extra)
  t, c, n, na, d, da = 3*one, 3*one, 1, 0, 0, 24

  while t > 1:
    n, na, d, da = n+na, na+8, d+da, da+32
    t = t * n // d
    c += t
  return c // (10 ** extra)

def pi_test(n = 5000):
    t1 = ticks_ms()
    t = pi(n)
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Pi', n, 'digit calculation: ', r, 's')
    return '%.2f'%r

def int_add_test(n = 1000000, a = 12345, b = 56789):
    t1 = ticks_ms()
    sum = 0
    for i in range(n):
        sum = a + b
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Integer Add test', n, 'times: ', r, 's')
    return '%.2f'%r

def float_add_test(n=1000000, a = 1234.5678, b = 5678.1234):
    t1 = ticks_ms()
    sum = 0
    for i in range(n):
        sum = a + b
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Float Add test', n, 'times:', r, 's')
    return '%.2f'%r

def int_mul_test(n=1000000, a = 12345, b = 56789):
    t1 = ticks_ms()
    sum = 0
    for i in range(n):
        sum = a * b
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Integer Mul test', n, 'times: ', r, 's')
    return '%.2f'%r

def float_mul_test(n=1000000, a = 1234.5678, b = 5678.1234):
    t1 = ticks_ms()
    sum = 0
    for i in range(n):
        sum = a * b
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Float Mul test', n, 'times: ', r, 's')
    return '%.2f'%r

def int_div_test(n=1000000, a = 123456, b = 567):
    t1 = ticks_ms()
    sum = 0
    for i in range(n):
        sum = a // b
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Integer Div test', n, 'times: ', r, 's')
    return '%.2f'%r

def float_div_test(n=1000000, a = 12345.678, b = 56.789):
    t1 = ticks_ms()
    sum = 0
    for i in range(n):
        sum = a / b
    t2 = ticks_ms()
    r = ticks_diff(t2, t1)/1000
    print('  Float Div test', n, 'times: ', r, 's')
    return '%.2f'%r

def mem():
    r = gc.mem_free()
    print('free memory:', r)

print('Speed test')
try:
    print('System freq: {:.1f} MHz'.format(freq()[0]/1000000))
except:
    print('System freq: {:.1f} MHz'.format(freq()/1000000))

print('\nCalcaulate integer addition')
gc.collect()
mem()
d1 = int_add_test()
d2 = int_add_test()
d3 = int_add_test()
r_int_add =  min(d1, d2, d3)
print('Integer addition test result: ', r_int_add, 's')
mem()

print('\nCalcaulate float addition')
gc.collect()
mem()
d1 = float_add_test()
d2 = float_add_test()
d3 = float_add_test()
r_float_add = min(d1, d2, d3)
print('Float addition test result: ', r_float_add, 's')
mem()

print('\nCalcaulate integer multiplication')
gc.collect()
mem()
d1 = int_mul_test()
d2 = int_mul_test()
d3 = int_mul_test()
r_int_mul = min(d1, d2, d3)
print('Integer multiplication test result: ', r_int_mul, 's')
mem()

print('\nCalcaulate float multiplication')
gc.collect()
mem()
d1 = float_mul_test()
d2 = float_mul_test()
d3 = float_mul_test()
r_float_mul = min(d1, d2, d3)
print('Float multiplication test result: ', r_float_mul, 's')
mem()

print('\nCalcaulate integer division')
gc.collect()
mem()
d1 = int_div_test()
d2 = int_div_test()
d3 = int_div_test()
r_int_div = min(d1, d2, d3)
print('Integer division test result: ', r_int_div, 's')
mem()

print('\nCalcaulate float division')
gc.collect()
mem()
d1 = float_div_test()
d2 = float_div_test()
d3 = float_div_test()
r_float_div = min(d1, d2, d3)
print('Float division test result: ', r_float_div, 's')
mem()

print('\nCalcaulate Pi 1000 digit')
gc.collect()
mem()
try:
    d1 = pi_test(1000)
    d2 = pi_test(1000)
    d3 = pi_test(1000)
    r_pi_1000 = min(d1, d2, d3)
    print('1000 digit Pi calculation result: ', r_pi_1000, 's')
    mem()
except:
    r_pi_1000 = None
    print('  calculation error')

print('\nCalcaulate Pi 5000 digit')
gc.collect()
mem()
try:
    d1 = pi_test(5000)
    d2 = pi_test(5000)
    d3 = pi_test(5000)
    r_pi_5000 = min(d1, d2, d3)
    print('5000 digit Pi calculation result: ', r_pi_5000, 's')
    mem()
except:
    r_pi_5000 = None
    print('  calculation error')

print('\nCalcaulate Pi 100,000 digit')
gc.collect()
mem()
try:
    d1 = pi_test(100000)
    d2 = pi_test(100000)
    d3 = pi_test(100000)
    r_pi_100000 = min(d1, d2, d3)
    print('100000 digit Pi calculation result: ', r_pi_100000, 's')
    mem()
except:
    r_pi_100000 = None
    print('  calculation error')

print('Test result:')
print('  Integer addition test result: ', r_int_add, 's')
print('  Float addition test result: ', r_float_add, 's')
print('  Integer multiplication test result: ', r_int_mul, 's')
print('  Float multiplication test result: ', r_float_mul, 's')
print('  Integer division test result: ', r_int_div, 's')
print('  Float division test result: ', r_float_div, 's')
if r_pi_1000:
    print('  1000 digit Pi calculation result: ', r_pi_1000, 's')
if r_pi_5000:
    print('  5000 digit Pi calculation result: ', r_pi_5000, 's')
if r_pi_100000:
    print('  100000 digit Pi calculation result: ', r_pi_100000, 's')
"""
# ----------------------------------------------------------------------------
