# Board version
from micropython import const
from robotling_lib.platform.platform import platform
if platform.ID == platform.ENV_ESP32_TINYPICO:
  BOARD_VER = 041
else:
  BOARD_VER = 042
