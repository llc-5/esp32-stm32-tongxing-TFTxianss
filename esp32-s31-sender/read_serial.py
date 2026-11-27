# -*- coding: utf-8 -*-
"""烧录后抓取串口启动日志（读 8 秒后自动退出）"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

ser = serial.Serial(PORT, 115200, timeout=0.2)
ser.setDTR(False)
ser.setRTS(True)   # EN low -> reset
time.sleep(0.1)
ser.setRTS(False)  # release reset
time.sleep(0.05)
try:
    ser.reset_input_buffer()
except Exception:
    pass

t0 = time.time()
buf = b""
while time.time() - t0 < SECONDS:
    buf += ser.read(4096)

ser.close()
sys.stdout.buffer.write(buf)
sys.stdout.flush()
