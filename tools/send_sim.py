#!/usr/bin/env python3
"""Send sim flight command (0xD0) to C.A.S.P.E.R.-2 via COBS-encoded CDC."""
import serial, sys, time

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
ser = serial.Serial(port, timeout=1)
time.sleep(0.5)
ser.write(bytes([0x02, 0xD0, 0x00]))  # COBS-encoded MSG_ID_SIM_FLIGHT
print(f"Sim flight command sent on {port}. Wait ~3 min for completion.")
ser.close()
