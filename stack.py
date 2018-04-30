import serial
import struct
import sys

if len(sys.argv) > 1:
    ser = serial.Serial(sys.argv[1], 115200)

def stack(dist, pause_ms, nsteps):
    b = b">"
    b = b + struct.pack('<i', dist)
    b = b + struct.pack('<I', pause_ms)
    b = b + struct.pack('<H', nsteps)
    b = b + b";"
    return b

def ss(dist, pause, nsteps):
    ser.write(stack(dist, pause, nsteps))

def move(dist):
    ser.write(stack(dist, 0, 1))
