import serial
import time

cmd = bytes.fromhex("01 03 00 00 00 01 84 0A")
ser = serial.Serial("/dev/ttyUSB1", 9600, bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.1)

while True:
    ser.write(cmd)
    time.sleep(0.1)
    data = ser.read_all()
    print("Recv:", data.hex())
    time.sleep(1)
