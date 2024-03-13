import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

def send_message(message):
    ser.write((message + "\n").encode())
    
send_message("a")