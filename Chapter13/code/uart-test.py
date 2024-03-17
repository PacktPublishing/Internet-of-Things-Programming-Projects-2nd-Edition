import serial
import time
import subprocess

# Set permissions on /dev/serial0
subprocess.run(['sudo', 'chmod', 'a+rw', '/dev/serial0'], check=True)

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

def send_message(message):
    ser.write((message + "\n").encode())
    
send_message("a")