import serial
import subprocess

# Embedding the admin password (not recommended for production use)
password = 'oFc2327'
command = 'chmod a+rw /dev/serial0'

# Using echo to send the password to sudo -S
subprocess.run(f'echo {password} | sudo -S {command}', shell=True, check=True)

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

def send_message(message):
    ser.write((message + "\n").encode())
    
send_message("a")
