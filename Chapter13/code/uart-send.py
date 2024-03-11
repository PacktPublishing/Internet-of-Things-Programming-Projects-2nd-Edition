import serial
import time

# Open serial connection
ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # Adjust port and baud rate as needed

def send_message(message):
    ser.write((message + "\n").encode())  # Send message
    
# Example usage
send_message("f")
time.sleep(2)
send_message("a")
time.sleep(2)