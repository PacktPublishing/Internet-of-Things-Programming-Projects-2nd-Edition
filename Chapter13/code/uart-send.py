import serial
import time

# Open serial connection
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Adjust port and baud rate as needed

def send_message(message):
    ser.write((message + "\n").encode())  # Send message
    ack = ser.readline().decode().strip()  # Wait for ACK
    print(ack)
    if ack == "ACK":
        print("Message acknowledged by Pico")
    else:
        print("Message not acknowledged, resending...")
        send_message(message)  # Resend message if not acknowledged

# Example usage
send_message("ffffffffffffff")
time.sleep(2)
send_message("ssssssssssssss")