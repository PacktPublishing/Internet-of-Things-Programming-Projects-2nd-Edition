import usb_cdc
import time

while True:
    data = usb_cdc.data.read(32)  # Read up to 32 bytes from the USB serial
    if data:
        message = data.decode("utf-8")  # Decode bytes to string
        print("Received:", message)  # Echo the received message

        # Example: Toggle an LED or perform actions based on the message
        if "forward" in message:
            # Add your code to move motors forward
            print("Moving forward")
            usb_cdc.data.write(b"ACK\n")
        elif "backward" in message:
            # Add your code to move motors backward
            print("Moving backward")
            usb_cdc.data.write(b"ACK\n")
    
    time.sleep(0.1)  # Small delay to prevent overwhelming the CPU
