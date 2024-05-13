from machine import UART, Pin
import time

# Initialize UART1
uart1 = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

def read_gps():
    while True:
        if uart1.any():
            gps_data = uart1.readline()
            print(gps_data)
        time.sleep(0.1)

if __name__ == "__main__":
    print("Starting to read GPS data...")
    read_gps()
