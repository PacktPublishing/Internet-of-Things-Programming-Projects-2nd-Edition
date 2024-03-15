import board
import busio
import time
from wheel import Wheel
from device_alarm import Alarm

robotics_board = KitronikPicoRobotics()
wheel = Wheel(robotics_board, 20)
alarm = Alarm()
uart = busio.UART(board.GP4, board.GP5, baudrate=115200)

def clear_uart_buffer():
    """Clears any data from the UART buffer."""
    while uart.in_waiting > 0:
        uart.read(uart.in_waiting)

# Main loop
while True:
    data = uart.read(32)  # Read up to 32 bytes
    if data is not None:
        message = ''.join([chr(b) for b in data])  # Convert bytes to string
        print("Received:", message)
        
        # Handle the message
        if "f" in message:
            print("Moving forward")
            wheel.forward()
        elif "b" in message:
            print("Moving in reverse")
            wheel.reverse()
        elif "l" in message:
            print("Left turn")
            wheel.turn_left()
        elif "r" in message:
            print("Right turn")
            wheel.turn_right()
        elif "a" in message:
            print("Alarm")
            wheel.stop()
            alarm.activate_alarm(2)
        else:
            print("Stop")
            wheel.stop()

        # Clear any leftover data in the UART buffer
        clear_uart_buffer()
        
    time.sleep(0.1)  # Small delay to prevent overwhelming the CPU
