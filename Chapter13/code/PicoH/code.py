#Please note that code may be slightly different than the book due to code improvements
import board
import busio
import time
from wheel import Wheel
from device_alarm import Alarm

wheel = Wheel(20)
alarm = Alarm()
uart = busio.UART(board.GP4, board.GP5, baudrate=115200)

def clear_uart_buffer():
    """Clears any data from the UART buffer."""
    while uart.in_waiting > 0:
        uart.read(uart.in_waiting)

message_buffer = ''  # Initialize a buffer for accumulating message data

# Main loop
while True:
    data = uart.read(uart.in_waiting or 32)
    if data:
        message_buffer += data.decode('utf-8', 'ignore')

    while '<' in message_buffer and '>' in message_buffer:
        start_index = message_buffer.find('<') + 1
        end_index = message_buffer.find('>', start_index)
        message = message_buffer[start_index:end_index].strip()
        message_buffer = message_buffer[end_index+1:]
        print("Received:", message)
        
        # Handle the message using exact matches
        if message == 'f':
            print("Moving forward")
            wheel.forward()
        elif message == 'b':
            print("Moving in reverse")
            wheel.reverse()
        elif message == 'l':
            print("Left turn")
            wheel.turn_left()
        elif message == 'r':
            print("Right turn")
            wheel.turn_right()
        elif message == 'a':
            print("Alarm")
            wheel.stop()
            alarm.activate_alarm(2)
        elif message == 's':
            print("Stop")
            wheel.stop()

    time.sleep(0.1)  # Small delay to prevent overwhelming the CPU
