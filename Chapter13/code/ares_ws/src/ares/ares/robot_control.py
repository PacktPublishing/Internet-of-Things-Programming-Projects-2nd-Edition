import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import serial
import board
import busio
import adafruit_vl53l0x
import time
import subprocess

class MQTTMessage:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.button1 = False
        self.button2 = False

    def update_values(self, message):
        parts = message.split(', ')
        self.x = float(parts[0].split(': ')[1])
        self.y = float(parts[1].split(': ')[1])
        self.button1 = parts[2].split(': ')[1].strip() == "True"
        self.button2 = parts[3].split(': ')[1].strip() == "True"

class RobotController(Node):
    def __init__(self, mqtt_message):
        super().__init__('robot_controller')
        self.mqtt_message = mqtt_message
        # Create I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        # Create VL53L0X object
        self.dist_sensor = adafruit_vl53l0x.VL53L0X(i2c)
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.username_pw_set("tvuuvbox", "Nqe3McF21AjF")
        self.mqtt_client.connect("driver.cloudmqtt.com", 18756, 60)
        self.mqtt_client.loop_start()
        
        # Embedding the admin password (not recommended for production use)
        password = 'oFc2327'
        command = 'chmod a+rw /dev/serial0'

        # Using echo to send the password to sudo -S
        subprocess.run(f'echo {password} | sudo -S {command}', shell=True, check=True)

        # Set the USB port for UART communication
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        self.ack_received = True

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected with result code {rc}")
        client.subscribe("JoystickPosition")

    def on_message(self, client, userdata, msg):
        self.mqtt_message.update_values(msg.payload.decode())

    def send_message(self, command):
        if self.ack_received:
            time.sleep(1)
            self.ser.write(command.encode())
            self.ser.flush()
            self.ack_received = False  # Reset ACK status awaiting next ACK.
            self.get_logger().info(f"Sent command: {command.strip()}")
            while not self.ack_received:
                if self.ser.in_waiting > 0:
                    response = self.ser.read(self.ser.in_waiting).decode()
                    if "ACK" in response:
                        self.ack_received = True
                        self.get_logger().info("ACK received")

    def timer_callback(self):
        command = self.generate_command()
        if command:
            self.send_message(command)

    def generate_command(self):
        command = ''
        # Forward or stop based on distance and y-axis input
        if self.mqtt_message.y > 0:
            command = 'f' if self.dist_sensor.range > 100 else 's'
        elif self.mqtt_message.y < 0:
            command = 'b'
        # Right or left based on x-axis input
        if self.mqtt_message.x > 0:
            command = 'r'
        elif self.mqtt_message.x < 0:
            command = 'l'
        # Stop if there's no x or y input
        if self.mqtt_message.y == 0 and self.mqtt_message.x == 0:
            command = 's'
        # Activate alarm if button1 is pressed
        if self.mqtt_message.button1:
            command = 'a'
        return command + '\n'

def main(args=None):
    rclpy.init(args=args)
    mqtt_message = MQTTMessage()
    robot_controller = RobotController(mqtt_message)
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
