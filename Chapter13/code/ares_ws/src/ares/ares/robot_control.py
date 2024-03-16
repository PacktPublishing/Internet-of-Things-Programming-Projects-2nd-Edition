import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import serial
import board
import busio
import adafruit_vl53l0x
import time

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
        self.last_send_time = time.time()  # Track the last send time
        self.send_interval = 0.5  # Minimum time interval between sends in seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
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
        # Set the USB port for UART communication
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected with result code " + str(rc))
        client.subscribe("JoystickPosition")

    def on_message(self, client, userdata, msg):
        self.mqtt_message.update_values(msg.payload.decode())

    def timer_callback(self):
        current_time = time.time()
        # Check if the interval has passed
        if current_time - self.last_send_time >= self.send_interval:
            command = self.generate_command()
            if command:
                self.get_logger().info(f"Sent command: {command}")
                self.ser.write((command + "\n").encode())
                self.ser.flush()  # Ensure data is sent immediately
                self.last_send_time = current_time  # Update the last send time

    def generate_command(self):
        # Generate command based on mqtt_message and dist_sensor
        # Similar logic as previously in timer_callback to generate the 'command' string
        # Return the 'command' string
        pass  # Implement command generation logic here

def main(args=None):
    rclpy.init(args=args)
    mqtt_message = MQTTMessage()
    robot_controller = RobotController(mqtt_message)
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
