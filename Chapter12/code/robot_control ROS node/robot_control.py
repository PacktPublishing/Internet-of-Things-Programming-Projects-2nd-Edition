import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

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
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vel_msg = Twist()
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.username_pw_set("mqtt-username", "mqtt-password")
        self.mqtt_client.connect("driver.cloudmqtt.com", port, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected successfully.")
            client.subscribe("JoystickPosition")
        else:
            print(f"Failed to connect with error code {rc}.")


    def on_message(self, client, userdata, msg):
        self.mqtt_message.update_values(msg.payload.decode())

    def timer_callback(self):
        if self.mqtt_message.button1:
            self.vel_msg.linear.x = 1.0
            self.vel_msg.angular.z = 1.0
        elif self.mqtt_message.button2:
            self.vel_msg.linear.x = -1.0
            self.vel_msg.angular.z = -1.0
        else:
            self.vel_msg.linear.x = float(self.mqtt_message.y)
            self.vel_msg.angular.z = float(self.mqtt_message.x)
        self.publisher.publish(self.vel_msg)


                
def main(args=None):
    rclpy.init(args=args)
    mqtt_message = MQTTMessage()
    robot_controller = RobotController(mqtt_message)
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

