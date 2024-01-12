import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

class MQTTMessage:
    def __init__(self):
        self.should_draw_circle = False

    def set_flag(self, message):
        if message == 'draw_circle':
            self.should_draw_circle = True
        elif message == 'stop':
            self.should_draw_circle = False

class CircleMover(Node):
    def __init__(self, mqtt_message):
        super().__init__('circle_mover')
        self.mqtt_message = mqtt_message
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vel_msg = Twist()
        # Initialize MQTT Client and set up callbacks
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.username_pw_set("username", "password")
        self.mqtt_client.connect("driver.cloudmqtt.com", 18756, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe("move")

    def on_message(self, client, userdata, msg):
        self.mqtt_message.set_flag(msg.payload.decode())

    def timer_callback(self):
        if self.mqtt_message.should_draw_circle:
            self.vel_msg.linear.x = 1.0
            self.vel_msg.angular.z = 1.0
        else:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
        self.publisher.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    mqtt_message = MQTTMessage()
    circle_mover = CircleMover(mqtt_message)
    rclpy.spin(circle_mover)
    circle_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
