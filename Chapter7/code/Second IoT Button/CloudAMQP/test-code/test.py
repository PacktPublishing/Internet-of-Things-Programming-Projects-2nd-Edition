import machine
import time
import network
from umqtt.simple import MQTTClient

# Configure your WiFi SSID and password
ssid = 'SSID'
password = 'XXXXXXXXXXXX'

# Connect to WiFi
def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print('Network config:', wlan.ifconfig())

# MQTT settings
mqtt_server = 'codfish.rmq.cloudamqp.com'
mqtt_port = 1883  # Default MQTT port
mqtt_user = 'mqtt_username'
mqtt_password = 'password'
client_id = 'IoTAlarm'

# Connect to MQTT server
def connect_to_mqtt():
    client = MQTTClient(client_id, mqtt_server, mqtt_port, mqtt_user, mqtt_password)
    client.connect()
    print('Connected to %s MQTT broker' % (mqtt_server))
    return client

# Publish a message
def publish_message(client):
    topic = 'test/topic'
    message = 'Hello Pico W'
    client.publish(topic, message)
    print('Published %s to %s' % (message, topic))

# Main function
def main():
    connect_to_wifi()
    mqtt_client = connect_to_mqtt()
    publish_message(mqtt_client)
    # Ensure to delay before ending to allow publish to complete
    time.sleep(2)

# Run the main function
if __name__ == '__main__':
    main()
