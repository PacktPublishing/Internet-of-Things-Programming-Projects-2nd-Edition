from m5stack import *
from m5ui import *
from uiflow import *
from m5mqtt import M5mqtt
import time

rgb.setColorAll(0x000000)

def cb_IoTAlarm(topic_data):
  if topic_data == 'motion':
    rgb.setColorAll(0xff0000)
    wait(5)
  rgb.setColorAll(0x00cccc)
  pass

def buttonA_pressFor():
  m5mqtt.publish(str('IoTAlarm'), str('buzzer'), 0)
  pass
btnA.pressFor(1, buttonA_pressFor)


m5mqtt = M5mqtt('IoTMatrix', 'codfish.rmq.cloudamqp.com', 1883, '', '', 300)

m5mqtt.subscribe(str('IoTAlarm'), cb_IoTAlarm)
rgb.setColorAll(0x00cccc)
m5mqtt.start()