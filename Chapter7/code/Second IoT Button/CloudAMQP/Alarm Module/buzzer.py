from machine import Pin, PWM
import utime

BUZZER_PIN = 16
buzzer = PWM(Pin(BUZZER_PIN))
BUZZER_FREQ = 4000

def activate_buzzer(duration=5):
    buzzer.freq(BUZZER_FREQ)
    buzzer.duty_u16(32768)
    utime.sleep(duration)
    buzzer.duty_u16(0)
