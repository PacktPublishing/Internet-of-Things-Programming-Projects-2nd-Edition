from machine import Pin, PWM
import utime

BUZZER_PIN = 16
buzzer = PWM(Pin(BUZZER_PIN))

def play_notes(
    notes = [
        (330, 0.5),  # E4 for 0.5 seconds
        (262, 0.5),  # C4 for 0.5 seconds
        (330, 0.5),  # E4 for 0.5 seconds
        (392, 0.5),  # G4 for 0.5 seconds
        (349, 0.5),  # F4 for 0.5 seconds
        (262, 1),  # C4 for 0.5 seconds
    ]
    ):
    
    for freq, duration in notes:
        buzzer.freq(freq)
        buzzer.duty_u16(32768)
        utime.sleep(duration)

    # Turn off the buzzer at the end of the melody
    buzzer.duty_u16(0)


