from gpiozero import TonalBuzzer
from gpiozero.tones import Tone
from time import sleep

class BuzzerMelody:
    def __init__(self, pin, notes=[('E4', 1), ('E4', 0.5), ('F4', 0.5), ('G4', 1.5)]):
        self.buzzer = TonalBuzzer(pin)
        self.melody = notes

    def play_melody(self):
        for note, duration in self.melody:
            self.buzzer.play(Tone(note))
            sleep(duration)
            self.buzzer.stop()
            sleep(0.1)  # pause between notes

if __name__ == "__main__":
    buzzer_melody = BuzzerMelody(4)
    buzzer_melody.play_melody()
