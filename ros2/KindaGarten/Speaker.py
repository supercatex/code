import wave
import threading
from playsound3 import playsound
from piper import PiperVoice, SynthesisConfig
# https://rhasspy.github.io/piper-samples/#zh_CN-xiao_ya-medium
# https://github.com/OHF-Voice/piper1-gpl/blob/main/docs/API_PYTHON.md
# python3 -m piper.download_voices en_US-lessac-medium


class SpeakerNode(object):
    def __init__(self, voice_name="en_US-lessac-medium", use_cuda=False):
        self.voice_name = voice_name
        self.voice = PiperVoice.load(
            model_path=f"../{self.voice_name}.onnx",
            use_cuda=use_cuda
        )
        self.is_playing = False
        self.stop_signal = False
        self.queue = []

    def speak(self, text, volume=1.0, scale=1.0, output_path="output.wav"):
        syn_config = SynthesisConfig(
            volume=volume,
            length_scale=scale,
            noise_scale=1.0,
            noise_w_scale=1.0,
            normalize_audio=False
        )
        with wave.open(output_path, "wb") as f:
            self.voice.synthesize_wav(text, f, syn_config=syn_config)

        sound_thread = threading.Thread(target=self.playsound, args=(output_path,))
        sound_thread.start()

    def playsound(self, output_path):
        self.is_playing = True
        sound = playsound(output_path, block=False)
        while sound.is_alive():
            if self.stop_signal:
                self.stop_signal = False
                sound.stop()
                break
        self.is_playing = False

    def stop(self):
        self.stop_signal = True


if __name__ == "__main__":
    speaker = SpeakerNode()
    speaker.speak("Hello! I am your assistant. What is your name?")
