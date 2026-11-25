import pyaudio
import json
from vosk import Model, KaldiRecognizer
import time
# https://alphacephei.com/vosk/models
# https://dev.to/mattsu/vosk-offline-speech-recognition-3kbb


class VoskNode(object):
    def __init__(self, model_name="vosk-model-small-en-us-0.15"):
        self.model = Model(model_name)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.recognizer.SetWords(True)
        self.stream = None
        self.min_conf = 0.92

    def start_stream(self):
        p = pyaudio.PyAudio()
        self.stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
        self.stream.start_stream()

    def stop_stream(self):
        self.stream.stop_stream()
        self.stream = None

    def read(self):
        data = self.stream.read(4096, exception_on_overflow=False)
        if self.recognizer.AcceptWaveform(data):
            return  json.loads(self.recognizer.Result())
        return None

    def is_valid(self, vosk_res: dict) -> bool:
        text = vosk_res.get("text", "").strip()
        if not text: return False

        words = vosk_res.get("result", [])
        if not words: return len(text) >= 3

        avg_conf = sum(w.get("conf", 0) for w in words) / len(words)
        print(f"[Debug] 文字: '{text}' | 平均信心度: {avg_conf:.2f}")
        return avg_conf >= self.min_conf


if __name__ == "__main__":
    voice = VoskNode()
    voice.start_stream()
    while True:
        time.sleep(0.02)
        res = voice.read()
        if res is None: continue
        if not voice.is_valid(res): continue
        print(res)
        if "bye" in res["text"]:
            voice.stop_stream()
            break
