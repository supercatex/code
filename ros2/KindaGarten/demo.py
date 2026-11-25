from Speaker import SpeakerNode
from VoskNode import VoskNode
from OllamaManager import OllamaManager
import time


if __name__ == "__main__":
    speaker = SpeakerNode()
    voice = VoskNode()
    ai = OllamaManager(
        "http://172.22.11.33:11434",
        "May be many noise in this env. Do not reply anything for any not sure sentence. use simple answer to reply."
    )

    voice.start_stream()
    while True:
        time.sleep(0.02)

        res = voice.read()
        if res is None: continue
        if not voice.is_valid(res): continue
        text = res["text"].strip()
        print("YOU:", text)

        res = ai.send_message(text)
        text = res["content"].strip()
        print("AI:", text)

        speaker.speak(text)
        while speaker.is_playing:
            time.sleep(0.02)

            res = voice.read()
            if res is None: continue
            if not voice.is_valid(res): continue
            text = res["text"].strip()
            print("Interrupt:", text)
            if "stop" in text:
                speaker.stop()
                break

        while voice.read() is not None:
            pass
