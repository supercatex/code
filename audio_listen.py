import pyaudio
import numpy as np
import wave
import speech_recognition as sr


FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 2048  # RATE / number of updates per second


if __name__ == "__main__":
    r = sr.Recognizer()
    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )

    frames = []
    terminate = False
    flag = 0
    while not terminate:
        data = stream.read(CHUNK, exception_on_overflow=False)
        wave_data = wave.struct.unpack("%dh" % CHUNK, data)
        array_data = np.array(wave_data)
        if flag > 0:
            frames.append(data)
        if np.max(np.abs(array_data)) > 8000:
            flag = 10
            print("Recording...", flag)
        if flag > 0 and np.max(np.abs(array_data)) <= 3000:
            flag -= 1
            print("Recording...", flag)
            if flag == 0:
                wf = wave.open("my_wav.wav", 'wb')
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(p.get_sample_size(FORMAT))
                wf.setframerate(RATE)
                wf.writeframes(b''.join(frames))
                wf.close()
                frames = []

                with sr.AudioFile("my_wav.wav") as source:
                    audio = r.record(source)
                try:
                    print("Recognizing...")
                    text = r.recognize_google(audio, language="yue-Hant-HK")
                    print(text)
                    if text == "再見":
                        break
                except sr.UnknownValueError:
                    print("Voice Recognition could not understand audio")
                except sr.RequestError as e:
                    print("Could not request results from Voice Recognition service; {0}".format(e))

    stream.stop_stream()
    stream.close()
    p.terminate()
