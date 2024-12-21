import whisper
import pyaudio
import numpy as np
import wave
import time
import noisereduce as nr
import os

class SpeechProcessor:
    def __init__(self, model_size="small", rate=44100, channels=1, width=2, chunk=4096, format=pyaudio.paInt16):
        self.model = whisper.load_model(model_size)
        self.RATE = rate
        self.CHANNELS = channels
        self.WIDTH = width
        self.CHUNK = chunk
        self.FORMAT = format
        self.p = pyaudio.PyAudio()

    def record_audio(self, duration, input_device_index, output_file):
        """
        Record audio for a specified duration and save to a file.
        """
        print("Recording...")
        stream = self.p.open(format=self.FORMAT,
                             channels=self.CHANNELS,
                             rate=self.RATE,
                             input=True,
                             input_device_index=input_device_index,
                             frames_per_buffer=self.CHUNK)
        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * duration)):
            data = stream.read(self.CHUNK)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        with wave.open(output_file, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.WIDTH)
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(frames))

        print(f"Recording finished. Saved to {output_file}")

    def reduce_noise(self, input_file, output_file):
        """
        Reduce noise from an audio file and save the result to a new file.
        """
        print("Reducing noise...")
        with wave.open(input_file, 'rb') as wf:
            num_channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            frame_rate = wf.getframerate()
            num_frames = wf.getnframes()
            audio_data = wf.readframes(num_frames)
            audio_np = np.frombuffer(audio_data, dtype=np.int16)

        reduced_noise_audio = nr.reduce_noise(y=audio_np, sr=frame_rate)

        with wave.open(output_file, 'wb') as wf:
            wf.setnchannels(num_channels)
            wf.setsampwidth(sample_width)
            wf.setframerate(frame_rate)
            wf.writeframes(reduced_noise_audio.astype(np.int16).tobytes())

        print(f"Noise reduction completed. Saved to {output_file}")

    def transcribe_audio(self, input_file, task="translate"):
        """
        Transcribe or translate audio using Whisper.
        """
        print("Transcribing and translating...")
        result = self.model.transcribe(input_file, task=task)
        return result["text"]

    def __del__(self):
        self.p.terminate()

PATH_TEMP_FILE = './temp_files'

def main():
    # 設定檔案路徑
    raw_audio_path = PATH_TEMP_FILE + "/audio.wav"
    denoised_audio_path = PATH_TEMP_FILE + "/denoised_audio.wav"
    text_path = PATH_TEMP_FILE + "/translated_text.txt"

    # 初始化處理器
    processor = SpeechProcessor()

    # 設定錄音參數
    duration = 5  # 錄音持續時間
    input_device_index = 11  # 麥克風設備索引

    # 錄音
    processor.record_audio(duration, input_device_index, raw_audio_path)

    # 去噪
    processor.reduce_noise(raw_audio_path, denoised_audio_path)

    # 語音轉譯
    translated_text = processor.transcribe_audio(denoised_audio_path, task="translate")
    with open(text_path, "w") as f:
        f.write(translated_text)
    print("Translated Text: ", translated_text)


if __name__ == "__main__":
    main()
