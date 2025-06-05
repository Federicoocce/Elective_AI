import os
import time
import numpy as np
import threading
import json
import traceback
from pynput import keyboard as pynput_keyboard
import sounddevice as sd
from faster_whisper import WhisperModel
from RealtimeTTS import TextToAudioStream, EdgeEngine # SystemEngine (if you switch back)
from scipy.io import wavfile

class SpeechInterface:
    def __init__(self,
                 offline_stt_model_name="medium",
                 offline_stt_device="cpu",
                 offline_stt_compute_type="int8",
                 sample_rate=16000,
                 channels=1,
                 tts_voice_name="en-GB-RyanNeural",
                 start_record_key_str='<ctrl>+<alt>+r',
                 stop_record_key_str='<ctrl>+<alt>+s'
                 ):
        self._scripted_inputs = []
        self._current_input_idx = 0
        self._script_mode_active = False
        print("SpeechInterface: Initializing with hotkeys, offline STT (direct audio), and RealtimeTTS...")
        self.sample_rate = sample_rate
        self.channels = channels
        self.is_recording = False
        self.recorded_frames = [] # Initialize here
        self.stream = None # Initialize stream attribute
        self.hotkey_listener = None # Initialize hotkey_listener attribute

        self.stt_model = WhisperModel(offline_stt_model_name, device=offline_stt_device, compute_type=offline_stt_compute_type)
        engine = EdgeEngine()
        engine.set_voice(tts_voice_name)
        self.tts_stream_realtimetts = TextToAudioStream(engine)
        self.start_record_key_str = start_record_key_str
        self.stop_record_key_str = stop_record_key_str

    def _audio_callback_sd(self, indata, frames, time, status):
        if status:
            print(f"Sounddevice status: {status}", flush=True)
        if self.is_recording:
            # indata is already float32 and normalized by sounddevice
            self.recorded_frames.append(indata.copy())

    def _start_recording_sounddevice(self):
        self.is_recording = True
        self.recorded_frames = [] # Clear previous frames
        print(f"Starting recording with SR={self.sample_rate}, Channels={self.channels}")
        self.stream = sd.InputStream(samplerate=self.sample_rate,
                                     channels=self.channels,
                                     callback=self._audio_callback_sd,
                                     dtype='float32') # Explicitly float32
        self.stream.start()
        print("Recording started...")

    def _stop_recording_and_process(self):
        self.is_recording = False
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None
        else:
            print("Warning: Stop called but stream was not active.")
            return "Error: Stream not active or already closed."

        if not self.recorded_frames:
            print("No audio frames recorded.")
            return "" # Or an appropriate message

        print("Recording stopped. Processing audio...")
        # Data from sounddevice is float32, normalized to [-1.0, 1.0]
        # Rename variable for clarity
        recording_data_float32 = np.concatenate(self.recorded_frames, axis=0)
        self.recorded_frames = [] # Clear for next recording

        filename = os.path.join("./", f"recording_scipy_{int(time.time())}.wav") # Add timestamp to avoid overwrite
        print(f"SAVING (SciPy): Attempting to save audio to {filename}")
        # scipy.io.wavfile.write will handle float32 input correctly,
        # typically by scaling to int16 if saving a standard PCM WAV.
        wavfile.write(filename, self.sample_rate, recording_data_float32)
        print(f"Audio saved to {filename}. Shape: {recording_data_float32.shape}, dtype: {recording_data_float32.dtype}")
        print(f"Audio data min: {np.min(recording_data_float32)}, max: {np.max(recording_data_float32)}")


        # --- Transcribe directly from NumPy array ---
        transcribed_text = self._transcribe_audio_data_offline(recording_data_float32)
        return transcribed_text

    def _transcribe_audio_data_offline(self, audio_data_float32_np):
        # audio_data_float32_np is already float32 and normalized from sounddevice
        # No need to divide by 32768.0

        # Ensure it's float32 (should already be, but good practice)
        audio_to_transcribe = audio_data_float32_np.astype(np.float32)

        if audio_to_transcribe.ndim > 1:
            # If stereo (e.g. (N,2)), convert to mono by averaging or taking one channel
            # For (N,1) from sounddevice with channels=1, flatten is fine.
            if audio_to_transcribe.shape[1] > 1: # Check if it's truly multi-channel
                 print(f"Warning: Multi-channel audio detected (shape {audio_to_transcribe.shape}). Converting to mono by averaging.")
                 audio_to_transcribe = np.mean(audio_to_transcribe, axis=1)
            else: # Shape is (N,1)
                 audio_to_transcribe = audio_to_transcribe.flatten()


        print(f"Audio for transcription: shape={audio_to_transcribe.shape}, dtype={audio_to_transcribe.dtype}, min={np.min(audio_to_transcribe):.4f}, max={np.max(audio_to_transcribe):.4f}")

        if np.max(np.abs(audio_to_transcribe)) < 0.001: # Check if audio is effectively silent
            print("Warning: Audio signal appears to be very quiet or silent. Transcription might be poor.")

        segments, info = self.stt_model.transcribe(audio_to_transcribe, beam_size=5, language="en")
        
        segment_list = list(segments) # Consume the generator
        if not segment_list:
            print("Whisper returned no segments.")
            text = ""
        else:
            print(f"Detected language: {info.language} with probability {info.language_probability:.2f}")
            print("Transcription Segments:")
            for i, segment in enumerate(segment_list):
                print(f"  [{segment.start:.2f}s -> {segment.end:.2f}s] {segment.text}")
            text = "".join(segment.text for segment in segment_list).strip()
        
        print(f"STT (Offline Result from audio data): '{text}'")
        return text

    def _on_press_start_record(self):
        if not self.is_recording:
            self._start_recording_sounddevice()
        else:
            print("Already recording, ignoring start request.")

    def _on_press_stop_record(self):
        if self.is_recording:
            transcribed_text = self._stop_recording_and_process()
            print(f"Transcribed text from hotkey stop: {transcribed_text}")
            # You might want to do something with this text, e.g., self.say(f"I heard: {transcribed_text}")
        else:
            print("Not currently recording, ignoring stop request.")

    def set_script(self, inputs_list):
        self._scripted_inputs = list(inputs_list)
        self._current_input_idx = 0
        self._script_mode_active = bool(self._scripted_inputs)
        mode_str = "ENABLED" if self._script_mode_active else "DISABLED"
        print(f"SPEECH INTERFACE: Script Mode {mode_str}.")
        if self._script_mode_active: print("  Hotkeys and real mic/TTS will be bypassed.")

    def say(self, text_to_say):
        # Logic for scripted inputs
        effective_text_to_say = text_to_say
        if self._script_mode_active:
            if self._current_input_idx < len(self._scripted_inputs["say"]): # Assuming script is dict {"say": [], "listen": []}
                effective_text_to_say = self._scripted_inputs["say"][self._current_input_idx]
                # _current_input_idx should be managed by listen_and_get_text for pairing
            else:
                print("No more scripted 'say' inputs available, using provided text.")

        if self.is_recording:
            print("Warning: 'say' called while recording. Stopping recording first.")
            self._stop_recording_and_process() # This will transcribe, might not be desired here.
                                               # Consider just stopping without processing if say interrupts.

        print(f"Speaking: {effective_text_to_say}")
        self.tts_stream_realtimetts.feed(effective_text_to_say)
        self.tts_stream_realtimetts.play_async() # Use play_async for non-blocking play

    def listen_and_get_text(self):
        if self._script_mode_active:
            # Assuming script is dict {"say": [], "listen": []} and listen returns pre-defined text
            if self._current_input_idx < len(self._scripted_inputs["listen"]):
                text = self._scripted_inputs["listen"][self._current_input_idx]
                print(f"Script mode: Returning '{text}'")
                self._current_input_idx += 1
                return text
            else:
                print("No more scripted 'listen' inputs available, returning empty string.")
                self._script_mode_active = False # Or loop, or error
                return ""

        # This method seems designed for a different flow (not hotkey based)
        # For hotkey based, transcription happens in _on_press_stop_record
        # If you intend listen_and_get_text to also control recording:
        if not self.is_recording:
            self._start_recording_sounddevice()
            print("Listening... (Use stop hotkey or another mechanism to get text)")
            # This function would then need to block or have a way to retrieve text later.
            # For now, it's not clear how it fits with hotkey flow.
            return "" # Placeholder, as transcription is tied to hotkey stop
        else:
            print("Already recording. Text will be processed on stop hotkey.")
            return ""


    def wait_for_wake_word(self, wake_phrase="Hey Tiago"):
        if self._script_mode_active:
            print(f"WAKE WORD (Scripted Bypass): Proceeding.")
            return True
        print("WAKE WORD: Not applicable in hotkey-driven recording mode. Proceeding conceptually.")
        return True

    def start_listening_for_hotkeys(self):
        if self.hotkey_listener:
            print("Hotkey listener already running.")
            return
        print("Starting global hotkey listener...")
        hotkey_actions = {
            self.start_record_key_str: self._on_press_start_record,
            self.stop_record_key_str: self._on_press_stop_record
        }
        self.hotkey_listener = pynput_keyboard.GlobalHotKeys(hotkey_actions)
        self.hotkey_listener.start()
        print(f"Global hotkey listener started. Start: '{self.start_record_key_str}', Stop: '{self.stop_record_key_str}'")

    def stop_listening_for_hotkeys(self):
        if self.hotkey_listener:
            print("Stopping global hotkey listener...")
            self.hotkey_listener.stop()
            self.hotkey_listener.join() # Wait for listener thread to finish
            self.hotkey_listener = None
            print("Global hotkey listener stopped.")
        else:
            print("Hotkey listener not running.")

    def shutdown(self):
        print("Shutting down SpeechInterface...")
        self.stop_listening_for_hotkeys()
        if self.is_recording:
            print("Recording was active during shutdown, stopping and processing...")
            self._stop_recording_and_process() # Process any pending recording

        if hasattr(self, 'tts_stream_realtimetts'):
            self.tts_stream_realtimetts.stop() # Ensure TTS is stopped
        print("SpeechInterface shutdown complete.")

if __name__ == '__main__':
    print("--- SpeechInterface Test (Hotkey, Offline STT direct audio, RealtimeTTS) ---")

    OFFLINE_STT_MODEL = "medium" # "large-v3" is very heavy for CPU, start with "base", "small", or "medium"
    OFFLINE_STT_DEVICE = "cpu"
    OFFLINE_STT_COMPUTE = "int8" # "float16" might be better if you have GPU, or "auto"
    AUDIO_SAMPLE_RATE = 16000
    TTS_VOICE = "en-GB-RyanNeural"
    START_KEY = "<ctrl>+<alt>+r"
    STOP_KEY = "<ctrl>+<alt>+s"

    print("\n=== INITIALIZING SPEECH INTERFACE ===")
    speech_module = None
    try:
        speech_module = SpeechInterface(
            offline_stt_model_name=OFFLINE_STT_MODEL,
            offline_stt_device=OFFLINE_STT_DEVICE,
            offline_stt_compute_type=OFFLINE_STT_COMPUTE,
            sample_rate=AUDIO_SAMPLE_RATE,
            tts_voice_name=TTS_VOICE,
            start_record_key_str=START_KEY,
            stop_record_key_str=STOP_KEY
        )

        speech_module.start_listening_for_hotkeys()
        speech_module.say("System initialized. Press Control Alt R to begin recording, and Control Alt S to end and transcribe.")

        print("\nApplication is now running. Use hotkeys to interact.")
        print(f"  Configured Start Recording Hotkey: {START_KEY}")
        print(f"  Configured Stop & Transcribe Hotkey: {STOP_KEY}")
        print("Press Ctrl+C in this terminal to quit the main application.")

        # Keep the main thread alive
        while True:
            time.sleep(1)
    except Exception as e:
        print(f"An error occurred during initialization or runtime: {e}")
        traceback.print_exc()
    except KeyboardInterrupt:
        print("\nCtrl+C received. Shutting down...")
    finally:
        if speech_module:
            speech_module.shutdown()

    print("--- SpeechInterface Test Finished ---")