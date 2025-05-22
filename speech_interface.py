# speech_interface.py

class SpeechInterface:
    """
    Mock interface for Speech-to-Text (STT) and Text-to-Speech (TTS).
    In a real system, this would integrate with actual STT/TTS engines.
    """
    def __init__(self):
        self._scripted_inputs = []
        self._current_input_idx = 0
        print("SpeechInterface initialized (Mock Mode).")

    def set_script(self, inputs_list):
        """Pre-loads a list of user responses for automated testing."""
        self._scripted_inputs = inputs_list
        self._current_input_idx = 0
        print(f"SPEECH: Script loaded with {len(inputs_list)} inputs.")

    def wait_for_wake_word(self):
        """Simulates waiting for a wake word."""
        print("SPEECH (TTS): Waiting for wake word... (Mocked: Proceeding)")
        # In a real system, this would block until the wake word is detected.
        pass

    def say(self, text_to_say):
        """Simulates the robot speaking."""
        print(f"🤖 ROBOT SAYS (TTS Mock): {text_to_say}")
        # In a real system, this would send `text_to_say` to a TTS engine.

    def listen_and_get_text(self):
        """
        Simulates listening to the user and returning transcribed text.
        Uses scripted input if available, otherwise prompts for manual input.
        """
        if self._current_input_idx < len(self._scripted_inputs):
            user_response = self._scripted_inputs[self._current_input_idx]
            print(f"🗣️ USER SAYS (STT Mock - Scripted): {user_response}")
            self._current_input_idx += 1
            return user_response
        else:
            # Fallback to manual input if script runs out or was never set
            try:
                user_response = input("🗣️ USER SAYS (STT Mock - Manual Input): ")
                return user_response
            except EOFError: # Handle cases where input stream is closed (e.g., piped input)
                 print("SPEECH: EOFError on input, returning 'stop'.")
                 return "stop"


    def shutdown(self):
        print("SPEECH: SpeechInterface shutdown (Mock Mode).")

if __name__ == '__main__':
    print("--- Testing SpeechInterface ---")
    speech_module = SpeechInterface()

    # Test with scripted input
    test_script = [
        "Hello robot",
        "I'm looking for a red dress.",
        "Yes, that sounds good.",
        "Thank you!"
    ]
    speech_module.set_script(test_script)

    speech_module.wait_for_wake_word()
    speech_module.say("Hello! How can I help you today?")
    
    response1 = speech_module.listen_and_get_text()
    speech_module.say(f"You said: {response1}. Searching now...")
    
    response2 = speech_module.listen_and_get_text()
    speech_module.say(f"Regarding '{response2}', I found some options.")

    response3 = speech_module.listen_and_get_text()
    speech_module.say("Great!")

    response4 = speech_module.listen_and_get_text()
    speech_module.say("You're welcome!")

    # Test with manual input if script is exhausted
    speech_module.say("What else can I do for you? (Manual input now)")
    manual_response = speech_module.listen_and_get_text()
    speech_module.say(f"You manually entered: {manual_response}")
    
    speech_module.shutdown()