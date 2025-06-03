# speech_interface.py
import time
import random

# Forward declaration for type hinting if AutomatedBenchmarkingLLM is in a separate file
# and you want to avoid circular imports for type checking.
# from typing import TYPE_CHECKING
# if TYPE_CHECKING:
# from automated_benchmarking_llm import AutomatedBenchmarkingLLM

class SpeechInterface:
    """
    Mock interface for Speech-to-Text (STT) and Text-to-Speech (TTS).
    Integrates with AutomatedBenchmarkingLLM for simulated user input.
    """
    def __init__(self):
        self._scripted_inputs = []
        self._current_input_idx = 0
        self.automated_llm_user = None # Will hold instance of AutomatedBenchmarkingLLM
        self.last_robot_utterance_for_auto_user = "" # Context for auto user
        print("SpeechInterface initialized (Mock Mode).")

    def set_script(self, inputs_list):
        """Pre-loads a list of user responses for automated testing."""
        self._scripted_inputs = inputs_list
        self._current_input_idx = 0
        print(f"SPEECH: Script loaded with {len(inputs_list)} inputs.")

    def set_automated_llm_user(self, llm_user_instance, knowledge_base_entities):
        """
        Sets the AutomatedBenchmarkingLLM instance to be used for generating user responses.
        llm_user_instance: An instance of AutomatedBenchmarkingLLM.
        knowledge_base_entities: A dict containing lists of known entities for the LLM.
        """
        self.automated_llm_user = llm_user_instance
        if self.automated_llm_user:
            # Pass the knowledge base to the automated LLM user
            self.automated_llm_user.set_knowledge_base(
                item_categories=knowledge_base_entities.get("item_categories", []),
                brands=knowledge_base_entities.get("brands", []),
                colors=knowledge_base_entities.get("colors", []),
                store_names=knowledge_base_entities.get("store_names", [])
            )
            print("SPEECH: Automated LLM User has been set and configured.")
        else:
            print("SPEECH: Automated LLM User cleared.")


    def wait_for_wake_word(self):
        """Simulates waiting for a wake word."""
        print("SPEECH (TTS): Waiting for wake word... (Mocked: Proceeding)")
        pass

    def say(self, text_to_say):
        """Simulates the robot speaking."""
        print(f" ROBOT SAYS (TTS Mock): {text_to_say}")
        self.last_robot_utterance_for_auto_user = text_to_say # Store for context

    def listen_and_get_text(self, dialogue_state_hint=None, context_for_auto_llm=None):
        """
        Simulates listening to the user and returning transcribed text.
        Uses automated LLM user if set, then scripted input, then manual console input.
        dialogue_state_hint: String hint for AutomatedBenchmarkingLLM about the current dialogue state.
        context_for_auto_llm: Dictionary with relevant data for AutomatedBenchmarkingLLM.
        """
        if self.automated_llm_user:
            time.sleep(random.uniform(0.3, 0.8)) # Shorter delay for faster non-ROS tests
            
            print(f"SPEECH: AutoLLMUser turn. Robot last said: \"{self.last_robot_utterance_for_auto_user}\". Hint: {dialogue_state_hint}")
            user_response = "I am not sure how to respond to that." # Default fallback

            if dialogue_state_hint == "initial_speaker_id":
                user_response = self.automated_llm_user.generate_initial_speaker_id()
            elif dialogue_state_hint == "initial_query":
                user_response = self.automated_llm_user.generate_initial_query()
            elif dialogue_state_hint == "clarify_shopping_target" and context_for_auto_llm:
                user_response = self.automated_llm_user.respond_to_clarify_shopping_target(
                    robot_query_summary=context_for_auto_llm.get("robot_query_summary", "your request")
                )
            elif dialogue_state_hint == "confirm_visit_store" and context_for_auto_llm:
                user_response = self.automated_llm_user.decide_to_visit_store(
                    store_name_suggested=context_for_auto_llm.get("store_name", "that store"),
                    item_context_summary=context_for_auto_llm.get("item_summary", "your item")
                )
            elif dialogue_state_hint == "acknowledge_arrival_readiness":
                user_response = self.automated_llm_user.acknowledge_arrival_and_readiness()
            elif dialogue_state_hint == "provide_fulfillment_feedback" and context_for_auto_llm:
                user_response = self.automated_llm_user.provide_fulfillment_feedback(
                    item_context_summary=context_for_auto_llm.get("item_summary", "the item"),
                    store_name=context_for_auto_llm.get("store_name", "this store")
                )
            elif dialogue_state_hint == "provide_general_store_feedback" and context_for_auto_llm:
                user_response = self.automated_llm_user.provide_general_store_feedback(
                     store_name=context_for_auto_llm.get("store_name", "this store")
                )
            elif dialogue_state_hint == "decide_next_action" and context_for_auto_llm:
                user_response = self.automated_llm_user.decide_next_action(
                    original_request_summary=context_for_auto_llm.get("original_request_summary", "your previous request"),
                    was_last_item_fulfilled=context_for_auto_llm.get("was_last_item_fulfilled", False),
                    has_more_options_for_current=context_for_auto_llm.get("has_more_options", False)
                )
            elif dialogue_state_hint == "respond_to_anything_else":
                user_response = self.automated_llm_user.respond_to_anything_else()
            elif dialogue_state_hint == "general_clarification": # A generic fallback for LLM
                user_response = "Could you rephrase that please?" # Or make LLM generate this
            
            # Fallback if no specific hint matched but auto LLM is active
            elif dialogue_state_hint is None or dialogue_state_hint == "general_input":
                 # A general prompt to the auto LLM user could be added here if needed
                 # For now, just use its default "I'm not sure" or a very generic response.
                 print(f"SPEECH_WARN: AutoLLMUser active but no specific hint ('{dialogue_state_hint}') matched. Using default response.")


            print(f" AUTO USER SAYS (LLM Mock): {user_response}")
            return user_response

        # Fallback to scripted or manual input if Automated LLM User is not set
        if self._current_input_idx < len(self._scripted_inputs):
            user_response = self._scripted_inputs[self._current_input_idx]
            print(f" USER SAYS (STT Mock - Scripted): {user_response}")
            self._current_input_idx += 1
            return user_response
        else:
            try:
                user_response = input("USER SAYS (STT Mock - Manual Input): ")
                return user_response
            except EOFError:
                 print("SPEECH: EOFError on input, returning 'stop'.")
                 return "stop"
            except KeyboardInterrupt:
                 print("SPEECH: KeyboardInterrupt during manual input, returning 'stop'.")
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