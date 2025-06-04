# speech_interface.py
import time
import random

class SpeechInterface:
    """
    Mock interface for Speech-to-Text (STT) and Text-to-Speech (TTS).
    Integrates with AutomatedBenchmarkingLLM for simulated user input.
    """
    def __init__(self):
        self._scripted_inputs = []
        self._current_input_idx = 0
        self.automated_llm_user = None 
        self.last_robot_utterance_for_auto_user = "" 
        print("SpeechInterface initialized (Mock Mode).")

    def set_script(self, inputs_list):
        self._scripted_inputs = inputs_list
        self._current_input_idx = 0
        print(f"SPEECH: Script loaded with {len(inputs_list)} inputs.")

    def set_automated_llm_user(self, llm_user_instance, knowledge_base_entities):
        self.automated_llm_user = llm_user_instance
        if self.automated_llm_user:
            self.automated_llm_user.set_knowledge_base(
                item_categories=knowledge_base_entities.get("item_categories", []),
                brands=knowledge_base_entities.get("brands", []),
                colors=knowledge_base_entities.get("colors", []),
                store_names=knowledge_base_entities.get("store_names", []),
                sample_products=knowledge_base_entities.get("sample_products", []) # Pass sample_products
            )
            print("SPEECH: Automated LLM User has been set and configured.")
        else:
            print("SPEECH: Automated LLM User cleared.")


    def wait_for_wake_word(self):
        print("SPEECH (TTS): Waiting for wake word... (Mocked: Proceeding)")
        pass

    def say(self, text_to_say):
        print(f" ROBOT SAYS (TTS Mock): {text_to_say}")
        self.last_robot_utterance_for_auto_user = text_to_say 

    def listen_and_get_text(self, dialogue_state_hint=None, context_for_auto_llm=None):
        if self.automated_llm_user:
            time.sleep(random.uniform(0.3, 0.8)) 
            
            print(f"SPEECH: AutoLLMUser turn. Robot last said: \"{self.last_robot_utterance_for_auto_user}\". Hint: {dialogue_state_hint}")
            user_response = "I am not sure how to respond to that." 

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
            elif dialogue_state_hint == "general_clarification": 
                user_response = self.automated_llm_user._call_groq_api_for_simulation( # Let LLM generate a generic clarification
                    f"You are impersonating a '{self.automated_llm_user.persona}'. The robot said something you didn't fully understand or needs rephrasing. Respond naturally asking for clarification or rephrasing.", max_tokens=20
                ) if self.automated_llm_user.client else "Could you rephrase that please?" # Mock if no client
            
            elif dialogue_state_hint is None or dialogue_state_hint == "general_input":
                 print(f"SPEECH_WARN: AutoLLMUser active but no specific hint ('{dialogue_state_hint}') matched. Using default response.")


            print(f" AUTO USER SAYS (LLM Mock): {user_response}")
            return user_response

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
                 return "stop" # Return "stop" to allow graceful termination
            except KeyboardInterrupt:
                 print("SPEECH: KeyboardInterrupt during manual input, returning 'stop'.")
                 return "stop" # Return "stop"


    def shutdown(self):
        print("SPEECH: SpeechInterface shutdown (Mock Mode).")

# (rest of the file if __name__ == '__main__' block)