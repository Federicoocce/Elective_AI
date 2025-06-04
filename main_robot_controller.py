#!/usr/bin/env python3
# main_robot_controller.py (NO ROS VERSION)

import os
import json
import math # For math.radians and yaw_to_quaternion
import time # For sleep
import random # For mock navigation time
import matplotlib.pyplot as plt
import numpy as np
# --- Mock ROS Imports/Classes ---
class MockLogger:
    def info(self, msg): print(f"INFO: {msg}")
    def warn(self, msg): print(f"WARN: {msg}")
    def error(self, msg, exc_info=False): print(f"ERROR: {msg}")
    def debug(self, msg): print(f"DEBUG: {msg}")

class MockGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.status = 4 # STATUS_SUCCEEDED
    
    async def get_result_async(self):
        await asyncio.sleep(random.uniform(0.5, 1.5)) 
        class MockResult:
            def __init__(self, status):
                self.status = status
        return MockResult(status=self.status) 

class MockActionClient:
    def __init__(self, node, action_type, action_name):
        self._node = node 
        self._action_name = action_name
        self._node.get_logger().info(f"MockActionClient for '{action_name}' initialized.")

    def server_is_ready(self): return True 
    def wait_for_server(self, timeout_sec): return True 

    async def send_goal_async(self, goal_msg):
        self._node.get_logger().info(f"MockNav: Goal received for {self._action_name}. Simulating acceptance.")
        await asyncio.sleep(0.1)
        return MockGoalHandle(accepted=True)

# --- Import Application Modules ---
from llm_groq_parser import GroqQueryParser
from speech_interface import SpeechInterface
from user_manager import UserProfileManager, VALID_USER_KEYS
from mall_query_engine import KnowledgeGraphService
from recommendation_engine import RecommenderEngine
from automated_benchmarking_llm import AutomatedBenchmarkingLLM 

# --- Global State ---
current_speaker_role = None 
speech_interface_global = SpeechInterface() # This is global, AutoLLM user is set on this
_running_main_loop = True 

# --- Global/Shared Instances (initialized once in __main__) ---
shared_kg_service_instance = None
shared_llm_manager_instance = None
shared_profile_manager_instance = None

import asyncio

def _yaw_to_quaternion(yaw_radians): 
    q = {} 
    q['x'] = 0.0; q['y'] = 0.0
    q['z'] = math.sin(yaw_radians / 2.0)
    q['w'] = math.cos(yaw_radians / 2.0)
    return q

class MainRobotController: 
    def __init__(self, kg_service_instance, llm_manager_instance, profile_manager_instance):
        self._logger = MockLogger() 
        self.get_logger().info("MainRobotController (NO ROS) initializing...")

        self.speech = speech_interface_global # Uses the global speech interface
        self.profile_manager = profile_manager_instance
        self.kg_service = kg_service_instance
        self.recommender = RecommenderEngine(knowledge_graph_service=self.kg_service, user_profile_manager=self.profile_manager)
        self.llm_manager = llm_manager_instance

        # LLM Manager's knowledge base lists are set once globally after KGS and LLM init.
        # No need to reset it here if KGS is static for the whole benchmark run.

        self._nav_action_client = MockActionClient(self, None, 'navigate_to_pose')
        self.get_logger().info("Mock NavigateToPose action client created.")
        
        self.tf_buffer = None 
        self.tf_listener = None
        self.get_logger().info("Mock TF Listener initialized (no actual TF).")
        
        self.get_logger().info("MainRobotController (NO ROS) initialized successfully.")

    def get_logger(self): return self._logger

    def _check_nav_server_ready(self, timeout_sec=5.0): 
        if not self._nav_action_client.server_is_ready():
            self.get_logger().warn("Mock Nav server not available. Waiting...") 
            if not self._nav_action_client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error("Mock Nav server not available after timeout.")
                return False
            self.get_logger().info("Mock Nav server is ready.")
        return True

    async def send_navigation_goal(self, x, y, theta_degrees): 
        self.get_logger().info(f"MockNav: Attempting to navigate to x={x:.2f}, y={y:.2f}, theta={theta_degrees:.2f} deg.")
        if not self._check_nav_server_ready(timeout_sec=1.0):
             self.get_logger().error("MockNav: Navigation server not ready, cannot send goal.")
             return False
        goal_msg_dummy = {"pose": {"x": x, "y": y, "theta_degrees": theta_degrees}}
        self.get_logger().info(f"MockNav: Sending goal: Pos({x:.2f}, {y:.2f}), Ori Degrees({theta_degrees:.2f})")
        try: goal_handle = await self._nav_action_client.send_goal_async(goal_msg_dummy) 
        except Exception as e: self.get_logger().error(f"MockNav: Error during send_goal_async: {e}"); return False
        if not goal_handle or not goal_handle.accepted: self.get_logger().error('MockNav: Goal was rejected by the server.'); return False
        self.get_logger().info('MockNav: Goal accepted. Simulating navigation...')
        try: result = await goal_handle.get_result_async() 
        except Exception as e: self.get_logger().error(f"MockNav: Error during get_result_async: {e}"); return False
        if result.status == 4: self.get_logger().info('MockNav: Navigation goal succeeded!'); return True
        else: self.get_logger().error(f'MockNav: Navigation goal failed with mock status: {result.status}'); return False

    def get_current_robot_pose(self): return (0.0, 0.0, 0.0) 

    def identify_role_from_text(self, text):
        if not text: return None
        text_lower = text.lower()
        # Prioritize more specific phrases if needed, but LLM should give direct role for "for me"
        # This function is more for general keyword spotting if LLM is not involved in role ID
        mapping = {
            "father": ["father", "dad", "papa", "husband", "man of the house"],
            "mother": ["mother", "mum", "mom", "mama", "wife", "lady of the house"],
            "child": ["child", "kid", "son", "daughter", "boy", "girl", "children", "young one", "little one"]}
        for role, keywords in mapping.items():
            if any(word in text_lower for word in keywords): return role
        return None

    def initialize_speaker_if_needed(self):
        global current_speaker_role
        if current_speaker_role: return
        self.speech.say("Before we begin, who will I be primarily talking with today? Please say Mother, Father, or Child.")
        for _ in range(3): 
            role_text = self.speech.listen_and_get_text(dialogue_state_hint="initial_speaker_id")
            if not role_text: role_text = "" 
            self.get_logger().info(f"[SPEAKER_ID] User said: {role_text}")
            
            # Check for direct persona match first for AutoLLMUser
            if self.speech.automated_llm_user and self.speech.automated_llm_user.persona in role_text.lower():
                role = self.speech.automated_llm_user.persona
            else: # Fallback to general identification
                role = self.identify_role_from_text(role_text)

            if role:
                current_speaker_role = role
                self.speech.say(f"Thank you, {current_speaker_role.capitalize()}."); return
            self.speech.say("I'm sorry, I didn't quite catch that. Could you please say if you are the Mother, Father, or Child?")
        
        # Fallback if AutoLLMUser is active and persona is known
        if self.speech.automated_llm_user and self.speech.automated_llm_user.persona in VALID_USER_KEYS:
             current_speaker_role = self.speech.automated_llm_user.persona
             self.speech.say(f"I'll assume I'm speaking with {current_speaker_role.capitalize()} for this session.")
        else: # Absolute fallback
            current_speaker_role = "mother" 
            self.speech.say(f"I'm having a little trouble understanding. I'll assume I'm speaking with Mother for now.")


    def get_confirmation_dialogue(self, prompt_text, max_retries=2, dialogue_state_hint_for_auto_llm=None, context_for_auto_llm=None):
        self.speech.say(prompt_text)
        for i in range(max_retries):
            response_text = self.speech.listen_and_get_text(
                dialogue_state_hint=dialogue_state_hint_for_auto_llm,
                context_for_auto_llm=context_for_auto_llm
            )
            if not response_text: response_text = "" 
            response = response_text.lower()
            self.get_logger().info(f"[CONFIRMATION] For '{prompt_text}', user said: '{response}'")
            if any(word in response for word in ["yes", "sure", "okay", "ok", "yep", "please", "do it", "let's go", "affirmative", "correct", "indeed", "let's check it out"]): return True # Added more affirmative
            if any(word in response for word in ["no", "nope", "don't", "skip", "cancel", "negative", "not now", "incorrect", "not that one"]): return False # Added more negative
            if i < max_retries - 1: self.speech.say("Sorry, I didn't understand. Please answer with a yes or no.")
        self.speech.say("Okay, I'll take that as a no for now."); return False

    def clarify_shopping_target(self):
        global current_speaker_role 
        self.speech.say("And who is this request for? (e.g., For me, For Mother, For Father, or For Child)")
        for _ in range(2): 
            response_text = self.speech.listen_and_get_text(
                 dialogue_state_hint="clarify_shopping_target",
                 context_for_auto_llm={"robot_query_summary": "this request"} # Context for AutoLLM if it's used
            )
            if not response_text: response_text = ""
            self.get_logger().info(f"[CLARIFY_TARGET] User said: '{response_text}'")
            
            # Check for "me" or "myself" type responses first
            # The AutoLLMUser is now prompted to give direct "For me" or "For [Persona]"
            if any(phrase in response_text.lower() for phrase in ["for me", "for myself", "this one's for me", f"for {current_speaker_role.lower() if current_speaker_role else ''}."]):
                if current_speaker_role:
                    self.get_logger().info(f"[CLARIFY_TARGET] Interpreted as for current speaker: '{current_speaker_role}'.")
                    return current_speaker_role
                else: # Should not happen if initialize_speaker_if_needed ran
                    self.speech.say("I'm not sure who 'me' is right now as the speaker isn't set. Assuming Mother for now.")
                    return "mother" 

            target_role = self.identify_role_from_text(response_text)
            if target_role: 
                self.get_logger().info(f"[CLARIFY_TARGET] Identified role: '{target_role}'.")
                return target_role
            
            self.speech.say("Sorry, I didn't catch that clearly. Is it for Mother, Father, or Child? Or you can say 'For me'.")
        
        default_target = current_speaker_role if current_speaker_role else "mother"
        self.speech.say(f"Okay, I'll assume this request is for {default_target.capitalize()}.")
        return default_target


    def _generate_request_summary_for_dialogue(self, parsed_llm_data):
        if not parsed_llm_data or not isinstance(parsed_llm_data, dict): return "your request"
        items = parsed_llm_data.get("item_types", [])
        attrs = parsed_llm_data.get("attributes", {})
        store = parsed_llm_data.get("store_name")
        desc_parts = []
        if isinstance(attrs, dict):
            if attrs.get("color"): desc_parts.append(attrs["color"])
            if attrs.get("brand"): desc_parts.append(attrs["brand"])
        item_str = ""
        if items and isinstance(items, list) and len(items) > 0:
            item_str = items[0] 
            if len(items) > 1: item_str += " or similar items"
        if item_str: desc_parts.append(item_str)
        if store: return f"finding {', '.join(desc_parts) if desc_parts else 'items'} at {store}" if desc_parts or not item_str else f"information about {store}"
        return f"finding {' '.join(desc_parts)}" if desc_parts else "your general request"

    async def _handle_iterative_store_visits_no_ros(self, original_request_parsed_data, ranked_store_options_with_details, shopping_for_user_role, active_profile_for_recs):
        request_summary = self._generate_request_summary_for_dialogue(original_request_parsed_data)
        self.get_logger().info(f"Iterative handler for: {request_summary} (for {shopping_for_user_role})")
        if not ranked_store_options_with_details:
            self.speech.say(f"It seems I don't have specific store suggestions for {request_summary} right now.")
            return {"status": "completed_recommendations_for_query"}

        any_request_fulfilled_overall = False # Tracks if the single item was found in *any* store visit attempt
        for i, store_option_details in enumerate(ranked_store_options_with_details):
            store_info = store_option_details["store_info"]
            product_count_if_any = store_option_details["product_count_if_any"]
            store_name = store_info.get('name', 'an unnamed store')
            dist_val = store_info.get('distance', float('inf'))
            dist_str = f" (Est. Distance: {dist_val:.1f}m)" if dist_val < float('inf') else "" 
            proposal_intro = f"For {request_summary},"
            proposal = f"{proposal_intro} {'I found ' + str(product_count_if_any) + ' matching item(s) at ' if product_count_if_any is not None else 'one potential place is '}{store_name}{dist_str}."
            is_last_option_in_list = (i == len(ranked_store_options_with_details) - 1)
            confirm_context = {"store_name": store_name, "item_summary": request_summary} # For AutoLLM
            
            # For single item task, if already fulfilled, no need to visit more stores for THIS item.
            if any_request_fulfilled_overall:
                self.get_logger().info(f"Item '{request_summary}' already fulfilled. Skipping further store visits for it.")
                break 

            if not self.get_confirmation_dialogue(f"{proposal} Shall we 'go' there (simulation)?",
                                                  dialogue_state_hint_for_auto_llm="confirm_visit_store",
                                                  context_for_auto_llm=confirm_context):
                self.speech.say(f"Okay, we'll skip {store_name} for now.")
                if is_last_option_in_list: self.speech.say(f"That was the last option I had for {request_summary}.")
                continue
            map_x, map_y, map_theta = store_info.get('map_x'), store_info.get('map_y'), store_info.get('map_theta', 0.0)
            if map_x is None or map_y is None:
                self.speech.say(f"Sorry, I don't have precise location data to simulate navigation to {store_name}."); nav_success = False
            else: nav_success = await self.send_navigation_goal(map_x, map_y, map_theta) 
            if nav_success:
                self.speech.say(f"We've 'arrived' at {store_name}. Please 'look around'. Let me know when you're ready.")
                self.speech.listen_and_get_text(dialogue_state_hint="acknowledge_arrival_readiness")
            else:
                self.speech.say(f"Sorry, I encountered an issue 'navigating' to {store_name}.")
                confirm_context_next_store = {"item_summary": request_summary} 
                if not is_last_option_in_list and self.get_confirmation_dialogue("Try the next store suggestion?", 
                                                                                dialogue_state_hint_for_auto_llm="confirm_visit_store", 
                                                                                context_for_auto_llm=confirm_context_next_store): continue
                else: self.speech.say(f"Okay, stopping search for {request_summary}."); return {"status": "completed_recommendations_for_query"}

            self.speech.say(f"Regarding {request_summary}, did you find what you were looking for at {store_name}?")
            fulfillment_response_text = self.speech.listen_and_get_text(
                dialogue_state_hint="provide_fulfillment_feedback",
                context_for_auto_llm={"item_summary": request_summary, "store_name": store_name}
            )
            if not fulfillment_response_text: fulfillment_response_text = ""
            fulfillment_status_llm = self.llm_manager.parse_fulfillment_status(fulfillment_response_text, request_summary)
            current_item_fulfilled_at_this_store = fulfillment_status_llm.get("fulfilled", False) if not fulfillment_status_llm.get("error") else False
            
            if current_item_fulfilled_at_this_store: 
                self.speech.say("That's great!"); 
                any_request_fulfilled_overall = True # Mark the single item as found
            else: self.speech.say(f"Oh, sorry you couldn't find {request_summary} there.")

            self.speech.say(f"Any other feedback about {store_name}? (e.g., selection, service) Or say 'no feedback'.")
            feedback_text_raw = self.speech.listen_and_get_text(
                 dialogue_state_hint="provide_general_store_feedback",
                 context_for_auto_llm={"store_name": store_name}
            )
            if not feedback_text_raw: feedback_text_raw = ""
            feedback_text = feedback_text_raw.strip().lower()
            if feedback_text not in ["no feedback", "nothing", "skip", "no", ""]:
                profile_updates_llm = self.llm_manager.parse_feedback_to_profile_update(
                    feedback_text_raw, store_name, original_request_parsed_data.get("item_types", [])
                )
                if profile_updates_llm and not profile_updates_llm.get("error"):
                    if "shop_review_update" not in profile_updates_llm or profile_updates_llm["shop_review_update"] is None: profile_updates_llm["shop_review_update"] = {} 
                    profile_updates_llm["shop_review_update"]["last_visit_fulfilled"] = current_item_fulfilled_at_this_store
                    self.profile_manager.update_profile_with_feedback(shopping_for_user_role, store_name, profile_updates_llm)
                    self.speech.say(f"Thanks! Updated notes for {shopping_for_user_role.capitalize()}.")
                else: self.speech.say("Thanks for the feedback. (Could not parse for profile)")
            else: self.speech.say("Okay.")

            # For single item task, after one store visit and feedback, the AutoLLM user will signal to stop.
            # So, the "what next" from robot should directly lead to the AutoLLM user deciding to stop the session.
            has_more_options_for_this_item = not is_last_option_in_list and not any_request_fulfilled_overall

            # This "what next" dialogue is now simplified because AutoLLM will choose to stop after one item.
            # However, for a human user, or a multi-item AutoLLM list, this logic would be more complex.
            # Given AutoLLM now does 1 item and then signals stop, this robot prompt is mostly for completeness
            # before AutoLLM gives its "That's all" response.
            if any_request_fulfilled_overall:
                next_action_prompt = "Since we found that, is there anything else for today, or shall we stop?"
            elif has_more_options_for_this_item:
                next_action_prompt = f"We didn't find {request_summary} there. Would you like to try another store for it, or stop for now?"
            else: # Not found, and no more options for this item
                next_action_prompt = f"It seems that was the last option for {request_summary}. Shall we stop, or look for something completely different?"
                
            self.speech.say(next_action_prompt)
            
            # AutoLLMUser's decide_next_action will be called here.
            # For a single item, if found, it will say "stop". If not found and no more options, it will say "stop".
            # If not found but more options, it might say "continue" or "stop".
            next_action_context = {
                "original_request_summary": request_summary,
                "was_last_item_fulfilled": any_request_fulfilled_overall, # Robot's understanding based on parser
                "has_more_options": has_more_options_for_this_item
            }
            next_action_response_text = self.speech.listen_and_get_text(dialogue_state_hint="decide_next_action", context_for_auto_llm=next_action_context)
            if not next_action_response_text: next_action_response_text = ""
            next_action_llm = self.llm_manager.parse_next_action_decision(next_action_response_text, request_summary, any_request_fulfilled_overall, has_more_options_for_this_item)

            if next_action_llm.get("error"): self.speech.say("Unsure what to do next. Ending this search."); return {"status": "completed_recommendations_for_query"}
            intent_next = next_action_llm.get("intent")
            new_query = next_action_llm.get("new_query_text") # This should be null if AutoLLM stops after 1 item

            if intent_next == "new_request" and new_query: # This path should not be taken by AutoLLM user with 1 item
                 self.speech.say(f"Okay, looking into: {new_query}"); return {"status": "new_request", "detail": new_query}
            elif intent_next == "stop_interaction": 
                return {"status": "stop_session"} 
            elif intent_next == "continue_current_request":
                if has_more_options_for_this_item: self.speech.say(f"Okay, next option for {request_summary}.")
                else: self.speech.say(f"Actually, that was the last option for {request_summary}."); return {"status": "completed_recommendations_for_query"}
            else: # Default to completing this query if intent is unclear
                 self.speech.say("Assuming we're done with that request."); return {"status": "completed_recommendations_for_query"}
            time.sleep(0.05) 

        if not any_request_fulfilled_overall: self.speech.say(f"We've gone through all suggestions for {request_summary}, but it seems we couldn't find it this time.")
        # If fulfilled, the "That's great!" from above suffices.
        return {"status": "completed_recommendations_for_query"}

    async def process_intent_no_ros(self, intent_from_llm, parsed_llm_data, shopping_for_user_role, active_profile_for_recs): 
        if intent_from_llm == "show_profile":
            self.speech.say(f"Preferences for {shopping_for_user_role.capitalize()}:")
            self.profile_manager.display_profile(shopping_for_user_role)
            return {"status": "interaction_completed_for_current_query"}
        if intent_from_llm == "update_profile_only":
            self.speech.say(f"Preferences updated for {shopping_for_user_role.capitalize()}.")
            # self.profile_manager.display_profile(shopping_for_user_role) # Optional: display after update
            return {"status": "interaction_completed_for_current_query"}
        if intent_from_llm in {"find_product", "find_store"}:
            current_pose_tuple = self.get_current_robot_pose() 
            if current_pose_tuple == (None, None, None): self.speech.say("Warning: My current location is unclear (mock), so recommendations might not be distance-optimized.")
            kg_query = {"intent": intent_from_llm, "item_types": parsed_llm_data.get("item_types", []), "attributes": parsed_llm_data.get("attributes", {}), "store_name": parsed_llm_data.get("store_name")}
            query_results_kg = self.kg_service.execute_structured_query(kg_query)
            recs_data = self.recommender.generate_recommendations(parsed_llm_data, query_results_kg, active_profile_for_recs, current_pose_tuple)
            ranked_options = []
            if intent_from_llm == "find_store" and recs_data.get("stores"):
                for store_rec in recs_data["stores"]: ranked_options.append({"store_info": store_rec, "product_count_if_any": None})
            elif intent_from_llm == "find_product" and recs_data.get("products_in_stores_ranked"):
                for prod_loc_rec in recs_data["products_in_stores_ranked"]: ranked_options.append({"store_info": prod_loc_rec["store_details"], "product_count_if_any": len(prod_loc_rec.get("products_found", []))})
            if not ranked_options: self.speech.say(f"Sorry, no suitable stores/products found for {self._generate_request_summary_for_dialogue(parsed_llm_data)}."); return {"status": "interaction_completed_for_current_query"}
            return await self._handle_iterative_store_visits_no_ros(parsed_llm_data, ranked_options, shopping_for_user_role, active_profile_for_recs)
        self.speech.say(f"Unsure how to handle intent: '{intent_from_llm}'. Try rephrasing?"); return {"status": "interaction_completed_for_current_query"}

    async def run_shopping_session_loop(self): 
        global current_speaker_role, _running_main_loop 
        _running_main_loop = True 

        self.speech.wait_for_wake_word() 
        self.initialize_speaker_if_needed()

        if not current_speaker_role:
            self.get_logger().error("Speaker role not initialized. Cannot start session.")
            self.speech.say("Startup error: Speaker not identified.")
            return 

        self.speech.say(f"Hello {current_speaker_role.capitalize()}! How can I help you or the family today with your one main shopping goal? (Or say 'stop' to end.)")
        current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="initial_query")

        while _running_main_loop: 
            if not current_user_input: 
                self.get_logger().warn("Received no input (likely EOF or Ctrl+C in manual), treating as stop.")
                parsed_llm_output = {"intent": "stop_interaction"}
            elif current_user_input.strip().lower() == "stop": 
                parsed_llm_output = {"intent": "stop_interaction"}
            else:
                # For AutoLLMUser, profile context is less critical for initial query parsing if it always refers to self.
                # For human users, it's more useful.
                profile_for_llm_context = self.profile_manager.get_profile(current_speaker_role)
                parsed_llm_output = self.llm_manager.generate_structured_query(
                    current_user_input, current_speaker_role_if_known=current_speaker_role,
                    shopping_for_user_profile_context=profile_for_llm_context
                )
            
            self.get_logger().info(f"LLM Parsed Output: {json.dumps(parsed_llm_output, indent=2)}")

            if not parsed_llm_output or parsed_llm_output.get("error"):
                self.speech.say("Sorry, I had trouble understanding that. Please try rephrasing.")
                current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="general_clarification") 
                time.sleep(0.05); continue 
            
            intent = parsed_llm_output.get("intent")
            if intent == "stop_interaction":
                _running_main_loop = False 
                time.sleep(0.05); continue # Go to top of while, which will now exit

            # --- Process non-stop intents ---
            shopping_for_user_this_query = None 
            llm_target = parsed_llm_output.get("shopping_for_user")

            if llm_target == "self": shopping_for_user_this_query = current_speaker_role
            elif llm_target in VALID_USER_KEYS: shopping_for_user_this_query = llm_target
            else: # shopping_for_user is null or invalid
                relevant_intents = {"find_product", "find_store", "show_profile", "update_profile_only"}
                if intent in relevant_intents: 
                    self.speech.say(f"I understood: \"{current_user_input}\".")
                    clarified = self.clarify_shopping_target() 
                    if clarified: shopping_for_user_this_query = clarified
                    else: # Clarification failed or user didn't specify
                        self.speech.say("Okay, what would you like to do then?")
                        current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="general_clarification")
                        time.sleep(0.05); continue 
                else: # For intents like "greeting" or "general_command" not needing a target user
                    shopping_for_user_this_query = current_speaker_role 

            self.speech.say(f"Working on this for {shopping_for_user_this_query.capitalize()}.")
            if parsed_llm_output.get("updated_profile_for_shopping_user"):
                self.profile_manager.update_basic_preferences(
                    shopping_for_user_this_query, parsed_llm_output["updated_profile_for_shopping_user"]
                ) 
            active_profile = self.profile_manager.get_profile(shopping_for_user_this_query)
            
            session_status = await self.process_intent_no_ros(intent, parsed_llm_output, shopping_for_user_this_query, active_profile) 
            
            self.get_logger().info(f"Main loop received session status: {session_status}")
            self.get_logger().info("-" * 60) 

            if session_status.get("status") == "new_request": # Should not happen with 1-item AutoLLM user
                current_user_input = session_status.get("detail", "")
            elif session_status.get("status") == "stop_session":
                _running_main_loop = False 
            else: # "interaction_completed_for_current_query" or "completed_recommendations_for_query"
                # For AutoLLMUser doing one task, it should have signalled stop_session by now.
                # This path is more for human users or multi-task AutoLLM.
                if _running_main_loop : 
                    self.speech.say(f"Is there anything else I can help with for your main shopping goal, {current_speaker_role.capitalize()}? Or say 'stop'.")
                    current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="respond_to_anything_else") # AutoLLM will say "No, that's all"
            
            time.sleep(0.05) 
            
        self.speech.say("Alright, ending our shopping trip. Have a great day!") 
        self.get_logger().info("Shopping session loop ended.")
        self.profile_manager._save_profiles() # Save at the very end of a session

async def main_async_runner(kg_service, llm_parser, user_profiles, args=None, iteration_num=0): 
    global _running_main_loop, current_speaker_role
    _running_main_loop = True 
    current_speaker_role = None # Reset for each iteration

    print(f"\n\n--- STARTING BENCHMARKING ITERATION {iteration_num + 1} ---\n")
    # User profiles are NOT deleted, they are passed in and persist.

    main_controller = MainRobotController(kg_service_instance=kg_service,
                                          llm_manager_instance=llm_parser,
                                          profile_manager_instance=user_profiles)
    USE_AUTOMATED_LLM_USER = True 
    # Persona selection is random for each iteration, simulating different users interacting over time
    AUTOMATED_USER_PERSONA = random.choice(["mother", "father", "child"]) 
    print(f"INFO: Automated user persona for iteration {iteration_num + 1}: {AUTOMATED_USER_PERSONA}")

    if USE_AUTOMATED_LLM_USER:
        auto_llm_user = AutomatedBenchmarkingLLM(persona=AUTOMATED_USER_PERSONA)
        sample_products_for_llm = []
        if main_controller.kg_service and main_controller.kg_service.graph:
            all_products = [data for _, data in main_controller.kg_service.graph.nodes(data=True) if data.get('label_node') == 'Product']
            if all_products:
                sample_size = min(len(all_products), 20) # Increased sample size for more variety in goals
                sample_products_for_llm = random.sample(all_products, sample_size)
        
        # Knowledge for AutoLLMUser is derived from the shared KGS instance
        knowledge_for_auto_user = {
            "item_categories": kg_service.get_all_item_categories(),
            "brands": kg_service.get_all_brands(),
            "colors": kg_service.get_all_colors(),
            "store_names": kg_service.get_all_store_names(),
            "sample_products": sample_products_for_llm 
        }
        speech_interface_global.set_automated_llm_user(auto_llm_user, knowledge_for_auto_user)
        # main_controller.speech is already speech_interface_global
        print(f"\n<<<<< RUNNING IN AUTOMATED BENCHMARKING MODE (Persona: {AUTOMATED_USER_PERSONA}) - Iteration {iteration_num + 1} >>>>>\n")
    else:
        speech_interface_global.set_automated_llm_user(None, {}) 
    
    try: await main_controller.run_shopping_session_loop() 
    except KeyboardInterrupt: main_controller.get_logger().info(f"KeyboardInterrupt received during iteration {iteration_num + 1}, shutting down this iteration.")
    except Exception as e: main_controller.get_logger().error(f"Unhandled exception during iteration {iteration_num + 1}: {e}", exc_info=True)
    finally:
        _running_main_loop = False 
        main_controller.get_logger().info(f"Shutting down MainRobotController (NO ROS) for iteration {iteration_num + 1}...")

    print("\n" + "="*20 + f" Final User Profiles After Iteration {iteration_num + 1} " + "="*20)
    # Display profiles from the shared profile manager
    for role_key_check in VALID_USER_KEYS: user_profiles.display_profile(role_key_check)
    print("="*60 + f"\nSession for iteration {iteration_num + 1} fully complete.\n" + f"--- COMPLETED BENCHMARKING ITERATION {iteration_num + 1} ---\n")

if __name__ == "__main__":
    num_iterations = 10 # Set number of iterations for benchmarking
    results_collector = []
    
    print("--- Initializing Shared Services for Benchmarking ---")
    shared_kg_service_instance = KnowledgeGraphService()
    shared_llm_manager_instance = GroqQueryParser()
    # Profile manager loads from file if it exists, allowing learning across benchmark runs if not reset externally.
    # For a truly clean N-iteration benchmark run where learning starts from scratch *for that run*,
    # you might delete the JSON file *before* the entire N-iteration loop starts.
    # The current change is to NOT delete it *between* iterations.
    prefs_file = "family_preferences_main.json"
    # Optional: Delete prefs file once before ALL iterations if you want each N-iteration run to start fresh
    # if os.path.exists(prefs_file) and num_iterations > 1: # Example condition
    #     print(f"INFO: Deleting existing '{prefs_file}' for a fresh multi-iteration benchmark run.")
    #     os.remove(prefs_file)

    shared_profile_manager_instance = UserProfileManager(users_file=prefs_file)

    # Set knowledge base for LLM Parser (GroqQueryParser) once, as KGS is static
    shared_llm_manager_instance.set_knowledge_base_lists(
        store_names=shared_kg_service_instance.get_all_store_names(),
        item_categories=shared_kg_service_instance.get_all_item_categories(),
        brands=shared_kg_service_instance.get_all_brands(),
        colors=shared_kg_service_instance.get_all_colors()
    )
    print("--- Shared Services Initialized ---")

    for i in range(num_iterations):
        print(f"==================== ITERATION {i + 1} OF {num_iterations} ====================")
        asyncio.run(main_async_runner(kg_service=shared_kg_service_instance,
                                      llm_parser=shared_llm_manager_instance,
                                      user_profiles=shared_profile_manager_instance,
                                      iteration_num=i)) # args=None is default
        
        if speech_interface_global.automated_llm_user:
            metrics = speech_interface_global.automated_llm_user.get_session_metrics()
            results_collector.append(metrics)
            print(f"ITERATION {i+1} METRICS: {metrics}")
        
        time.sleep(0.5) # Small delay between iterations
    
    # --- Plotting Results ---
    if results_collector:
        # Plot 1: Success rate (e.g., per block of 5 iterations or rolling average)
        block_size = 5 if num_iterations >= 5 else 1
        success_rates_blocks = []
        iteration_blocks_labels = []
        for i in range(0, num_iterations, block_size):
            chunk = results_collector[i:i+block_size]
            if not chunk: continue
            success_rate = sum(1 for r in chunk if r['success']) / len(chunk)
            success_rates_blocks.append(success_rate)
            iteration_blocks_labels.append(f"Iter {i+1}-{min(i+block_size, num_iterations)}")
        
        if success_rates_blocks:
            plt.figure(figsize=(10, 5))
            plt.bar(iteration_blocks_labels, success_rates_blocks, color='skyblue')
            plt.xlabel("Iteration Blocks")
            plt.ylabel("Success Rate (Found Single Target Item)")
            plt.title(f"Success Rate per {block_size} Iterations")
            plt.ylim(0, 1.05)
            plt.xticks(rotation=45, ha="right")
            plt.tight_layout()
            plt.savefig("success_rate_benchmark.png")
            print("Success rate plot saved to success_rate_benchmark.png")
        
        # Plot 2: Interaction count per iteration
        interaction_counts = [r['interaction_count'] for r in results_collector]
        iterations_x_axis = range(1, len(interaction_counts) + 1)
        plt.figure(figsize=(10, 5))
        plt.plot(iterations_x_axis, interaction_counts, marker='o', linestyle='-', color='coral')
        plt.xlabel("Iteration Number")
        plt.ylabel("Number of Interactions (LLM User)")
        plt.title("Interaction Count per Iteration")
        if num_iterations <= 20 : plt.xticks(iterations_x_axis) # Show all ticks if not too many
        else: plt.locator_params(axis='x', nbins=10) # Otherwise, limit number of x-ticks
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        plt.savefig("interaction_count_benchmark.png")
        print("Interaction count plot saved to interaction_count_benchmark.png")
    else:
        print("No results collected for plotting.")
    
    print("\nBenchmarking complete. User profiles have been updated in family_preferences_main.json (if changes occurred).")