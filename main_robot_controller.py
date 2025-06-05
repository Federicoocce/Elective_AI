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
        # Simulate some async work
        await asyncio.sleep(random.uniform(0.5, 1.5)) # Simulate navigation time
        class MockResult:
            def __init__(self, status):
                self.status = status
        return MockResult(status=self.status) 

class MockActionClient:
    def __init__(self, node, action_type, action_name):
        self._node = node # Mock node-like object with get_logger()
        self._action_name = action_name
        self._node.get_logger().info(f"MockActionClient for '{action_name}' initialized.")

    def server_is_ready(self): return True # Always ready in mock
    def wait_for_server(self, timeout_sec): return True # Always available in mock

    async def send_goal_async(self, goal_msg):
        self._node.get_logger().info(f"MockNav: Goal received for {self._action_name}. Simulating acceptance.")
        await asyncio.sleep(0.1) # Simulate server response time
        return MockGoalHandle(accepted=True)

# --- Import Application Modules ---
from llm_groq_parser import GroqQueryParser
from speech_interface import SpeechInterface
from user_manager import UserProfileManager, VALID_USER_KEYS # UserProfileManager is key
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
shared_profile_manager_instance = None # This will be reloaded per iteration now

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
        self.profile_manager = profile_manager_instance # This instance is now reloaded per iteration
        self.kg_service = kg_service_instance
        self.recommender = RecommenderEngine(knowledge_graph_service=self.kg_service, user_profile_manager=self.profile_manager)
        self.llm_manager = llm_manager_instance

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
            
            identified_role_in_text = self.identify_role_from_text(role_text)

            if self.speech.automated_llm_user and self.speech.automated_llm_user.persona == identified_role_in_text:
                role = identified_role_in_text 
            elif self.speech.automated_llm_user and self.speech.automated_llm_user.persona in role_text.lower(): 
                role = self.speech.automated_llm_user.persona
            else: 
                role = identified_role_in_text


            if role:
                current_speaker_role = role
                self.speech.say(f"Thank you, {current_speaker_role.capitalize()}."); return
            self.speech.say("I'm sorry, I didn't quite catch that. Could you please say if you are the Mother, Father, or Child?")
        
        if self.speech.automated_llm_user and self.speech.automated_llm_user.persona in VALID_USER_KEYS:
             current_speaker_role = self.speech.automated_llm_user.persona
             self.speech.say(f"I'll assume I'm speaking with {current_speaker_role.capitalize()} for this session.")
        else: 
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
            if any(word in response for word in ["yes", "sure", "okay", "ok", "yep", "please", "do it", "let's go", "affirmative", "correct", "indeed", "let's check it out"]): return True 
            if any(word in response for word in ["no", "nope", "don't", "skip", "cancel", "negative", "not now", "incorrect", "not that one"]): return False 
            if i < max_retries - 1: self.speech.say("Sorry, I didn't understand. Please answer with a yes or no.")
        self.speech.say("Okay, I'll take that as a no for now."); return False

    def clarify_shopping_target(self):
        global current_speaker_role 
        self.speech.say("And who is this request for? (e.g., For me, For Mother, For Father, or For Child)")
        for _ in range(2): 
            response_text = self.speech.listen_and_get_text(
                 dialogue_state_hint="clarify_shopping_target",
                 context_for_auto_llm={"robot_query_summary": "this request"} 
            )
            if not response_text: response_text = ""
            self.get_logger().info(f"[CLARIFY_TARGET] User said: '{response_text}'")
            
            is_for_self = False
            if current_speaker_role:
                if any(phrase in response_text.lower() for phrase in ["for me", "for myself", "this one's for me", f"for {current_speaker_role.lower()}."]):
                    is_for_self = True
            else: 
                 if any(phrase in response_text.lower() for phrase in ["for me", "for myself", "this one's for me"]):
                    is_for_self = True
            
            if is_for_self:
                if current_speaker_role:
                    self.get_logger().info(f"[CLARIFY_TARGET] Interpreted as for current speaker: '{current_speaker_role}'.")
                    return current_speaker_role
                else: 
                    self.speech.say("I'm not sure who 'me' is right now as the speaker isn't set. Let's assume Mother for this item for now.")
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
        if item_str: desc_parts.append(item_str)
        if store: return f"finding {', '.join(desc_parts) if desc_parts else 'items'} at {store}" if desc_parts or not item_str else f"information about {store}"
        return f"finding {' '.join(desc_parts)}" if desc_parts else "your general request"

    async def _handle_iterative_store_visits_no_ros(self, original_request_parsed_data, ranked_store_options_with_details, shopping_for_user_role, active_profile_for_recs):
        request_summary = self._generate_request_summary_for_dialogue(original_request_parsed_data)
        self.get_logger().info(f"Iterative handler for item: {request_summary} (for {shopping_for_user_role})")
        
        if not ranked_store_options_with_details:
            self.speech.say(f"It seems I don't have specific store suggestions for {request_summary} right now.")
            return {"status": "completed_recommendations_for_this_item", "item_fulfilled": False}


        item_fulfilled_in_this_handler = False 
        for i, store_option_details in enumerate(ranked_store_options_with_details):
            store_info = store_option_details["store_info"]
            product_count_if_any = store_option_details["product_count_if_any"]
            store_name = store_info.get('name', 'an unnamed store')
            dist_val = store_info.get('distance', float('inf'))
            dist_str = f" (Est. Distance: {dist_val:.1f}m)" if dist_val < float('inf') else "" 
            proposal_intro = f"For {request_summary},"
            proposal = f"{proposal_intro} {'I found ' + str(product_count_if_any) + ' matching item(s) at ' if product_count_if_any is not None else 'one potential place is '}{store_name}{dist_str}."
            is_last_option_in_list_for_this_item = (i == len(ranked_store_options_with_details) - 1)
            confirm_context = {"store_name": store_name, "item_summary": request_summary} 
            
            if item_fulfilled_in_this_handler: 
                self.get_logger().info(f"Item '{request_summary}' already fulfilled in a previous store visit. Breaking from store options for this item.")
                break 

            if not self.get_confirmation_dialogue(f"{proposal} Shall we 'go' there (simulation)?",
                                                  dialogue_state_hint_for_auto_llm="confirm_visit_store",
                                                  context_for_auto_llm=confirm_context):
                self.speech.say(f"Okay, we'll skip {store_name} for now.")
                if is_last_option_in_list_for_this_item: self.speech.say(f"That was the last store option I had for {request_summary}.")
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
                if not is_last_option_in_list_for_this_item and self.get_confirmation_dialogue("Try the next store suggestion for this item?", 
                                                                                dialogue_state_hint_for_auto_llm="confirm_visit_store", 
                                                                                context_for_auto_llm={"item_summary": request_summary, "failed_store_name": store_name}): continue 
                else: self.speech.say(f"Okay, stopping search attempts for {request_summary}."); return {"status": "completed_recommendations_for_this_item", "item_fulfilled": False}

            self.speech.say(f"Regarding {request_summary}, did you find what you were looking for at {store_name}?")
            fulfillment_response_text = self.speech.listen_and_get_text(
                dialogue_state_hint="provide_fulfillment_feedback",
                context_for_auto_llm={"item_summary": request_summary, "store_name": store_name}
            )
            if not fulfillment_response_text: fulfillment_response_text = ""
            
            current_item_fulfilled_at_this_store = self.speech.automated_llm_user.last_item_found_at_store if self.speech.automated_llm_user else False
            
            if current_item_fulfilled_at_this_store: 
                self.speech.say("That's great!"); 
                item_fulfilled_in_this_handler = True 
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
                    if "shop_review_update" not in profile_updates_llm or not isinstance(profile_updates_llm["shop_review_update"], dict): 
                        profile_updates_llm["shop_review_update"] = {}
                    
                    # The UserProfileManager expects "request_fulfilled" from the LLM output
                    # Set it based on current_item_fulfilled_at_this_store for accuracy.
                    profile_updates_llm["shop_review_update"]["request_fulfilled"] = current_item_fulfilled_at_this_store
                    
                    self.profile_manager.update_profile_with_feedback(shopping_for_user_role, store_name, profile_updates_llm)
                    self.speech.say(f"Thanks! Updated notes for {shopping_for_user_role.capitalize()}.")
                else: self.speech.say("Thanks for the feedback. (Could not parse for profile)")
            else: self.speech.say("Okay.")
            
            has_more_store_options_for_this_item = not is_last_option_in_list_for_this_item and not item_fulfilled_in_this_handler

            if item_fulfilled_in_this_handler:
                next_action_prompt = f"We found the {request_summary}! What would you like to do now?"
            elif has_more_store_options_for_this_item:
                next_action_prompt = f"We didn't find {request_summary} at {store_name}. Would you like to try another store for it, or something else?"
            else: 
                next_action_prompt = f"It seems that was the last option for {request_summary} (or we couldn't find it). What's next?"
                
            self.speech.say(next_action_prompt)
            
            next_action_context = {
                "original_request_summary": request_summary, 
                "was_last_item_fulfilled": item_fulfilled_in_this_handler, 
                "has_more_options": has_more_store_options_for_this_item 
            }
            next_action_response_text = self.speech.listen_and_get_text(dialogue_state_hint="decide_next_action", context_for_auto_llm=next_action_context)
            if not next_action_response_text: next_action_response_text = ""
            
            next_action_llm_parsed = self.llm_manager.parse_next_action_decision(next_action_response_text, request_summary, item_fulfilled_in_this_handler, has_more_store_options_for_this_item)

            if next_action_llm_parsed.get("error"): 
                self.speech.say("I'm a bit confused. Let's assume we're done with this item for now."); 
                return {"status": "completed_recommendations_for_this_item", "item_fulfilled": item_fulfilled_in_this_handler}

            intent_next = next_action_llm_parsed.get("intent")
            new_query_text_from_user = next_action_llm_parsed.get("new_query_text")

            if intent_next == "new_request" and new_query_text_from_user: 
                 self.speech.say(f"Okay, processing: {new_query_text_from_user}"); 
                 return {"status": "new_request", "detail": new_query_text_from_user, "item_fulfilled": item_fulfilled_in_this_handler}
            elif intent_next == "stop_interaction": 
                return {"status": "stop_session", "item_fulfilled": item_fulfilled_in_this_handler} 
            elif intent_next == "continue_current_request":
                if has_more_store_options_for_this_item: 
                    self.speech.say(f"Okay, let's try another option for {request_summary}.")
                else: 
                    self.speech.say(f"Actually, that was the last store option for {request_summary}."); 
                    return {"status": "completed_recommendations_for_this_item", "item_fulfilled": item_fulfilled_in_this_handler}
            else: 
                 self.speech.say("Okay, let's consider this item search concluded for now."); 
                 return {"status": "completed_recommendations_for_this_item", "item_fulfilled": item_fulfilled_in_this_handler}
            time.sleep(0.05) 

        if not item_fulfilled_in_this_handler: 
            self.speech.say(f"We've gone through all store suggestions for {request_summary}, but it seems we couldn't find it this time.")
        
        return {"status": "completed_recommendations_for_this_item", "item_fulfilled": item_fulfilled_in_this_handler}


    async def process_intent_no_ros(self, intent_from_llm, parsed_llm_data, shopping_for_user_role, active_profile_for_recs): 
        if intent_from_llm == "show_profile":
            self.speech.say(f"Preferences for {shopping_for_user_role.capitalize()}:")
            self.profile_manager.display_profile(shopping_for_user_role)
            return {"status": "interaction_completed_for_current_query"} 
        if intent_from_llm == "update_profile_only":
            self.speech.say(f"Preferences updated for {shopping_for_user_role.capitalize()}.")
            # Profile update was handled during LLM parsing if "updated_profile_for_shopping_user" was present
            # Or if this intent itself came with profile data (though current LLM prompt doesn't focus on that for this intent)
            return {"status": "interaction_completed_for_current_query"}
        
        if intent_from_llm in {"find_product", "find_store"}:
            if self.speech.automated_llm_user and self.speech.automated_llm_user.persona == "child" and not self.speech.automated_llm_user.shopping_list:
                self.speech.say("It seems you don't have anything specific on your list right now. Just looking around is fine!")
                return {"status": "interaction_completed_for_current_query"} 

            current_pose_tuple = self.get_current_robot_pose() 
            if current_pose_tuple == (None, None, None): self.speech.say("Warning: My current location is unclear (mock), so recommendations might not be distance-optimized.")
            
            kg_query = {"intent": intent_from_llm, 
                        "item_types": parsed_llm_data.get("item_types", []), 
                        "attributes": parsed_llm_data.get("attributes", {}), 
                        "store_name": parsed_llm_data.get("store_name")}
            query_results_kg = self.kg_service.execute_structured_query(kg_query)
            
            recs_data = self.recommender.generate_recommendations(parsed_llm_data, query_results_kg, active_profile_for_recs, current_pose_tuple)
            
            ranked_options = []
            if intent_from_llm == "find_store" and recs_data.get("stores"):
                for store_rec in recs_data["stores"]: ranked_options.append({"store_info": store_rec, "product_count_if_any": None})
            elif intent_from_llm == "find_product" and recs_data.get("products_in_stores_ranked"):
                for prod_loc_rec in recs_data["products_in_stores_ranked"]: ranked_options.append({"store_info": prod_loc_rec["store_details"], "product_count_if_any": len(prod_loc_rec.get("products_found", []))})
            
            if not ranked_options: 
                self.speech.say(f"Sorry, no suitable stores/products found for {self._generate_request_summary_for_dialogue(parsed_llm_data)}."); 
                return {"status": "completed_recommendations_for_this_item", "item_fulfilled": False} 
            
            return await self._handle_iterative_store_visits_no_ros(parsed_llm_data, ranked_options, shopping_for_user_role, active_profile_for_recs)
        
        self.speech.say(f"Unsure how to handle intent: '{intent_from_llm}'. Try rephrasing?"); 
        return {"status": "interaction_completed_for_current_query"} 

    async def run_shopping_session_loop(self): 
        global current_speaker_role, _running_main_loop 
        _running_main_loop = True 

        self.speech.wait_for_wake_word() 
        self.initialize_speaker_if_needed()

        if not current_speaker_role:
            self.get_logger().error("Speaker role not initialized. Cannot start session.")
            self.speech.say("Startup error: Speaker not identified.")
            return 

        if self.speech.automated_llm_user and self.speech.automated_llm_user.persona == "child" and not self.speech.automated_llm_user.shopping_list:
             self.speech.say(f"Hello {current_speaker_role.capitalize()}! What would you like to do today? (Or say 'stop' to end.)")
        else:
             self.speech.say(f"Hello {current_speaker_role.capitalize()}! How can I help you with your shopping list today? You can tell me about the first item. (Or say 'stop' to end.)")
        
        current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="initial_query")

        while _running_main_loop: 
            if not current_user_input: 
                self.get_logger().warn("Received no input (likely EOF or Ctrl+C in manual), treating as stop.")
                parsed_llm_output = {"intent": "stop_interaction"}
            elif current_user_input.strip().lower() == "stop": 
                parsed_llm_output = {"intent": "stop_interaction"}
            else:
                profile_context_user = current_speaker_role
                if self.speech.automated_llm_user:
                    auto_llm_target_item = self.speech.automated_llm_user._get_current_target_product()
                    if auto_llm_target_item and auto_llm_target_item.get('target_recipient_persona'):
                        profile_context_user = auto_llm_target_item['target_recipient_persona']
                
                # Profile manager now holds data reloaded at start of iteration
                profile_for_llm_context = self.profile_manager.get_profile(profile_context_user)
                parsed_llm_output = self.llm_manager.generate_structured_query(
                    current_user_input, current_speaker_role_if_known=current_speaker_role,
                    shopping_for_user_profile_context=profile_for_llm_context
                )
            
            self.get_logger().info(f"LLM Parsed Output for \"{current_user_input}\": {json.dumps(parsed_llm_output, indent=2)}")

            if not parsed_llm_output or parsed_llm_output.get("error"):
                self.speech.say("Sorry, I had trouble understanding that. Please try rephrasing.")
                current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="general_clarification") 
                time.sleep(0.05); continue 
            
            intent = parsed_llm_output.get("intent")
            if intent == "stop_interaction":
                _running_main_loop = False 
                time.sleep(0.05); continue 

            shopping_for_user_this_query = None 
            llm_target = parsed_llm_output.get("shopping_for_user")

            if llm_target == "self": shopping_for_user_this_query = current_speaker_role
            elif llm_target in VALID_USER_KEYS: shopping_for_user_this_query = llm_target
            else: 
                relevant_intents = {"find_product", "find_store", "show_profile", "update_profile_only"}
                if intent in relevant_intents: 
                    self.speech.say(f"I understood: \"{current_user_input}\".")
                    clarified = self.clarify_shopping_target() 
                    if clarified: shopping_for_user_this_query = clarified
                    else: 
                        self.speech.say("Okay, what would you like to do then?")
                        current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="general_clarification")
                        time.sleep(0.05); continue 
                else: 
                    shopping_for_user_this_query = current_speaker_role 

            if shopping_for_user_this_query: 
                self.speech.say(f"Working on this for {shopping_for_user_this_query.capitalize()}.")
                if parsed_llm_output.get("updated_profile_for_shopping_user"):
                    self.profile_manager.update_basic_preferences(
                        shopping_for_user_this_query, parsed_llm_output["updated_profile_for_shopping_user"]
                    ) 
                active_profile = self.profile_manager.get_profile(shopping_for_user_this_query)
                
                session_status = await self.process_intent_no_ros(intent, parsed_llm_output, shopping_for_user_this_query, active_profile) 
            else: 
                self.speech.say("I'm not sure who this request is for. Let's try again.")
                current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="initial_query")
                time.sleep(0.05); continue

            self.get_logger().info(f"Main loop received session status: {session_status}")
            self.get_logger().info("-" * 60) 

            if session_status.get("status") == "new_request": 
                current_user_input = session_status.get("detail", "") 
            elif session_status.get("status") == "stop_session":
                _running_main_loop = False 
            else: 
                if _running_main_loop : 
                    self.speech.say(f"Is there anything else I can help with for your shopping trip, {current_speaker_role.capitalize()}? Or say 'stop'.")
                    current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="respond_to_anything_else") 
            
            time.sleep(0.05) 
            
        self.speech.say("Alright, ending our shopping trip. Have a great day!") 
        self.get_logger().info("Shopping session loop ended.")
        # Save the final state of profiles for THIS iteration.
        # If reloading is active, this saved state will be the starting point for the *next manual run of the script*,
        # but *not* for the next iteration within this current script execution.
        self.profile_manager._save_profiles() 

async def main_async_runner(kg_service, llm_parser, user_profiles_manager_instance, global_favorite_stores, args=None, iteration_num=0): 
    global _running_main_loop, current_speaker_role
    _running_main_loop = True 
    current_speaker_role = None 

    print(f"\n\n--- STARTING BENCHMARKING ITERATION {iteration_num + 1} ---\n")
    
    # *** KEY CHANGE: Reload profiles at the start of each iteration ***
    print(f"MAIN_RUNNER: Reloading profiles from file for iteration {iteration_num + 1}...")
    user_profiles_manager_instance.reload_profiles_from_file()
    # Display reloaded profiles for verification
    print("MAIN_RUNNER: Profiles after reload for this iteration:")
    for role_key_check in VALID_USER_KEYS:
        user_profiles_manager_instance.display_profile(role_key_check)
    print("-" * 30)


    main_controller = MainRobotController(kg_service_instance=kg_service,
                                          llm_manager_instance=llm_parser,
                                          profile_manager_instance=user_profiles_manager_instance) # Pass the (reloaded) manager
    USE_AUTOMATED_LLM_USER = True 
    
    personas_for_shopping_runner = ["mother", "father"] 
    if iteration_num < 10 : 
        AUTOMATED_USER_PERSONA = personas_for_shopping_runner[iteration_num % len(personas_for_shopping_runner)]
    else: 
        AUTOMATED_USER_PERSONA = random.choice(VALID_USER_KEYS) 

    print(f"INFO: Automated user persona for iteration {iteration_num + 1}: {AUTOMATED_USER_PERSONA}")

    if USE_AUTOMATED_LLM_USER:
        auto_llm_user = AutomatedBenchmarkingLLM(persona=AUTOMATED_USER_PERSONA)
        
        sample_products_for_llm = []
        if main_controller.kg_service and main_controller.kg_service.graph:
            all_product_nodes = [(pid, data) for pid, data in main_controller.kg_service.graph.nodes(data=True) if data.get('label_node') == 'Product']
            
            temp_products_with_stores = []
            for pid, pdata in all_product_nodes:
                product_stores = []
                if main_controller.kg_service.graph.has_node(pid):
                    for successor_id in main_controller.kg_service.graph.successors(pid):
                        if main_controller.kg_service.graph.has_node(successor_id):
                            successor_node_data = main_controller.kg_service.graph.nodes[successor_id]
                            if successor_node_data.get('label_node') == 'Store':
                                product_stores.append({
                                    'store_id': successor_id,
                                    'store_name': successor_node_data.get('name')
                                })
                
                if product_stores:
                    chosen_store_info = random.choice(product_stores)
                    augmented_pdata = pdata.copy() 
                    augmented_pdata['store_id'] = chosen_store_info['store_id']
                    augmented_pdata['store_name'] = chosen_store_info['store_name']
                    temp_products_with_stores.append(augmented_pdata)
            
            if temp_products_with_stores:
                sample_size = min(len(temp_products_with_stores), 30) 
                if sample_size > 0: 
                    sample_products_for_llm = random.sample(temp_products_with_stores, sample_size)
                else:
                    sample_products_for_llm = []
            else:
                sample_products_for_llm = []
        
        knowledge_for_auto_user = {
            "item_categories": kg_service.get_all_item_categories(),
            "brands": kg_service.get_all_brands(),
            "colors": kg_service.get_all_colors(),
            "store_names": kg_service.get_all_store_names(),
            "sample_products": sample_products_for_llm,
            "global_favorite_stores": global_favorite_stores, 
            "current_iteration_num": iteration_num           
        }
        speech_interface_global.set_automated_llm_user(auto_llm_user, knowledge_for_auto_user)
        print(f"\n<<<<< RUNNING IN AUTOMATED BENCHMARKING MODE (Persona: {AUTOMATED_USER_PERSONA}) - Iteration {iteration_num + 1} >>>>>\n")
    else:
        speech_interface_global.set_automated_llm_user(None, {}) 
    
    try: await main_controller.run_shopping_session_loop() 
    except KeyboardInterrupt: main_controller.get_logger().info(f"KeyboardInterrupt received during iteration {iteration_num + 1}, shutting down this iteration.")
    except Exception as e: main_controller.get_logger().error(f"Unhandled exception during iteration {iteration_num + 1}: {e}", exc_info=True)
    finally:
        _running_main_loop = False 
        main_controller.get_logger().info(f"Shutting down MainRobotController (NO ROS) for iteration {iteration_num + 1}...")

    # Profiles displayed here will show changes made *during this iteration only*,
    # as the next iteration will start by reloading from the file.
    print("\n" + "="*20 + f" Final User Profiles (in-memory) After Iteration {iteration_num + 1} " + "="*20)
    for role_key_check in VALID_USER_KEYS: user_profiles_manager_instance.display_profile(role_key_check)
    print("="*60 + f"\nSession for iteration {iteration_num + 1} fully complete.\n" + f"--- COMPLETED BENCHMARKING ITERATION {iteration_num + 1} ---\n")

if __name__ == "__main__":
    num_iterations = 10 
    results_collector = []
    all_personas_metrics = {"mother": [], "father": [], "child": []} 
    
    print("--- Initializing Shared Services for Benchmarking ---")
    shared_kg_service_instance = KnowledgeGraphService()
    shared_llm_manager_instance = GroqQueryParser()
    prefs_file = "family_preferences_main.json" # This is the master file
    
    # Create the UserProfileManager instance ONCE.
    # Its reload_profiles_from_file() method will be called at the start of each iteration.
    shared_profile_manager_instance = UserProfileManager(users_file=prefs_file)

    # Make sure the prefs_file exists with a base structure if it's the very first run ever
    # The UserProfileManager's __init__ now handles creating/normalizing the file.
    if not os.path.exists(prefs_file):
        print(f"WARNING: Preference file {prefs_file} did not exist. UserProfileManager will create it with defaults.")
        # No need to manually create it here, UserProfileManager handles it.

    shared_llm_manager_instance.set_knowledge_base_lists(
        store_names=shared_kg_service_instance.get_all_store_names(),
        item_categories=shared_kg_service_instance.get_all_item_categories(),
        brands=shared_kg_service_instance.get_all_brands(),
        colors=shared_kg_service_instance.get_all_colors()
    )
    print("--- Shared Services Initialized ---")

    all_store_names_from_kg = shared_kg_service_instance.get_all_store_names()
    global_favorite_stores = []
    if not all_store_names_from_kg:
        print("CRITICAL WARNING: No stores found in KG. Favorite store logic will be impacted.")
    elif len(all_store_names_from_kg) < 3:
        print(f"WARNING: Only {len(all_store_names_from_kg)} stores in KG. Using all of them as global favorite stores.")
        global_favorite_stores = all_store_names_from_kg
    else:
        global_favorite_stores = random.sample(all_store_names_from_kg, k=3)
    print(f"--- Global Favorite Stores for this benchmarking session: {global_favorite_stores} ---")


    for i in range(num_iterations):
        print(f"==================== ITERATION {i + 1} OF {num_iterations} ====================")
        asyncio.run(main_async_runner(kg_service=shared_kg_service_instance,
                                      llm_parser=shared_llm_manager_instance,
                                      user_profiles_manager_instance=shared_profile_manager_instance, # Pass the manager
                                      global_favorite_stores=global_favorite_stores, 
                                      iteration_num=i))
        
        if speech_interface_global.automated_llm_user:
            metrics = speech_interface_global.automated_llm_user.get_session_metrics()
            results_collector.append(metrics)
            current_persona_metric = metrics.get("persona")
            if current_persona_metric in all_personas_metrics:
                 all_personas_metrics[current_persona_metric].append(metrics)
            print(f"ITERATION {i+1} METRICS (Persona: {current_persona_metric}): {metrics}")
        
        # The family_preferences_main.json file will be updated at the end of each
        # run_shopping_session_loop. If reloading is active, the *next* iteration
        # will start from the state of this file as it was *before this current iteration started*.
        time.sleep(0.5) 
    
    # After all iterations, save the final state of the profiles.
    # This reflects all changes made in the *last* iteration if reloading was active.
    # If you want the file to reflect the *original state* after all iterations,
    # you'd need to make a backup of the original file and restore it here.
    # Current setup: family_preferences_main.json will contain updates from the *last* iteration.
    print("\n--- Final state of profiles after all iterations (saved to file reflects last iteration's changes) ---")
    shared_profile_manager_instance._save_profiles() # Ensure final save
    for role_key_check in VALID_USER_KEYS:
        shared_profile_manager_instance.display_profile(role_key_check)


    if results_collector:
        success_values = [r['success'] for r in results_collector]
        avg_success_rate = sum(success_values) / len(success_values) if success_values else 0
        print(f"\nOverall Average Session Success Rate across {num_iterations} iterations: {avg_success_rate:.2%}")

        plt.figure(figsize=(12, 6))
        iterations_x_axis = range(1, len(success_values) + 1)
        persona_colors_map = {"mother": "skyblue", "father": "lightgreen", "child": "lightcoral"}
        bar_colors = [persona_colors_map.get(r['persona'], 'gray') for r in results_collector]
        
        plt.subplot(1, 2, 1)
        bars = plt.bar(iterations_x_axis, [1 if r['success'] else 0 for r in results_collector], color=bar_colors, label='Success (1) / Fail (0)')
        plt.xlabel("Iteration Number")
        plt.ylabel("Session Success (All items found / Child session)")
        plt.title(f"Session Success Rate per Iteration (Avg: {avg_success_rate:.2%})")
        plt.yticks([0, 1], ['Fail', 'Success'])
        if num_iterations <= 20 : plt.xticks(iterations_x_axis)
        
        personas_in_plot_legend = sorted(list(set(r['persona'] for r in results_collector)))
        legend_elements = [plt.Rectangle((0,0),1,1, color=persona_colors_map[p], label=p.capitalize()) for p in personas_in_plot_legend if p in persona_colors_map]
        plt.legend(handles=legend_elements, title="Persona")
        
        interaction_counts = [r['interaction_count'] for r in results_collector]
        avg_interaction_count = sum(interaction_counts) / len(interaction_counts) if interaction_counts else 0
        
        plt.subplot(1, 2, 2)
        plt.plot(iterations_x_axis, interaction_counts, marker='o', linestyle='-', color='coral')
        plt.axhline(y=avg_interaction_count, color='r', linestyle='--', label=f'Avg Interactions ({avg_interaction_count:.2f})')
        plt.xlabel("Iteration Number")
        plt.ylabel("Number of Interactions (LLM User)")
        plt.title("Interaction Count per Iteration")
        if num_iterations <= 20 : plt.xticks(iterations_x_axis)
        else: plt.locator_params(axis='x', nbins=10)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.tight_layout() 
        plt.savefig("session_metrics_benchmark.png") 
        print("Combined session success and interaction count plot saved to session_metrics_benchmark.png")

        if num_iterations >= 3:
            rolling_window = max(1, num_iterations // 3) 
            successes_numeric = [1 if r['success'] else 0 for r in results_collector]
            if len(successes_numeric) >= rolling_window: 
                rolling_avg_success = np.convolve(successes_numeric, np.ones(rolling_window), 'valid') / rolling_window
                
                plt.figure(figsize=(10, 5)) 
                plt.plot(range(rolling_window -1, num_iterations), rolling_avg_success, marker='x', linestyle='-', color='purple')
                plt.xlabel("Iteration Number (end of window)")
                plt.ylabel(f"Rolling Avg. Success Rate (window={rolling_window})")
                plt.title("Trend: Rolling Average Session Success Rate")
                plt.ylim(-0.05, 1.05)
                plt.grid(True, linestyle='--', alpha=0.7)
                plt.tight_layout()
                plt.savefig("success_rate_trend_benchmark.png")
                print("Success rate trend plot saved to success_rate_trend_benchmark.png")
            else:
                print(f"Not enough data ({len(successes_numeric)} points) for rolling average with window {rolling_window}.")


        fig_persona, ax_persona = plt.subplots(figsize=(8,5))
        personas_in_results_set = sorted(list(all_personas_metrics.keys())) # Use keys from all_personas_metrics
        
        avg_interactions_values = []
        valid_personas_for_bar = []

        for p_key in personas_in_results_set: # Iterate through the set of personas that actually ran
            p_metrics = all_personas_metrics[p_key]
            if p_metrics: 
                avg_int = sum(m['interaction_count'] for m in p_metrics) / len(p_metrics)
                avg_interactions_values.append(avg_int)
                valid_personas_for_bar.append(p_key.capitalize())
        
        if valid_personas_for_bar:
            bar_colors_persona = [persona_colors_map.get(p.lower(), 'gray') for p in valid_personas_for_bar] 
            ax_persona.bar(valid_personas_for_bar, avg_interactions_values, color=bar_colors_persona)
            ax_persona.set_xlabel("User Persona")
            ax_persona.set_ylabel("Average Interactions per Session")
            ax_persona.set_title("Benchmarking: Avg. Interactions by User Persona")
            fig_persona.tight_layout()
            fig_persona.savefig("benchmark_interaction_counts_by_persona.png")
            print("Per-persona average interaction count plot saved to benchmark_interaction_counts_by_persona.png")

    else:
        print("No results collected for plotting.")
    
    print(f"\nBenchmarking complete. User profiles have been updated in {prefs_file} (reflects changes from the last iteration).")