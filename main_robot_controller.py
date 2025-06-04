#!/usr/bin/env python3
# main_robot_controller.py (NO ROS VERSION)

import os
import json
import math # For math.radians and yaw_to_quaternion
import time # For sleep
import random # For mock navigation time

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
speech_interface_global = SpeechInterface()
_running_main_loop = True 

import asyncio

def _yaw_to_quaternion(yaw_radians): 
    q = {} 
    q['x'] = 0.0; q['y'] = 0.0
    q['z'] = math.sin(yaw_radians / 2.0)
    q['w'] = math.cos(yaw_radians / 2.0)
    return q

class MainRobotController: 
    def __init__(self):
        self._logger = MockLogger() 
        self.get_logger().info("MainRobotController (NO ROS) initializing...")

        self.speech = speech_interface_global
        self.profile_manager = UserProfileManager(users_file="family_preferences_main.json")
        self.kg_service = KnowledgeGraphService()
        self.recommender = RecommenderEngine(knowledge_graph_service=self.kg_service, user_profile_manager=self.profile_manager)
        self.llm_manager = GroqQueryParser()

        self.llm_manager.set_knowledge_base_lists(
            store_names=self.kg_service.get_all_store_names(),
            item_categories=self.kg_service.get_all_item_categories(),
            brands=self.kg_service.get_all_brands(),
            colors=self.kg_service.get_all_colors()
        )

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
            "father": ["father", "dad", "papa", "husband", "hubby", "man of the house"],
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
            if not role_text: role_text = "" # Handle None from listen_and_get_text
            self.get_logger().info(f"[SPEAKER_ID] User said: {role_text}")
            role = self.identify_role_from_text(role_text)
            if role:
                current_speaker_role = role
                self.speech.say(f"Thank you, {current_speaker_role.capitalize()}."); return
            self.speech.say("I'm sorry, I didn't quite catch that. Could you please say if you are the Mother, Father, or Child?")
        current_speaker_role = "mother" 
        self.speech.say(f"I'm having a little trouble understanding. I'll assume I'm speaking with Mother for now.")

    def get_confirmation_dialogue(self, prompt_text, max_retries=2, dialogue_state_hint_for_auto_llm=None, context_for_auto_llm=None):
        self.speech.say(prompt_text)
        for i in range(max_retries):
            response_text = self.speech.listen_and_get_text(
                dialogue_state_hint=dialogue_state_hint_for_auto_llm,
                context_for_auto_llm=context_for_auto_llm
            )
            if not response_text: response_text = "" # Handle None
            response = response_text.lower()
            self.get_logger().info(f"[CONFIRMATION] For '{prompt_text}', user said: '{response}'")
            if any(word in response for word in ["yes", "sure", "okay", "ok", "yep", "please", "do it", "let's go", "affirmative", "correct", "indeed"]): return True
            if any(word in response for word in ["no", "nope", "don't", "skip", "cancel", "negative", "not now", "incorrect"]): return False
            if i < max_retries - 1: self.speech.say("Sorry, I didn't understand. Please answer with a yes or no.")
        self.speech.say("Okay, I'll take that as a no for now."); return False

    def clarify_shopping_target(self):
        global current_speaker_role 
        self.speech.say("And who is this request for? (Mother, Father, or Child)")
        for _ in range(2): 
            response_text = self.speech.listen_and_get_text(
                 dialogue_state_hint="clarify_shopping_target",
                 context_for_auto_llm={"robot_query_summary": "this request"}
            )
            if not response_text: response_text = ""
            self.get_logger().info(f"[CLARIFY_TARGET] User said: '{response_text}'")
            target_role = self.identify_role_from_text(response_text)
            if target_role: return target_role
            self.speech.say("Sorry, I didn't catch that clearly. Is it for Mother, Father, or Child?")
        self.speech.say(f"Okay, I'll assume this request is for {current_speaker_role.capitalize()}."); return current_speaker_role

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

        any_request_fulfilled_overall = False
        for i, store_option_details in enumerate(ranked_store_options_with_details):
            store_info = store_option_details["store_info"]
            product_count_if_any = store_option_details["product_count_if_any"]
            store_name = store_info.get('name', 'an unnamed store')
            dist_val = store_info.get('distance', float('inf'))
            dist_str = f" (Est. Distance: {dist_val:.1f}m)" if dist_val < float('inf') else "" 
            proposal_intro = f"For {request_summary},"
            proposal = f"{proposal_intro} {'I found ' + str(product_count_if_any) + ' matching item(s) at ' if product_count_if_any is not None else 'one potential place is '}{store_name}{dist_str}."
            is_last_option_in_list = (i == len(ranked_store_options_with_details) - 1)
            confirm_context = {"store_name": store_name, "item_summary": request_summary}
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
            current_item_fulfilled = fulfillment_status_llm.get("fulfilled", False) if not fulfillment_status_llm.get("error") else False
            if current_item_fulfilled: self.speech.say("That's great!"); any_request_fulfilled_overall = True
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
                    profile_updates_llm["shop_review_update"]["last_visit_fulfilled"] = current_item_fulfilled 
                    self.profile_manager.update_profile_with_feedback(shopping_for_user_role, store_name, profile_updates_llm)
                    self.speech.say(f"Thanks! Updated notes for {shopping_for_user_role.capitalize()}.")
                else: self.speech.say("Thanks for the feedback. (Could not parse for profile)")
            else: self.speech.say("Okay.")

            has_more_options = not is_last_option_in_list
            next_action_prompt = "What next? "
            if any_request_fulfilled_overall: next_action_prompt += "Look for something new, or stop? "
            else: next_action_prompt += f"Still looking for {request_summary}. "
            if has_more_options: next_action_prompt += f"Or, I can show other options for {request_summary}."
            elif not any_request_fulfilled_overall: next_action_prompt += f"That was the last option for {request_summary}. Look for something new, or stop? "
                
            self.speech.say(next_action_prompt)
            next_action_context = {"original_request_summary": request_summary, "was_last_item_fulfilled": any_request_fulfilled_overall, "has_more_options": has_more_options}
            next_action_response_text = self.speech.listen_and_get_text(dialogue_state_hint="decide_next_action", context_for_auto_llm=next_action_context)
            if not next_action_response_text: next_action_response_text = ""
            next_action_llm = self.llm_manager.parse_next_action_decision(next_action_response_text, request_summary, any_request_fulfilled_overall, has_more_options)

            if next_action_llm.get("error"): self.speech.say("Unsure what to do next. Ending this search."); return {"status": "completed_recommendations_for_query"}
            intent_next = next_action_llm.get("intent")
            new_query = next_action_llm.get("new_query_text")

            if intent_next == "new_request" and new_query: self.speech.say(f"Okay, looking into: {new_query}"); return {"status": "new_request", "detail": new_query}
            elif intent_next == "stop_interaction": return {"status": "stop_session"} # Let main loop handle final goodbye
            elif intent_next == "continue_current_request":
                if has_more_options: self.speech.say(f"Okay, next option for {request_summary}.")
                else: self.speech.say(f"Actually, that was the last option for {request_summary}."); return {"status": "completed_recommendations_for_query"}
            else: self.speech.say("Assuming we're done with that request."); return {"status": "completed_recommendations_for_query"}
            time.sleep(0.05) 

        if not any_request_fulfilled_overall: self.speech.say(f"We've gone through all suggestions for {request_summary}, but didn't find it.")
        else: self.speech.say(f"We've explored options for {request_summary}.")
        return {"status": "completed_recommendations_for_query"}

    async def process_intent_no_ros(self, intent_from_llm, parsed_llm_data, shopping_for_user_role, active_profile_for_recs): 
        if intent_from_llm == "show_profile":
            self.speech.say(f"Preferences for {shopping_for_user_role.capitalize()}:")
            self.profile_manager.display_profile(shopping_for_user_role)
            return {"status": "interaction_completed_for_current_query"}
        if intent_from_llm == "update_profile_only":
            self.speech.say(f"Preferences updated for {shopping_for_user_role.capitalize()}.")
            self.profile_manager.display_profile(shopping_for_user_role) 
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
        _running_main_loop = True # Session starts, loop should run

        self.speech.wait_for_wake_word() 
        self.initialize_speaker_if_needed()

        if not current_speaker_role:
            self.get_logger().error("Speaker role not initialized. Cannot start session.")
            self.speech.say("Startup error: Speaker not identified.")
            return 

        self.speech.say(f"Hello {current_speaker_role.capitalize()}! How can I help you or the family today? (Or say 'stop' to end.)")
        current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="initial_query")

        while _running_main_loop: 
            if not current_user_input: 
                self.get_logger().warn("Received no input (likely EOF or Ctrl+C in manual), treating as stop.")
                parsed_llm_output = {"intent": "stop_interaction"}
            elif current_user_input.strip().lower() == "stop": 
                parsed_llm_output = {"intent": "stop_interaction"}
            else:
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
                _running_main_loop = False # Signal loop to stop
                # No break here, loop condition will handle termination at the end of this iteration.

            if not _running_main_loop: # If loop is signalled to stop, skip further processing
                time.sleep(0.05); continue # Go to top of while, which will now exit

            # --- Process non-stop intents ---
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
                else: shopping_for_user_this_query = current_speaker_role 

            self.speech.say(f"Working on this for {shopping_for_user_this_query.capitalize()}.")
            if parsed_llm_output.get("updated_profile_for_shopping_user"):
                self.profile_manager.update_basic_preferences(
                    shopping_for_user_this_query, parsed_llm_output["updated_profile_for_shopping_user"]
                ) 
            active_profile = self.profile_manager.get_profile(shopping_for_user_this_query)
            
            session_status = await self.process_intent_no_ros(intent, parsed_llm_output, shopping_for_user_this_query, active_profile) 
            
            self.get_logger().info(f"Main loop received session status: {session_status}")
            self.get_logger().info("-" * 60) 

            if session_status.get("status") == "new_request":
                current_user_input = session_status.get("detail", "")
            elif session_status.get("status") == "stop_session":
                _running_main_loop = False 
            else: # "interaction_completed_for_current_query" or "completed_recommendations_for_query"
                if _running_main_loop : # Only ask if we are not already stopping
                    self.speech.say(f"Anything else for you or the family, {current_speaker_role.capitalize()}? (Or say 'stop')")
                    current_user_input = self.speech.listen_and_get_text(dialogue_state_hint="respond_to_anything_else")
            
            time.sleep(0.05) 
            
        # Loop has ended
        self.speech.say("Alright, ending our shopping trip. Have a great day!") 
        self.get_logger().info("Shopping session loop ended.")
        self.profile_manager._save_profiles()

async def main_async_runner(args=None, iteration_num=0): 
    global _running_main_loop, current_speaker_role
    _running_main_loop = True 
    current_speaker_role = None 

    print(f"\n\n--- STARTING BENCHMARKING ITERATION {iteration_num + 1} ---\n")
    prefs_file_path = "family_preferences_main.json"
    if os.path.exists(prefs_file_path):
        try: os.remove(prefs_file_path); print(f"Removed old preferences file: {prefs_file_path} for fresh test for iteration {iteration_num + 1}.")
        except OSError as e: print(f"Warning: Could not remove {prefs_file_path} for iteration {iteration_num + 1}: {e}")

    main_controller = MainRobotController()
    USE_AUTOMATED_LLM_USER = True 
    AUTOMATED_USER_PERSONA = random.choice(["mother", "father", "child"]) 
    print(f"INFO: Automated user persona for iteration {iteration_num + 1}: {AUTOMATED_USER_PERSONA}")

    if USE_AUTOMATED_LLM_USER:
        auto_llm_user = AutomatedBenchmarkingLLM(persona=AUTOMATED_USER_PERSONA)
        sample_products_for_llm = []
        if main_controller.kg_service and main_controller.kg_service.graph:
            all_products = [data for _, data in main_controller.kg_service.graph.nodes(data=True) if data.get('label_node') == 'Product']
            if all_products:
                sample_size = min(len(all_products), 10) 
                sample_products_for_llm = random.sample(all_products, sample_size)
        knowledge_for_auto_user = {
            "item_categories": main_controller.kg_service.get_all_item_categories(),
            "brands": main_controller.kg_service.get_all_brands(),
            "colors": main_controller.kg_service.get_all_colors(),
            "store_names": main_controller.kg_service.get_all_store_names(),
            "sample_products": sample_products_for_llm 
        }
        speech_interface_global.set_automated_llm_user(auto_llm_user, knowledge_for_auto_user)
        main_controller.speech = speech_interface_global 
        print(f"\n<<<<< RUNNING IN AUTOMATED BENCHMARKING MODE (Persona: {AUTOMATED_USER_PERSONA}) - Iteration {iteration_num + 1} >>>>>\n")
    else:
        speech_interface_global.set_automated_llm_user(None, {}) 
        main_controller.speech = speech_interface_global
    
    try: await main_controller.run_shopping_session_loop() 
    except KeyboardInterrupt: main_controller.get_logger().info(f"KeyboardInterrupt received during iteration {iteration_num + 1}, shutting down this iteration.")
    except Exception as e: main_controller.get_logger().error(f"Unhandled exception during iteration {iteration_num + 1}: {e}", exc_info=True)
    finally:
        _running_main_loop = False # Ensure loop condition is false for any external break
        main_controller.get_logger().info(f"Shutting down MainRobotController (NO ROS) for iteration {iteration_num + 1}...")

    print("\n" + "="*20 + f" Final User Profiles After Iteration {iteration_num + 1} " + "="*20)
    for role_key_check in VALID_USER_KEYS: main_controller.profile_manager.display_profile(role_key_check)
    print("="*60 + f"\nSession for iteration {iteration_num + 1} fully complete.\n" + f"--- COMPLETED BENCHMARKING ITERATION {iteration_num + 1} ---\n")

if __name__ == "__main__":
    num_iterations = 10
    for i in range(num_iterations):
        print(f"==================== PREPARING ITERATION {i + 1} / {num_iterations} ====================")
        asyncio.run(main_async_runner(iteration_num=i))
        if i < num_iterations - 1:
            print("\nWaiting a moment before next iteration...\n"); time.sleep(1) # Reduced sleep
    print(f"\n\n==================== ALL {num_iterations} ITERATIONS COMPLETED ====================")