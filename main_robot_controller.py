#!/usr/bin/env python3
# main_robot_controller.py

import os
import json
import math # For math.radians and yaw_to_quaternion
import time # For sleep

# --- ROS2 Imports ---
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion as RosQuaternion
from action_msgs.msg import GoalStatus
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# --- Import Application Modules ---
from llm_groq_parser import GroqQueryParser # Ensure this file exists and is correct
from speech_interface import SpeechInterface # Ensure this file exists
from user_manager import UserProfileManager, VALID_USER_KEYS # Ensure this file exists
from mall_query_engine import KnowledgeGraphService # Ensure this file exists
from recommendation_engine import RecommenderEngine # Ensure this file exists

# --- Global State (outside class for script compatibility with provided structure) ---
current_speaker_role = None 
speech_interface_global = SpeechInterface() # Global instance for speech

def _yaw_to_quaternion(yaw_radians):
    """Helper function to convert yaw angle (in radians) to ROS Quaternion."""
    q = RosQuaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_radians / 2.0)
    q.w = math.cos(yaw_radians / 2.0)
    return q

class MainRobotControllerNode(Node):
    def __init__(self):
        super().__init__('main_robot_controller_node')
        self.get_logger().info("MainRobotControllerNode initializing...")

        # --- Component Initialization ---
        self.speech = speech_interface_global # Use the global instance
        self.profile_manager = UserProfileManager(users_file="family_preferences_main.json")
        self.kg_service = KnowledgeGraphService()
        self.recommender = RecommenderEngine(knowledge_graph_service=self.kg_service, user_profile_manager=self.profile_manager)
        self.llm_manager = GroqQueryParser() # LLM parser

        # Provide LLM with knowledge about the mall's entities
        self.llm_manager.set_knowledge_base_lists(
            store_names=self.kg_service.get_all_store_names(),
            item_categories=self.kg_service.get_all_item_categories(),
            brands=self.kg_service.get_all_brands(),
            colors=self.kg_service.get_all_colors()
        )

        # --- ROS2 Navigation Action Client ---
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("NavigateToPose action client created.")
        self._check_nav_server_ready() # Check and log server readiness

        # --- TF Buffer and Listener for Robot Pose ---
        self.tf_buffer = Buffer()
        # Spin_thread=True is good for TransformListener if it doesn't own the main spin.
        # However, since we explicitly spin_once in loops, it might be okay without,
        # but True is safer for background TF updates.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 
        self.get_logger().info("TF Listener initialized for robot pose.")
        
        self.get_logger().info("MainRobotControllerNode initialized successfully.")

    def _check_nav_server_ready(self, timeout_sec=5.0):
        """Checks if the Nav2 action server is ready, waits if necessary."""
        if not self._nav_action_client.server_is_ready():
            self.get_logger().warn("NavigateToPose action server not available. Waiting...")
            if not self._nav_action_client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error("NavigateToPose action server not available after timeout. Navigation will fail.")
                return False
            self.get_logger().info("NavigateToPose action server is ready.")
        return True

    def send_navigation_goal(self, x, y, theta_degrees):
        """Sends a navigation goal to Nav2 and waits for the result."""
        self.get_logger().info(f"Attempting to navigate to x={x:.2f}, y={y:.2f}, theta={theta_degrees:.2f} deg.")
        if not self._check_nav_server_ready(timeout_sec=1.0): # Quick check before sending
             self.get_logger().error("Navigation server not ready, cannot send goal.")
             return False

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map' # Assuming goals are in the map frame
        goal_pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        
        theta_radians = math.radians(float(theta_degrees))
        orientation_q = _yaw_to_quaternion(theta_radians)
        goal_pose.pose.orientation = orientation_q
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal to Nav2: Pos({x:.2f}, {y:.2f}), Ori Degrees({theta_degrees:.2f})")
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        
        # Spin until send_goal_future is done
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0) # Timeout for goal sending

        if not send_goal_future.done() or send_goal_future.exception():
            exc = send_goal_future.exception()
            self.get_logger().error(f"Goal sending failed or rejected by server: {exc if exc else 'Timeout or unknown error during send_goal'}")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the Nav2 server.')
            return False

        self.get_logger().info('Goal accepted by Nav2 server. Waiting for navigation result...')
        get_result_future = goal_handle.get_result_async()

        # Spin until get_result_future is done
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=120.0) # Long timeout for navigation itself

        if not get_result_future.done() or get_result_future.exception():
            exc = get_result_future.exception()
            self.get_logger().error(f"Error while getting navigation goal result: {exc if exc else 'Timeout or unknown error during get_result'}")
            # You might want to check goal_handle.status here if available and useful
            return False

        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Navigation goal failed with status: {GoalStatus.STATUS_STRING[status] if hasattr(GoalStatus, "STATUS_STRING") else status}')
            return False

    def get_current_robot_pose(self):
        """
        Returns the current robot pose as (x, y, theta_radians) in the 'map' frame.
        Returns (None, None, None) on error.
        """
        if not (self.tf_buffer and self.tf_listener): # Should always be true after init
            self.get_logger().warn("TF buffer or listener not available for getting current pose.")
            return None, None, None
        try:
            # Wait for the transform to be available, with a timeout
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame='map', source_frame='base_link', time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = transform_stamped.transform.translation.x
            y = transform_stamped.transform.translation.y
            q = transform_stamped.transform.rotation
            # Standard conversion from quaternion to yaw
            yaw_radians = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            # self.get_logger().debug(f"Current robot pose: x={x:.2f}, y={y:.2f}, yaw_deg={math.degrees(yaw_radians):.2f}")
            return x, y, yaw_radians
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform base_link to map: {ex}. Pose unavailable.')
            return None, None, None
        except Exception as e: 
            self.get_logger().error(f'Unexpected error getting current pose: {e}. Pose unavailable.')
            return None, None, None

    # --- DIALOGUE MANAGEMENT AND CORE LOGIC METHODS ---
    def identify_role_from_text(self, text):
        """Identifies user role (mother, father, child) from text."""
        text_lower = text.lower()
        mapping = {
            "father": ["father", "dad", "papa"],
            "mother": ["mother", "mum", "mom", "mama"],
            "child": ["child", "kid", "son", "daughter", "boy", "girl"]
        }
        for role, keywords in mapping.items():
            if any(word in text_lower for word in keywords):
                return role
        return None

    def initialize_speaker_if_needed(self):
        """
        Identifies the primary speaker at the beginning of the session.
        """
        global current_speaker_role
        if current_speaker_role: 
            return

        self.speech.say("Before we begin, who will I be primarily talking with today? Please say Mother, Father, or Child.")
        for _ in range(3): 
            role_text = self.speech.listen_and_get_text()
            self.get_logger().info(f"[SPEAKER_ID] User said: {role_text}")
            role = self.identify_role_from_text(role_text)
            if role:
                current_speaker_role = role
                self.speech.say(f"Thank you, {current_speaker_role.capitalize()}.")
                return
            self.speech.say("I'm sorry, I didn't quite catch that. Could you please say if you are the Mother, Father, or Child?")

        current_speaker_role = "mother" # Default if identification fails
        self.speech.say(f"I'm having a little trouble understanding. I'll assume I'm speaking with Mother for now.")

    def get_confirmation_dialogue(self, prompt_text, max_retries=2):
        """Asks a yes/no question and returns True for yes, False for no. (Renamed to avoid conflict if used elsewhere)"""
        self.speech.say(prompt_text)
        for i in range(max_retries):
            response = self.speech.listen_and_get_text().lower()
            self.get_logger().info(f"[CONFIRMATION] For '{prompt_text}', user said: '{response}'")
            if any(word in response for word in ["yes", "sure", "okay", "ok", "yep", "please", "do it", "let's go", "affirmative", "correct", "indeed"]):
                return True
            if any(word in response for word in ["no", "nope", "don't", "skip", "cancel", "negative", "not now", "incorrect"]):
                return False
            if i < max_retries - 1:
                self.speech.say("Sorry, I didn't understand. Please answer with a yes or no.")
        self.speech.say("Okay, I'll take that as a no for now.") 
        return False

    def clarify_shopping_target(self):
        """Asks the speaker who the current shopping request is for."""
        global current_speaker_role # Access global
        self.speech.say("And who is this request for? (Mother, Father, or Child)")
        for _ in range(2): 
            response = self.speech.listen_and_get_text()
            self.get_logger().info(f"[CLARIFY_TARGET] User said: '{response}'")
            target_role = self.identify_role_from_text(response)
            if target_role:
                return target_role
            self.speech.say("Sorry, I didn't catch that clearly. Is it for Mother, Father, or Child?")
        
        self.speech.say(f"Okay, I'll assume this request is for {current_speaker_role.capitalize()}.")
        return current_speaker_role

    def _generate_request_summary_for_dialogue(self, parsed_llm_data):
        """Generates a brief human-readable summary of the user's request."""
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
        if store:
            return f"finding {', '.join(desc_parts) if desc_parts else 'items'} at {store}" if desc_parts or not item_str else f"information about {store}"
        return f"finding {' '.join(desc_parts)}" if desc_parts else "your general request"

    def _handle_iterative_store_visits_ros(self, original_request_parsed_data, ranked_store_options_with_details, shopping_for_user_role, active_profile_for_recs):
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
            dist_str = f" (Distance: {dist_val:.1f}m)" if dist_val < float('inf') else ""

            proposal_intro = f"For {request_summary},"
            proposal = f"{proposal_intro} {'I found ' + str(product_count_if_any) + ' matching item(s) at ' if product_count_if_any is not None else 'one potential place is '}{store_name}{dist_str}."
            
            is_last_option_in_list = (i == len(ranked_store_options_with_details) - 1)

            if not self.get_confirmation_dialogue(f"{proposal} Shall we go there?"):
                self.speech.say(f"Okay, we'll skip {store_name} for now.")
                if is_last_option_in_list: self.speech.say(f"That was the last option I had for {request_summary}.")
                continue

            map_x, map_y, map_theta = store_info.get('map_x'), store_info.get('map_y'), store_info.get('map_theta', 0.0)
            if map_x is None or map_y is None:
                self.speech.say(f"Sorry, I don't have precise location data to navigate to {store_name}.")
            elif self.send_navigation_goal(map_x, map_y, map_theta):
                self.speech.say(f"We've arrived at {store_name}. Please look around. Let me know when you're ready.")
                self.speech.listen_and_get_text() # User says "okay", "ready"
            else:
                self.speech.say(f"Sorry, I encountered an issue navigating to {store_name}.")
                if not is_last_option_in_list and self.get_confirmation_dialogue("Try the next store?"): continue
                else:
                    self.speech.say(f"Okay, stopping search for {request_summary}.")
                    return {"status": "completed_recommendations_for_query"}

            # Post-visit interaction
            self.speech.say(f"Regarding {request_summary}, did you find what you were looking for at {store_name}?")
            fulfillment_response = self.speech.listen_and_get_text()
            fulfillment_status_llm = self.llm_manager.parse_fulfillment_status(fulfillment_response, request_summary)
            current_item_fulfilled = fulfillment_status_llm.get("fulfilled", False) if not fulfillment_status_llm.get("error") else False
            
            if current_item_fulfilled: self.speech.say("That's great!"); any_request_fulfilled_overall = True
            else: self.speech.say(f"Oh, sorry you couldn't find {request_summary} there.")

            self.speech.say(f"Any other feedback about {store_name}? (e.g., selection, service) Or say 'no feedback'.")
            feedback_text = self.speech.listen_and_get_text()
            if feedback_text.strip().lower() not in ["no feedback", "nothing", "skip", "no"]:
                profile_updates_llm = self.llm_manager.parse_feedback_to_profile_update(
                    feedback_text, store_name, original_request_parsed_data.get("item_types", [])
                )
                if profile_updates_llm and not profile_updates_llm.get("error"):
                    if "shop_review_update" not in profile_updates_llm or profile_updates_llm["shop_review_update"] is None:
                        profile_updates_llm["shop_review_update"] = {}
                    profile_updates_llm["shop_review_update"]["request_fulfilled"] = current_item_fulfilled
                    self.profile_manager.update_profile_with_feedback(shopping_for_user_role, store_name, profile_updates_llm)
                    self.speech.say(f"Thanks! Updated notes for {shopping_for_user_role.capitalize()}.")
                else: self.speech.say("Thanks for the feedback. (Could not parse for profile)")
            else: self.speech.say("Okay.")

            # Decide next action
            has_more_options = not is_last_option_in_list
            next_action_prompt = "What next? "
            if any_request_fulfilled_overall:
                next_action_prompt += "Look for something new, or stop? "
                if has_more_options: next_action_prompt += f"Or, I can show other options for {request_summary}."
            else:
                if has_more_options: next_action_prompt += f"Other suggestions for {request_summary}? Or look for something new, or stop? "
                else: next_action_prompt += f"That was the last option for {request_summary}. Look for something new, or stop? "
            
            self.speech.say(next_action_prompt)
            next_action_response = self.speech.listen_and_get_text()
            next_action_llm = self.llm_manager.parse_next_action_decision(
                next_action_response, request_summary, any_request_fulfilled_overall, has_more_options
            )

            if next_action_llm.get("error"):
                self.speech.say("Unsure what to do next. Ending this search.")
                return {"status": "completed_recommendations_for_query"}

            intent_next = next_action_llm.get("intent")
            new_query = next_action_llm.get("new_query_text")

            if intent_next == "new_request" and new_query:
                self.speech.say(f"Okay, looking into: {new_query}")
                return {"status": "new_request", "detail": new_query}
            elif intent_next == "stop_interaction":
                self.speech.say("Alright.")
                return {"status": "stop_session" if any_request_fulfilled_overall else "completed_recommendations_for_query"}
            elif intent_next == "continue_current_request":
                if has_more_options: self.speech.say(f"Okay, next option for {request_summary}.")
                else:
                    self.speech.say(f"Actually, that was the last option for {request_summary}.")
                    return {"status": "completed_recommendations_for_query"}
            else:
                self.speech.say("Assuming we're done with that request.")
                return {"status": "completed_recommendations_for_query"}
            rclpy.spin_once(self, timeout_sec=0.05) # Allow ROS spin

        # Loop finished
        if not any_request_fulfilled_overall: self.speech.say(f"We've gone through all suggestions for {request_summary}, but didn't find it.")
        else: self.speech.say(f"We've explored options for {request_summary}.")
        return {"status": "completed_recommendations_for_query"}

    def process_intent_ros(self, intent_from_llm, parsed_llm_data, shopping_for_user_role, active_profile_for_recs):
        """ROS-aware intent processing."""
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
            if current_pose_tuple == (None, None, None):
                self.speech.say("Warning: My current location is unclear, so recommendations might not be distance-optimized.")
            
            kg_query = {
                "intent": intent_from_llm, "item_types": parsed_llm_data.get("item_types", []),
                "attributes": parsed_llm_data.get("attributes", {}), "store_name": parsed_llm_data.get("store_name")
            }
            query_results_kg = self.kg_service.execute_structured_query(kg_query)
            recs_data = self.recommender.generate_recommendations(
                parsed_llm_data, query_results_kg, active_profile_for_recs, current_pose_tuple
            )

            ranked_options = []
            if intent_from_llm == "find_store" and recs_data.get("stores"):
                for store_rec in recs_data["stores"]:
                    ranked_options.append({"store_info": store_rec, "product_count_if_any": None})
            elif intent_from_llm == "find_product" and recs_data.get("products_in_stores_ranked"):
                for prod_loc_rec in recs_data["products_in_stores_ranked"]:
                    ranked_options.append({
                        "store_info": prod_loc_rec["store_details"],
                        "product_count_if_any": len(prod_loc_rec.get("products_found", []))
                    })
            
            if not ranked_options:
                self.speech.say(f"Sorry, no suitable stores/products found for {self._generate_request_summary_for_dialogue(parsed_llm_data)}.")
                return {"status": "interaction_completed_for_current_query"}

            return self._handle_iterative_store_visits_ros(
                parsed_llm_data, ranked_options, shopping_for_user_role, active_profile_for_recs
            )

        self.speech.say(f"Unsure how to handle intent: '{intent_from_llm}'. Try rephrasing?")
        return {"status": "interaction_completed_for_current_query"}

    def run_shopping_session_loop(self):
        global current_speaker_role 
        self.speech.wait_for_wake_word() # Simulates waiting
        self.initialize_speaker_if_needed()

        if not current_speaker_role:
            self.get_logger().error("Speaker role not initialized. Cannot start session.")
            self.speech.say("Startup error: Speaker not identified.")
            return

        self.speech.say(f"Hello {current_speaker_role.capitalize()}! How can I help you or the family today? (Or say 'stop' to end.)")
        current_user_input = self.speech.listen_and_get_text()

        while rclpy.ok(): 
            if not current_user_input or current_user_input.strip().lower() == "stop":
                parsed_llm_output = {"intent": "stop_interaction"}
            else:
                parsed_llm_output = self.llm_manager.generate_structured_query(
                    current_user_input, current_speaker_role_if_known=current_speaker_role
                )
            
            self.get_logger().info(f"LLM Parsed Output: {json.dumps(parsed_llm_output, indent=2)}")

            if not parsed_llm_output or parsed_llm_output.get("error"):
                self.speech.say("Sorry, I had trouble understanding that. Please try rephrasing.")
                current_user_input = self.speech.listen_and_get_text()
                if current_user_input.strip().lower() == "stop": parsed_llm_output = {"intent": "stop_interaction"}
                else: rclpy.spin_once(self, timeout_sec=0.05); continue
            
            intent = parsed_llm_output.get("intent")
            if intent == "stop_interaction":
                self.speech.say("Alright, ending our shopping trip. Have a great day!"); break 

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
                    else: # Clarification failed
                        self.speech.say("Okay, what would you like to do then?")
                        current_user_input = self.speech.listen_and_get_text()
                        rclpy.spin_once(self, timeout_sec=0.05); continue
                else: shopping_for_user_this_query = current_speaker_role 

            self.speech.say(f"Working on this for {shopping_for_user_this_query.capitalize()}.")

            if parsed_llm_output.get("updated_profile_for_shopping_user"):
                self.profile_manager.update_basic_preferences(
                    shopping_for_user_this_query, parsed_llm_output["updated_profile_for_shopping_user"]
                ) # Verbose speech for this is optional

            active_profile = self.profile_manager.get_profile(shopping_for_user_this_query)
            
            session_status = self.process_intent_ros(intent, parsed_llm_output, shopping_for_user_this_query, active_profile)
            
            self.get_logger().info(f"Main loop received session status: {session_status}")
            self.get_logger().info("-" * 60) # Console separator

            if session_status.get("status") == "new_request":
                current_user_input = session_status.get("detail", "")
            elif session_status.get("status") == "stop_session":
                break # Iterative handler decided to stop session
            else: # completed_recommendations, interaction_completed, or other
                self.speech.say(f"Anything else for you or the family, {current_speaker_role.capitalize()}? (Or 'stop')")
                current_user_input = self.speech.listen_and_get_text()
            rclpy.spin_once(self, timeout_sec=0.05) # Allow ROS spin
            
        self.get_logger().info("Shopping session loop ended.")
        self.profile_manager._save_profiles() # Save profiles at end of session

def main(args=None):
    rclpy.init(args=args)
    # Ensure the preferences file is cleared for a fresh test run if desired
    prefs_file_path = "family_preferences_main.json"
    if os.path.exists(prefs_file_path):
        try:
            os.remove(prefs_file_path)
            print(f"Removed old preferences file: {prefs_file_path} for fresh test.")
        except OSError as e:
            print(f"Warning: Could not remove {prefs_file_path}: {e}")

    main_controller_node = MainRobotControllerNode() # This will re-create/load profile_manager
    
    # Test script for the dialogue flow
    test_script_for_dialogue_ros = [
        "I am the Mother.",
        "I'm looking for a toy for my child.", "Yes, let's go.", "Okay, I'm ready now.", "No, not quite the right one this time.", "It was a bit small, but the staff tried hard to help.", "Are there any other toy stores available?",
        "Sure, let's try that one.", "I've had a good look around.", "Yes! We found a fantastic building block set here. Perfect!", "They had a wonderful selection and it was very well organized. Much better.",
        "Now, I'd like to find a red dress for myself.", "No, let's skip Chic Boutique for now.", "Yes, let's check Urban Stylez.", "Finished looking here.", "Unfortunately no, they didn't have it in my size, only very small ones.", "The store was nice and bright though, good layout.",
        "That's all for today, thank you very much.",
        "stop" 
    ]
    speech_interface_global.set_script(test_script_for_dialogue_ros)
    
    global current_speaker_role 
    current_speaker_role = None # Reset for initialization

    try:
        main_controller_node.run_shopping_session_loop()
    except KeyboardInterrupt:
        main_controller_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    except Exception as e:
        main_controller_node.get_logger().error(f"Unhandled exception in main_controller_node: {e}", exc_info=True)
    finally:
        main_controller_node.get_logger().info("Shutting down MainRobotControllerNode...")
        # speech_interface_global.shutdown() # If SpeechInterface has cleanup
        
        if main_controller_node and rclpy.ok():
             # Explicitly stop the TF listener's thread if it was started with spin_thread=True
            if hasattr(main_controller_node, 'tf_listener') and main_controller_node.tf_listener._thread is not None:
                 main_controller_node.tf_listener._stop_thread() # Accessing protected member, but common for cleanup
            main_controller_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown process completed.")

    print("\n" + "="*20 + " Final User Profiles After ROS Session " + "="*20)
    # Load profiles again to show what was saved to disk
    final_profile_manager_checker = UserProfileManager(users_file=prefs_file_path) 
    for role_key_check in VALID_USER_KEYS:
        final_profile_manager_checker.display_profile(role_key_check)
    print("="*60)
    print("Session fully complete.")

if __name__ == "__main__":
    main()