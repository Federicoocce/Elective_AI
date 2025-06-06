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
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# --- Import Application Modules ---
from .llm_groq_parser import GroqQueryParser
from .speech_interface import SpeechInterface
from .user_manager import UserProfileManager, VALID_USER_KEYS
from .mall_query_engine import KnowledgeGraphService
from .recommendation_engine import RecommenderEngine

# --- Global State (outside class for script compatibility) ---
current_speaker_role = None 
speech_interface_global = SpeechInterface() 

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

        # --- ROS2 Navigation Action Client ---
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("NavigateToPose action client created.")
        self._check_nav_server_ready() 

        # --- TF Buffer and Listener for Robot Pose ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.get_logger().info("TF Listener initialized for robot pose.")
        # Optional: Short delay to allow TF buffer to populate if accessed very early
        # time.sleep(1.0) 

        self.get_logger().info("MainRobotControllerNode initialized successfully.")

    def _check_nav_server_ready(self, timeout_sec=5.0):
        if not self._nav_action_client.server_is_ready():
            self.get_logger().warn("NavigateToPose action server not available. Waiting...")
            if not self._nav_action_client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error("NavigateToPose action server not available after timeout. Navigation will fail.")
                return False
            self.get_logger().info("NavigateToPose action server is ready.")
        return True

    def send_navigation_goal(self, x, y, theta_degrees):
        self.get_logger().info(f"Attempting to navigate to x={x:.2f}, y={y:.2f}, theta={theta_degrees:.2f} deg.")
        if not self._check_nav_server_ready(timeout_sec=1.0):
             return False

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        
        theta_radians = math.radians(float(theta_degrees))
        orientation_q = _yaw_to_quaternion(theta_radians)
        goal_pose.pose.orientation = orientation_q
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal to Nav2: Pos({x:.2f}, {y:.2f}), Ori Degrees({theta_degrees:.2f})")
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.1) 

        if not send_goal_future.done() or send_goal_future.exception():
            exc = send_goal_future.exception()
            self.get_logger().error(f"Goal sending failed or rejected: {exc if exc else 'Unknown error'}")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()

        while rclpy.ok() and not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1) 

        if not get_result_future.done() or get_result_future.exception():
            exc = get_result_future.exception()
            self.get_logger().error(f"Error getting goal result: {exc if exc else 'Unknown error'}")
            return False

        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Navigation goal failed with status: {status}')
            return False

    def get_current_robot_pose(self):
        """Returns current robot pose as (x, y, theta_radians) with retries"""
        for _ in range(5):  # Add retry mechanism
            try:
                # Wait for transform to become available
                self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                
                transform_stamped = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(seconds=0)  # Use latest available transform
                )
                x = transform_stamped.transform.translation.x
                y = transform_stamped.transform.translation.y
                q = transform_stamped.transform.rotation
                yaw_radians = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return x, y, yaw_radians
                
            except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().warn(f'TF unavailable, retrying: {ex}')
                time.sleep(0.5)  # Brief delay between retries
        
        self.get_logger().error('Failed to get pose after 5 attempts')
        return None, None, None


    def identify_role_from_text(self, text):
        mapping = { "father": ["father", "dad"], "mother": ["mother", "mum"], "child": ["child", "kid"] }
        for role, keywords in mapping.items():
            if any(word in text for word in keywords): return role
        return None

    def initialize_speaker_if_needed(self):
        global current_speaker_role
        if current_speaker_role: return

        self.speech.say("Before we begin, who will I be primarily talking with today? Please say Mother, Father, or Child.")
        for _ in range(3):
            role_text = self.speech.listen_and_get_text().lower()
            self.get_logger().info(f"Speaker identification input: {role_text}")
            role = self.identify_role_from_text(role_text)
            if role: current_speaker_role = role; break
            self.speech.say("I'm sorry, I didn't understand. Could you please say if you are the Mother, Father, or Child?")
        if not current_speaker_role:
            current_speaker_role = "mother"
            self.speech.say("I'm not sure. I will assume I'm speaking with Mother.")
        self.speech.say(f"Okay, {current_speaker_role.capitalize()}, I'm ready when you are.")

    def get_confirmation(self, text):
        affirmatives = ["yes", "sure", "okay", "ok", "yep", "please", "sounds good", "do it"]
        return any(word in text.lower() for word in affirmatives)

    def clarify_shopping_target(self):
        global current_speaker_role
        self.speech.say("And who is this request for? (Mother, Father, or Child)")
        for _ in range(3):
            response = self.speech.listen_and_get_text().lower()
            target_role = self.identify_role_from_text(response)
            if target_role: return target_role
            self.speech.say("Sorry, I didn't catch that. Is it for Mother, Father, or Child?")
        self.speech.say(f"I'll assume it's for {current_speaker_role.capitalize()}.")
        return current_speaker_role

    def process_intent(self, intent, parsed_data, shopping_for_user_role, active_profile):
        if intent == "show_profile":
            self.speech.say(f"Here are the preferences for {shopping_for_user_role.capitalize()}:")
            self.profile_manager.display_profile(shopping_for_user_role)
            return True

        if intent == "update_profile_only":
            self.profile_manager.display_profile(shopping_for_user_role)
            self.speech.say(f"Preferences updated for {shopping_for_user_role.capitalize()}.")
            return True

        if intent in {"find_product", "find_store"}:
            current_robot_pose = self.get_current_robot_pose() 
            if current_robot_pose == (None, None, None):
                self.speech.say("I'm having a bit of trouble finding my exact spot. Recommendations might not be distance-optimized.")
            
            kg_query = {
                "intent": intent, "item_types": parsed_data.get("item_types", []),
                "attributes": parsed_data.get("attributes", {}), "store_name": parsed_data.get("store_name")
            }
            query_results = self.kg_service.execute_structured_query(kg_query)
            
            recommendations = self.recommender.generate_recommendations(
                parsed_data, query_results, active_profile, current_robot_pose
            )

            store_to_navigate = None
            navigation_prompt = ""

            if intent == "find_store" and recommendations.get("stores"):
                if recommendations["stores"]:
                    store_to_navigate = recommendations["stores"][0]
                    dist_str = f" (Distance: {store_to_navigate['distance']:.1f}m)" if store_to_navigate.get('distance', float('inf')) < float('inf') else ""
                    navigation_prompt = f"I recommend {store_to_navigate['name']}{dist_str}. Shall we go there?"
                else: self.speech.say("Sorry, I couldn't find any stores matching your request."); return False
            
            elif intent == "find_product" and recommendations.get("products_in_stores_ranked"):
                if recommendations["products_in_stores_ranked"]:
                    top_option = recommendations["products_in_stores_ranked"][0]
                    store_to_navigate = top_option["store_details"]
                    count = len(top_option["products_found"])
                    dist_str = f" (Distance: {top_option['distance']:.1f}m)" if top_option.get('distance', float('inf')) < float('inf') else ""
                    navigation_prompt = f"I found {count} item(s) at {store_to_navigate['name']}{dist_str}. Shall we go there?"
                else: self.speech.say("Sorry, I couldn't find that product in any store right now."); return False
            
            if store_to_navigate and navigation_prompt:
                self.speech.say(navigation_prompt)
                confirmation_text = self.speech.listen_and_get_text()
                if self.get_confirmation(confirmation_text):
                    self.speech.say(f"Okay, navigating to {store_to_navigate['name']}.")
                    nav_x, nav_y, nav_theta = store_to_navigate.get('map_x'), store_to_navigate.get('map_y'), store_to_navigate.get('map_theta')

                    if nav_x is None or nav_y is None or nav_theta is None:
                        self.speech.say(f"Sorry, I don't have map coordinates for {store_to_navigate['name']}."); return False

                    if self.send_navigation_goal(nav_x, nav_y, nav_theta):
                        self.speech.say(f"We have arrived at {store_to_navigate['name']}.")
                        self.speech.say("Did you find what you were looking for?")
                        fulfilled_text = self.speech.listen_and_get_text()
                        
                        self.speech.say("Thanks! Any other comments or details?")
                        feedback_text = self.speech.listen_and_get_text()
                        
                        if feedback_text and feedback_text.strip().lower() not in ["no", "none", "nothing", "not really"]:
                            combined_feedback = f"Fulfilled: {self.get_confirmation(fulfilled_text)}. User comments: {feedback_text}"
                            llm_parsed_feedback = self.llm_manager.parse_feedback_to_profile_update(
                                combined_feedback, store_to_navigate['name'], parsed_data.get("item_types", [])
                            )
                            if llm_parsed_feedback and not llm_parsed_feedback.get("error"):
                                self.profile_manager.update_profile_with_feedback(
                                    shopping_for_user_role, store_to_navigate['name'], llm_parsed_feedback
                                )
                                self.speech.say(f"Great, I've updated the profile for {shopping_for_user_role.capitalize()}.")
                            else: self.speech.say("Thanks for the feedback. I couldn't structure it this time.")
                        else:
                            simple_feedback_update = {
                                "shop_review_update": {"request_fulfilled": self.get_confirmation(fulfilled_text)},
                                "item_feedback_updates": []}
                            self.profile_manager.update_profile_with_feedback(
                                shopping_for_user_role, store_to_navigate['name'], simple_feedback_update)
                            self.speech.say("Okay, noted.")
                        return True
                    else: self.speech.say(f"Sorry, I encountered a problem trying to navigate to {store_to_navigate['name']}."); return False
                else: self.speech.say("Okay, we won't go there now."); return True
            else: self.speech.say("Sorry, I couldn't find a suitable store with navigation options."); return False

        self.speech.say("I'm not sure how to handle that request."); return False

    def run_shopping_session_loop(self):
        global current_speaker_role 
        self.speech.wait_for_wake_word()
        self.initialize_speaker_if_needed()

        while rclpy.ok(): 
            self.speech.say(f"Okay, {current_speaker_role.capitalize()}, what can I do for you next? (Or say 'stop')")
            user_input = self.speech.listen_and_get_text()

            if not user_input:
                self.speech.say("Sorry, I didn't hear anything."); rclpy.spin_once(self, timeout_sec=0.1); continue

            parsed_data = self.llm_manager.generate_structured_query(
                user_input, current_speaker_role_if_known=current_speaker_role
            )
            self.get_logger().info(f"LLM Parsed Data: {json.dumps(parsed_data, indent=2)}")

            if not parsed_data or parsed_data.get("error"):
                self.speech.say("Sorry, I had trouble understanding that."); rclpy.spin_once(self, timeout_sec=0.1); continue
            
            intent = parsed_data.get("intent")
            if intent == "stop_interaction":
                self.speech.say("Alright, ending our shopping trip. Have a great day!"); break 

            shopping_for_llm = parsed_data.get("shopping_for_user")
            if shopping_for_llm == "self": shopping_for_user_role = current_speaker_role
            elif shopping_for_llm in VALID_USER_KEYS: shopping_for_user_role = shopping_for_llm
            else:
                if intent in ["find_product", "find_store", "show_profile", "update_profile_only"]:
                    shopping_for_user_role = self.clarify_shopping_target()
                else: shopping_for_user_role = current_speaker_role 

            self.speech.say(f"Working on this for {shopping_for_user_role.capitalize()}.")

            if parsed_data.get("updated_profile_for_shopping_user"):
                self.profile_manager.update_basic_preferences(
                    shopping_for_user_role, parsed_data["updated_profile_for_shopping_user"]
                )
                self.speech.say(f"Noted new preferences for {shopping_for_user_role.capitalize()}.")

            active_profile = self.profile_manager.get_profile(shopping_for_user_role)
            self.process_intent(intent, parsed_data, shopping_for_user_role, active_profile)
            self.get_logger().info("-" * 60)
            rclpy.spin_once(self, timeout_sec=0.1) 
        self.get_logger().info("Shopping session loop ended.")


def main(args=None):
    rclpy.init(args=args)
    main_controller_node = MainRobotControllerNode()
    
    # Example script for testing
    speech_interface_global.set_script([
        "father", 
        "I'm looking for Kids Playworld for my child", 
        "yes", 
        "yes, we found a great toy!", 
        "The store was fantastic, lots of options.", 
        "stop"
    ])
    global current_speaker_role 
    current_speaker_role = None 

    try:
        main_controller_node.run_shopping_session_loop()
    except KeyboardInterrupt:
        main_controller_node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        import traceback
        error_msg = f"Unhandled exception in main: {e}\n{traceback.format_exc()}"
        main_controller_node.get_logger().error(error_msg)
    finally:
        main_controller_node.get_logger().info("Shutting down MainRobotControllerNode.")
        # SpeechInterface shutdown might not be strictly necessary if it has no resources to release
        # speech_interface_global.shutdown() 
        
        # Destroy the TF listener by ensuring node destruction.
        # The TransformListener's background thread should be stopped when the node is destroyed.
        if main_controller_node and rclpy.ok(): # Ensure node exists and rclpy is still up
            main_controller_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown complete.")

    print("\n--- Final User Profiles ---")
    final_profile_manager = UserProfileManager(users_file="family_preferences_main.json")
    for role_key in VALID_USER_KEYS:
        final_profile_manager.display_profile(role_key)
    print("Session complete.")

if __name__ == "__main__":
    main()