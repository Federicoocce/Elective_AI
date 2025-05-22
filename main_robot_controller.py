#!/usr/bin/env python3
# main_robot_controller.py

import os # For checking/deleting test preference file
import json # For printing dicts if needed

# --- Import Application Modules ---
from llm_groq_parser import GroqQueryParser
from speech_interface import SpeechInterface
from user_manager import UserProfileManager, VALID_USER_KEYS
from mall_query_engine import KnowledgeGraphService
from recommendation_engine import RecommenderEngine
from navigation_manager import NavigationManager
# from graph_data_generator import fashion_mnist_classes, fictional_brands_list, colors_list # Already used by KGS

# --- Global State (Session Level) ---
current_speaker_role = None # Role of the person currently interacting ("mother", "father", "child")

# --- Component Initialization ---
# These are now instances of the actual classes
speech = SpeechInterface()
profile_manager = UserProfileManager(users_file="family_preferences_main.json") # Use a distinct file
kg_service = KnowledgeGraphService()
# Pass dependencies to RecommenderEngine
recommender = RecommenderEngine(knowledge_graph_service=kg_service, user_profile_manager=profile_manager)
nav_manager = NavigationManager(use_real_robot=False) # Set to True to attempt real robot connection

# Initialize LLM Parser and provide it with knowledge from KG
llm_manager = GroqQueryParser()
llm_manager.set_knowledge_base_lists(
    store_names=kg_service.get_all_store_names(),
    item_categories=kg_service.get_all_item_categories(),
    brands=kg_service.get_all_brands(),
    colors=kg_service.get_all_colors()
)


def initialize_speaker_if_needed():
    """
    Identifies the primary speaker for the session by repeatedly asking until a valid role is recognized.
    """
    global current_speaker_role
    if current_speaker_role: # If already set (e.g., from a previous successful call in a longer session)
        return

    speech.say("Before we begin, who will I be primarily talking with today? Please say Mother, Father, or Child.")
    
    max_attempts = 3 # Limit the number of retries to avoid an infinite loop
    attempts = 0

    while attempts < max_attempts:
        response = speech.listen_and_get_text().lower()
        print(f"[DEBUG] Speaker clarification attempt {attempts+1}: User said '{response}'") # For debugging

        if "father" in response:
            current_speaker_role = "father"
            break # Exit loop once a valid role is found
        elif "child" in response:
            current_speaker_role = "child"
            break
        elif "mother" in response or "mum" in response:
            current_speaker_role = "mother"
            break
        else:
            attempts += 1
            if attempts < max_attempts:
                speech.say("I'm sorry, I didn't quite understand. Could you please say if you are the Mother, Father, or Child?")
            else:
                # After max attempts, fall back to a default or handle the error
                current_speaker_role = "mother" # Default after exhausting attempts
                speech.say(f"I'm still not sure. For now, I'll assume I'm speaking with Mother. We can clarify later if needed.")
                print(f"[SESSION INFO] Could not clearly identify speaker after {max_attempts} attempts. Defaulting to 'mother'.")
                break # Exit loop

    speech.say(f"Okay, {current_speaker_role.capitalize()}, I'm ready when you are.")
    print(f"[SESSION INFO] Primary speaker for this session has been set to: {current_speaker_role.capitalize()}")



def run_shopping_session():
    global current_speaker_role

    speech.wait_for_wake_word()
    initialize_speaker_if_needed()

    while True:
        shopping_for_user_role = None
        request_fulfilled_this_round = False

        if request_fulfilled_this_round or shopping_for_user_role is None:
             speech.say(f"Okay, {current_speaker_role.capitalize()}, what can I do for you or the family next? (Or say 'stop')")
        else:
            speech.say(f"Sorry about the last one. What would you like to try now, {current_speaker_role.capitalize()}? (Or say 'stop')")

        user_raw_request = speech.listen_and_get_text()
        

        profile_context_for_llm = None


        parsed_data = llm_manager.generate_structured_query(
            user_raw_request,
            current_speaker_role_if_known=current_speaker_role,
            shopping_for_user_profile_context=profile_context_for_llm
        )

        if not parsed_data or parsed_data.get("error") or parsed_data.get("intent") == "stop_interaction":
            if parsed_data and parsed_data.get("error"):
                speech.say(f"I encountered an issue processing that: {parsed_data.get('details', 'Unknown error')}")
            speech.say("Alright, ending our shopping trip. Have a great day!")
            break

        llm_identified_target = parsed_data.get("shopping_for_user")

        if llm_identified_target == "self":
            shopping_for_user_role = current_speaker_role
            speech.say(f"Understood, {current_speaker_role.capitalize()}, this request is for you.")
        elif llm_identified_target in VALID_USER_KEYS:
            shopping_for_user_role = llm_identified_target
            speech.say(f"Got it, this request is for {shopping_for_user_role.capitalize()}.")
        else:
            speech.say(f"Okay. And for whom in the family is this particular request? (Mother, Father, or Child)")
            clarification_text = speech.listen_and_get_text().lower()
            if "mother" in clarification_text or "mum" in clarification_text : shopping_for_user_role = "mother"
            elif "father" in clarification_text or "dad" in clarification_text: shopping_for_user_role = "father"
            elif "child" in clarification_text or "kid" in clarification_text: shopping_for_user_role = "child"
            else:
                speech.say(f"I'm not sure about that. Let's assume it's for {current_speaker_role.capitalize()} this time.")
                shopping_for_user_role = current_speaker_role
            speech.say(f"Okay, focusing on {shopping_for_user_role.capitalize()} for this task.")

        if parsed_data.get("updated_profile_for_shopping_user"):
            updates = parsed_data["updated_profile_for_shopping_user"]
            if updates:
                profile_manager.update_basic_preferences(shopping_for_user_role, updates)
                speech.say(f"I've noted new preferences for {shopping_for_user_role.capitalize()}.")

        intent = parsed_data.get("intent")
        active_profile_for_task = profile_manager.get_profile(shopping_for_user_role)

        if intent == "show_profile":
            speech.say(f"Here are the current preferences for {shopping_for_user_role.capitalize()}:")
            profile_manager.display_profile(shopping_for_user_role)
            # speak_profile_details(active_profile_for_task) # To be implemented
            request_fulfilled_this_round = True

        elif intent == "update_profile_only":
            speech.say(f"Preferences for {shopping_for_user_role.capitalize()} have been noted/updated.")
            profile_manager.display_profile(shopping_for_user_role)
            request_fulfilled_this_round = True

        elif intent == "find_product" or intent == "find_store":
            print(f"[CONTROLLER] Action: '{intent}' for user '{shopping_for_user_role.capitalize()}'")
            kg_query_params = {
                "intent": intent, "item_types": parsed_data.get("item_types", []),
                "attributes": parsed_data.get("attributes", {}), "store_name": parsed_data.get("store_name")
            }
            query_results_from_kg = kg_service.execute_structured_query(kg_query_params)
            
            current_robot_pose = nav_manager.get_current_pose()
            recommended_options = recommender.generate_recommendations(
                parsed_data, query_results_from_kg, active_profile_for_task, current_robot_pose
            )

            selected_shop_to_visit_data = None # This will be the dict of the selected store
            recommendation_made = False

            if intent == "find_store" and recommended_options.get("stores"):
                if recommended_options["stores"]:
                    selected_shop_to_visit_data = recommended_options["stores"][0] # Simplistic: take first
                    speech.say(f"I recommend {selected_shop_to_visit_data['name']} ({selected_shop_to_visit_data['type']}). Would you like to go there?")
                    recommendation_made = True
            elif intent == "find_product" and recommended_options.get("products_in_stores_ranked"):
                if recommended_options["products_in_stores_ranked"]:
                    top_choice_store_info = recommended_options["products_in_stores_ranked"][0]
                    selected_shop_to_visit_data = top_choice_store_info["store_details"]
                    num_items = len(top_choice_store_info["products_found"])
                    speech.say(f"I found {num_items} matching item(s) at {selected_shop_to_visit_data['name']}. Shall we head there?")
                    recommendation_made = True
            
            if recommendation_made and selected_shop_to_visit_data:
                user_nav_confirmation = speech.listen_and_get_text().lower()
                if "yes" in user_nav_confirmation or "sure" in user_nav_confirmation or "okay" in user_nav_confirmation:
                    target_coords = (selected_shop_to_visit_data['map_x'], selected_shop_to_visit_data['map_y'], selected_shop_to_visit_data['map_theta'])
                    nav_success = nav_manager.navigate_to_goal(*target_coords)

                    if nav_success:
                        speech.say(f"We've arrived at {selected_shop_to_visit_data['name']}.")
                        speech.say(f"Did {shopping_for_user_role.capitalize()} find what they were looking for here?")
                        fulfillment_response = speech.listen_and_get_text().lower()
                        
                        if "yes" in fulfillment_response:
                            request_fulfilled_this_round = True; speech.say("Excellent!")
                        else:
                            request_fulfilled_this_round = False; speech.say("Oh, okay. Good to know.")

                        speech.say("Any other feedback about this shop or the items for my notes?")
                        feedback_text = speech.listen_and_get_text()
                        if feedback_text and "no" not in feedback_text.lower() and "nothing" not in feedback_text.lower():
                            # Use LLM to parse this feedback
                            parsed_feedback_for_profile = llm_manager.parse_feedback_to_profile_update(
                                feedback_text,
                                selected_shop_to_visit_data['name'],
                                parsed_data.get("item_types", []) # Original item types sought
                            )
                            if parsed_feedback_for_profile and not parsed_feedback_for_profile.get("error"):
                                profile_manager.update_profile_with_feedback(
                                    shopping_for_user_role,
                                    selected_shop_to_visit_data['name'],
                                    parsed_feedback_for_profile
                                )
                                speech.say(f"Thanks! I've updated the notes for {shopping_for_user_role.capitalize()}.")
                            elif parsed_feedback_for_profile.get("error"):
                                speech.say("I had a bit of trouble processing that feedback, but thank you.")
                    else: # Navigation failed
                        speech.say(f"I'm sorry, I had trouble getting to {selected_shop_to_visit_data['name']}.")
                        request_fulfilled_this_round = False
                else: # User did not confirm navigation
                    speech.say("Okay, no problem. We won't go there then.")
                    request_fulfilled_this_round = False
            else: # No shop recommended or found from recommender
                speech.say(f"Sorry, I couldn't find a specific place for that request right now.")
                request_fulfilled_this_round = False
        else: # Unknown intent
            speech.say("I'm not quite sure how to handle that specific request type yet.")
            request_fulfilled_this_round = False
        
        print("-" * 25 + f" End of Request Round for {shopping_for_user_role.upper()} " + "-"*25 + "\n")

    # Cleanup after session loop ends
    nav_manager.shutdown()
    speech.shutdown()
    print("Shopping session controller finished.")


if __name__ == "__main__":
    # Clean up old preference file for a fresh test run
    PREFS_FILE = "family_preferences_main.json"
    if os.path.exists(PREFS_FILE):
        os.remove(PREFS_FILE)
    # Re-initialize profile_manager for the test run, as it might have been
    # instantiated globally before the file was deleted.
    profile_manager = UserProfileManager(users_file=PREFS_FILE)


    # Define a script of user inputs for the mock speech interface
    # This script will test the speaker initialization loop
    interaction_script_for_speaker_init_test = [
        # Initialize speaker (first attempt unclear, second unclear, third valid)
        "someone",      # User says something unclear
        "the adult",    # User says something still unclear
        "I am the Father", # User finally clarifies

        # Continue with a simple shopping round
        "I'm looking for a toy for the child",
        "yes",
        "yes, they found it",
        "it's a great toy",
        "stop"
    ]
    speech.set_script(interaction_script_for_speaker_init_test) # Use the new script

    print("--- Starting Main Robot Controller Session (with speaker init loop test) ---")
    try:
        # Reset global current_speaker_role before each test run if running __main__ multiple times
        current_speaker_role = None
        run_shopping_session()
    except Exception as e:
        print(f"An error occurred during the session: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("--- Main Robot Controller Session Ended ---")

        print("\n--- Final User Profiles (from family_preferences_main.json) ---")
        final_profile_manager = UserProfileManager(users_file=PREFS_FILE) # Load fresh from file
        if current_speaker_role: # Check if speaker was set
            final_profile_manager.display_profile(current_speaker_role)
        final_profile_manager.display_profile("mother") # Display all expected profiles
        final_profile_manager.display_profile("father")
        final_profile_manager.display_profile("child")