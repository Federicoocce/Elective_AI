#!/usr/bin/env python3
# main_robot_controller.py

import os
import json

# --- Import Application Modules ---
from llm_groq_parser import GroqQueryParser
from speech_interface import SpeechInterface
from user_manager import UserProfileManager, VALID_USER_KEYS
from mall_query_engine import KnowledgeGraphService
from recommendation_engine import RecommenderEngine
from navigation_manager import NavigationManager

# --- Global State ---
current_speaker_role = None

# --- Component Initialization ---
speech = SpeechInterface()
profile_manager = UserProfileManager(users_file="family_preferences_main.json")
kg_service = KnowledgeGraphService()
recommender = RecommenderEngine(knowledge_graph_service=kg_service, user_profile_manager=profile_manager)
nav_manager = NavigationManager(use_real_robot=False)
llm_manager = GroqQueryParser()

llm_manager.set_knowledge_base_lists(
    store_names=kg_service.get_all_store_names(),
    item_categories=kg_service.get_all_item_categories(),
    brands=kg_service.get_all_brands(),
    colors=kg_service.get_all_colors()
)

def identify_role_from_text(text):
    mapping = {
        "father": ["father", "dad"],
        "mother": ["mother", "mum"],
        "child": ["child", "kid"]
    }
    for role, keywords in mapping.items():
        if any(word in text for word in keywords):
            return role
    return None

def initialize_speaker_if_needed():
    global current_speaker_role
    if current_speaker_role:
        return

    speech.say("Before we begin, who will I be primarily talking with today? Please say Mother, Father, or Child.")
    for _ in range(3):
        role_text = speech.listen_and_get_text().lower()
        print(f"[DEBUG] Speaker said: {role_text}")
        role = identify_role_from_text(role_text)
        if role:
            current_speaker_role = role
            break
        speech.say("I'm sorry, I didn't understand. Could you please say if you are the Mother, Father, or Child?")

    if not current_speaker_role:
        current_speaker_role = "mother"
        speech.say("I'm not sure. I will assume I'm speaking with Mother.")

    speech.say(f"Okay, {current_speaker_role.capitalize()}, I'm ready when you are.")

def get_confirmation(text):
    affirmatives = ["yes", "sure", "okay", "ok", "yep"]
    return any(word in text for word in affirmatives)

def clarify_shopping_target():
    speech.say("And who is this request for? (Mother, Father, or Child)")
    response = speech.listen_and_get_text().lower()
    return identify_role_from_text(response) or current_speaker_role

def process_intent(intent, parsed_data, shopping_for_user_role, active_profile):
    if intent == "show_profile":
        speech.say(f"Here are the preferences for {shopping_for_user_role.capitalize()}:")
        profile_manager.display_profile(shopping_for_user_role)
        return True

    if intent == "update_profile_only":
        profile_manager.display_profile(shopping_for_user_role)
        speech.say(f"Preferences updated for {shopping_for_user_role.capitalize()}.")
        return True

    if intent in {"find_product", "find_store"}:
        kg_query = {
            "intent": intent,
            "item_types": parsed_data.get("item_types", []),
            "attributes": parsed_data.get("attributes", {}),
            "store_name": parsed_data.get("store_name")
        }
        query_results = kg_service.execute_structured_query(kg_query)
        pose = nav_manager.get_current_pose()
        recommendations = recommender.generate_recommendations(parsed_data, query_results, active_profile, pose)

        store_info = None
        if intent == "find_store" and recommendations.get("stores"):
            store_info = recommendations["stores"][0]
            speech.say(f"I recommend {store_info['name']}. Shall we go there?")
        elif intent == "find_product" and recommendations.get("products_in_stores_ranked"):
            product_info = recommendations["products_in_stores_ranked"][0]
            store_info = product_info["store_details"]
            count = len(product_info["products_found"])
            speech.say(f"I found {count} item(s) at {store_info['name']}. Shall we go there?")

        if store_info:
            confirmation = speech.listen_and_get_text().lower()
            if get_confirmation(confirmation):
                coords = (store_info['map_x'], store_info['map_y'], store_info['map_theta'])
                if nav_manager.navigate_to_goal(*coords):
                    speech.say(f"Arrived at {store_info['name']}.")
                    fulfilled = speech.listen_and_get_text().lower()
                    if get_confirmation(fulfilled):
                        speech.say("Excellent!")
                        feedback = speech.listen_and_get_text()
                        if feedback.strip() and not any(x in feedback.lower() for x in ["no", "nothing"]):
                            updates = llm_manager.parse_feedback_to_profile_update(feedback, store_info['name'], parsed_data.get("item_types", []))
                            if updates and not updates.get("error"):
                                profile_manager.update_profile_with_feedback(shopping_for_user_role, store_info['name'], updates)
                                speech.say(f"Thanks! Updated notes for {shopping_for_user_role.capitalize()}.")
                        return True
                    else:
                        speech.say("Thanks for the feedback.")
                        return False
            else:
                speech.say("Okay, we won't go there now.")
        else:
            speech.say("Sorry, I couldn't find a suitable store right now.")
        return False

    speech.say("I'm not sure how to handle that request.")
    return False

def run_shopping_session():
    global current_speaker_role

    speech.wait_for_wake_word()
    initialize_speaker_if_needed()

    while True:
        speech.say(f"Okay, {current_speaker_role.capitalize()}, what can I do for you or the family next? (Or say 'stop')")
        user_input = speech.listen_and_get_text()

        parsed_data = llm_manager.generate_structured_query(
            user_input,
            current_speaker_role_if_known=current_speaker_role
        )

        if not parsed_data or parsed_data.get("intent") == "stop_interaction":
            speech.say("Alright, ending our shopping trip. Have a great day!")
            break

        shopping_for = parsed_data.get("shopping_for_user")
        shopping_for_user_role = (
            current_speaker_role if shopping_for == "self"
            else shopping_for if shopping_for in VALID_USER_KEYS
            else clarify_shopping_target()
        )
        speech.say(f"Working on this for {shopping_for_user_role.capitalize()}.")

        if parsed_data.get("updated_profile_for_shopping_user"):
            profile_manager.update_basic_preferences(shopping_for_user_role, parsed_data["updated_profile_for_shopping_user"])
            speech.say(f"Updated preferences for {shopping_for_user_role.capitalize()}.")

        active_profile = profile_manager.get_profile(shopping_for_user_role)
        process_intent(parsed_data.get("intent"), parsed_data, shopping_for_user_role, active_profile)
        print("-" * 60)

    nav_manager.shutdown()
    speech.shutdown()
    print("Session complete.")

if __name__ == "__main__":
    PREFS_FILE = "family_preferences_main.json"
    if os.path.exists(PREFS_FILE):
        os.remove(PREFS_FILE)
    profile_manager = UserProfileManager(users_file=PREFS_FILE)
    speech.set_script([
        "someone", "the adult", "I am the Father",
        "I'm looking for a toy for the child", "yes", "yes, they found it", "it's a great toy", "stop"
    ])
    current_speaker_role = None
    run_shopping_session()

    final_profile_manager = UserProfileManager(users_file=PREFS_FILE)
    for role in ["mother", "father", "child"]:
        final_profile_manager.display_profile(role)
