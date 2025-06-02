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
current_speaker_role = None # This will be set at the start of the session

# --- Component Initialization ---
speech = SpeechInterface()
profile_manager = UserProfileManager(users_file="family_preferences_main.json")
kg_service = KnowledgeGraphService()
recommender = RecommenderEngine(knowledge_graph_service=kg_service, user_profile_manager=profile_manager)
nav_manager = NavigationManager(use_real_robot=False) # Set to True for real robot if robot_cmd_ros is available
llm_manager = GroqQueryParser()

# Provide LLM with knowledge about the mall's entities
llm_manager.set_knowledge_base_lists(
    store_names=kg_service.get_all_store_names(),
    item_categories=kg_service.get_all_item_categories(),
    brands=kg_service.get_all_brands(),
    colors=kg_service.get_all_colors()
)

def identify_role_from_text(text):
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

def initialize_speaker_if_needed():
    """
    Identifies the primary speaker at the beginning of the session.
    This speaker is assumed constant for the session unless a new wake word event
    (and subsequent re-identification) is implemented.
    """
    global current_speaker_role
    if current_speaker_role: # Already initialized
        return

    speech.say("Before we begin, who will I be primarily talking with today? Please say Mother, Father, or Child.")
    for _ in range(3): # Allow 3 attempts
        role_text = speech.listen_and_get_text()
        print(f"[DEBUG] Speaker ID attempt, user said: {role_text}")
        role = identify_role_from_text(role_text)
        if role:
            current_speaker_role = role
            speech.say(f"Thank you, {current_speaker_role.capitalize()}.")
            return
        speech.say("I'm sorry, I didn't quite catch that. Could you please say if you are the Mother, Father, or Child?")

    # Default if identification fails after attempts
    current_speaker_role = "mother"
    speech.say(f"I'm having a little trouble understanding. I'll assume I'm speaking with Mother for now. You can correct me later if needed.")

def get_confirmation(prompt_text, max_retries=2):
    """Asks a yes/no question and returns True for yes, False for no."""
    speech.say(prompt_text)
    for i in range(max_retries):
        response = speech.listen_and_get_text().lower()
        print(f"[DEBUG] Confirmation for '{prompt_text}', user said: '{response}'")
        if any(word in response for word in ["yes", "sure", "okay", "ok", "yep", "please", "do it", "let's go", "affirmative"]):
            return True
        if any(word in response for word in ["no", "nope", "don't", "skip", "cancel", "negative", "not now"]):
            return False
        if i < max_retries - 1:
            speech.say("Sorry, I didn't understand. Please answer with a yes or no.")
    speech.say("Okay, I'll take that as a no for now.") # Default to no if unclear after retries
    return False

def clarify_shopping_target():
    """
    Asks the speaker who the current shopping request is for.
    Returns one of VALID_USER_KEYS or defaults to current_speaker_role.
    """
    global current_speaker_role
    speech.say("And who is this request for? (Mother, Father, or Child)")
    for _ in range(2): # 2 attempts
        response = speech.listen_and_get_text()
        print(f"[DEBUG] Clarify target, user said: '{response}'")
        role = identify_role_from_text(response)
        if role:
            return role
        speech.say("Sorry, I didn't catch that clearly. Is it for Mother, Father, or Child?")
    
    speech.say(f"Okay, I'll assume this request is for {current_speaker_role.capitalize()}.")
    return current_speaker_role

def _generate_request_summary(parsed_llm_data):
    """Generates a brief human-readable summary of the user's request for use in prompts."""
    if not parsed_llm_data or not isinstance(parsed_llm_data, dict):
        return "your request"
    
    items = parsed_llm_data.get("item_types", [])
    attrs = parsed_llm_data.get("attributes", {})
    store = parsed_llm_data.get("store_name")

    desc_parts = []
    if isinstance(attrs, dict): # Ensure attrs is a dict
        if attrs.get("color"): desc_parts.append(attrs["color"])
        if attrs.get("brand"): desc_parts.append(attrs["brand"])
    
    item_str = ""
    if items and isinstance(items, list) and len(items) > 0:
        item_str = items[0] # Take the first item type for simplicity
        if len(items) > 1:
            item_str += " or similar items"
    
    if item_str:
        desc_parts.append(item_str)

    if store:
        if desc_parts:
            return f"finding {', '.join(desc_parts)} at {store}"
        return f"information about {store}"
    
    if desc_parts:
        return f"finding {' '.join(desc_parts)}"
    
    return "your general request"

def handle_iterative_store_visits(original_request_parsed_data, ranked_store_options_with_details, shopping_for_user_role, active_profile_for_recommendations):
    """
    Manages iterating through store recommendations, navigation, and post-visit interactions.
    Returns a dictionary indicating the next action for the main loop:
    e.g., {"status": "new_request", "detail": "user's new query text"}
          {"status": "stop_session"}
          {"status": "completed_recommendations_for_query"}
    """
    request_summary = _generate_request_summary(original_request_parsed_data)
    print(f"[DEBUG] Iterative handler for: {request_summary} (for {shopping_for_user_role})")

    if not ranked_store_options_with_details: # Should be caught before calling, but defensive
        speech.say(f"It seems I don't have any specific store suggestions for {request_summary} right now.")
        return {"status": "completed_recommendations_for_query"}

    any_request_fulfilled_overall = False # Was the core request fulfilled at ANY store during this iteration?

    for i, store_option_details in enumerate(ranked_store_options_with_details):
        store_info = store_option_details["store_info"] # This is the dict from recommender (name, map_x, etc.)
        product_count_if_any = store_option_details["product_count_if_any"]
        
        store_name = store_info.get('name', 'an unnamed store')
        proposal_intro = f"For {request_summary},"
        if product_count_if_any is not None:
            proposal = f"{proposal_intro} I found {product_count_if_any} matching item(s) at {store_name}."
        else:
            proposal = f"{proposal_intro} one potential place is {store_name}."
        
        is_last_option_in_list = (i == len(ranked_store_options_with_details) - 1)

        if not get_confirmation(f"{proposal} Shall we go there?"):
            speech.say(f"Okay, we'll skip {store_name} for now.")
            if is_last_option_in_list:
                speech.say(f"That was the last option I had for {request_summary}.")
            continue # To the next store option or end of loop if this was the last

        # User wants to go to this store
        map_x = store_info.get('map_x')
        map_y = store_info.get('map_y')
        map_theta = store_info.get('map_theta', 0.0) # Default theta to 0 if not present

        if map_x is None or map_y is None:
            speech.say(f"Sorry, I don't have precise location data to navigate to {store_name}.")
        elif nav_manager.navigate_to_goal(map_x, map_y, map_theta):
            speech.say(f"We've arrived at {store_name}. Please take your time to look around. Let me know when you're ready for feedback or to discuss next steps.")
            _ = speech.listen_and_get_text() # User says something like "okay", "I'm ready", "finished"
        else: # Navigation failed
            speech.say(f"Sorry, I encountered an issue trying to navigate to {store_name}.")
            if not is_last_option_in_list:
                if get_confirmation("Shall we try the next store on the list for this request?"):
                    continue
                else:
                    speech.say(f"Okay, we'll stop looking for {request_summary} for now.")
                    return {"status": "completed_recommendations_for_query"} # End this line of query
            else:
                speech.say(f"That was the last option for {request_summary}, and navigation failed.")
                return {"status": "completed_recommendations_for_query"} # End this line of query

        # --- Post-visit interaction at the current store ---
        current_item_fulfilled_at_this_store = False
        speech.say(f"So, regarding your search for {request_summary}, were you able to find what you were looking for here at {store_name}?")
        fulfillment_response_text = speech.listen_and_get_text()
        fulfillment_status_llm = llm_manager.parse_fulfillment_status(fulfillment_response_text, request_summary)
        
        if fulfillment_status_llm.get("error"):
            speech.say("I had a little trouble understanding if you found it. Let's assume not for this store.")
            current_item_fulfilled_at_this_store = False
        else:
            current_item_fulfilled_at_this_store = fulfillment_status_llm.get("fulfilled", False)

        if current_item_fulfilled_at_this_store:
            speech.say("That's great to hear!")
            any_request_fulfilled_overall = True # Mark original request as met (at least once)
        else:
            speech.say(f"Oh, I'm sorry to hear you couldn't find {request_summary} there.")

        # Get general store feedback
        speech.say(f"Do you have any other feedback about your experience at {store_name}? For example, about their selection, service, or the store itself? Or just say 'no feedback'.")
        feedback_text = speech.listen_and_get_text()
        if feedback_text.strip().lower() not in ["no feedback", "nothing", "skip", "no"]:
            profile_updates_llm = llm_manager.parse_feedback_to_profile_update(
                feedback_text, 
                store_name, 
                original_request_parsed_data.get("item_types", []) # Pass item types sought for context
            )
            if profile_updates_llm and not profile_updates_llm.get("error"):
                # Ensure the fulfillment status for *this specific store visit* is part of the shop_review_update
                if "shop_review_update" not in profile_updates_llm or profile_updates_llm["shop_review_update"] is None:
                    profile_updates_llm["shop_review_update"] = {}
                profile_updates_llm["shop_review_update"]["request_fulfilled"] = current_item_fulfilled_at_this_store
                
                profile_manager.update_profile_with_feedback(shopping_for_user_role, store_name, profile_updates_llm)
                speech.say(f"Thanks! I've updated the notes for {shopping_for_user_role.capitalize()} regarding {store_name}.")
            elif profile_updates_llm.get("error"):
                 speech.say("I had a bit of trouble parsing that feedback, but thanks for sharing.")
            else: # LLM returned empty or unexpected, but not an error structure
                speech.say("Thanks for the feedback.")
        else:
            speech.say("Okay, no problem.")

        # --- Decide what to do next (after this store visit) ---
        has_more_options_for_this_original_request = not is_last_option_in_list
        
        next_action_prompt = "What would you like to do next? "
        if any_request_fulfilled_overall: # If main goal was met (possibly at this store or a previous one)
            next_action_prompt += "We can look for something else entirely, or we can stop for now. "
            if has_more_options_for_this_original_request: # Still offer, but as a secondary option
                 next_action_prompt += f"Or, if you like, I can still show you other options for {request_summary}."
        else: # Main goal NOT met yet
            if has_more_options_for_this_original_request:
                next_action_prompt += f"I have other store suggestions for {request_summary}. Or, we can look for something new, or stop for now. "
            else: # No more options for this request
                next_action_prompt += f"That was the last option for {request_summary}. We can look for something new, or stop for now. "

        speech.say(next_action_prompt)
        next_action_response_text = speech.listen_and_get_text()
        
        next_action_llm = llm_manager.parse_next_action_decision(
            next_action_response_text,
            request_summary, # Context of the original request
            any_request_fulfilled_overall, # Was the original request fulfilled AT ANY POINT
            has_more_options_for_this_original_request # Are there more stores in THIS LIST for current request
        )

        if next_action_llm.get("error"):
            speech.say("I'm a bit unsure what to do next. Let's assume we are done with this search for now, and you can give me a new command if you like.")
            return {"status": "completed_recommendations_for_query"}

        intent_from_llm = next_action_llm.get("intent")
        new_query_text_from_llm = next_action_llm.get("new_query_text")

        if intent_from_llm == "new_request" and new_query_text_from_llm:
            speech.say(f"Okay, let's look into: {new_query_text_from_llm}")
            return {"status": "new_request", "detail": new_query_text_from_llm}
        elif intent_from_llm == "stop_interaction":
            speech.say("Alright, understood.")
            # If overall request was fulfilled, it implies user is happy to stop the whole session or this line.
            # If not fulfilled, but user wants to stop, it means stop this line of query.
            return {"status": "stop_session" if any_request_fulfilled_overall else "completed_recommendations_for_query"}
        elif intent_from_llm == "continue_current_request":
            if has_more_options_for_this_original_request:
                speech.say(f"Okay, let's see the next option for {request_summary}.")
                # The loop will naturally continue to the next store_option_details
            else:
                speech.say(f"Actually, it seems that was the last option I had for {request_summary}.")
                return {"status": "completed_recommendations_for_query"} # End this line of query
        else: # Default or unrecognized LLM intent for next action
            speech.say("I'll assume we are done with that particular request for now.")
            return {"status": "completed_recommendations_for_query"} # End this line of query

    # If loop finishes (all stores in ranked_store_options_with_details were visited or skipped)
    if not any_request_fulfilled_overall:
        speech.say(f"We've gone through all the suggestions for {request_summary}, but it seems we didn't find exactly what you wanted this time.")
    else:
        speech.say(f"We've explored the options for {request_summary}.")
    
    return {"status": "completed_recommendations_for_query"} # Indicates this specific query's recommendations are exhausted


def process_intent(intent_from_llm, parsed_llm_data, shopping_for_user_role, active_profile_for_recs):
    """
    Handles the main intent from the LLM, delegating to other functions as needed.
    Returns a status dictionary for the main loop.
    """
    if intent_from_llm == "show_profile":
        speech.say(f"Okay, here are the current preferences I have for {shopping_for_user_role.capitalize()}:")
        profile_manager.display_profile(shopping_for_user_role)
        return {"status": "interaction_completed_for_current_query"}

    if intent_from_llm == "update_profile_only":
        # Basic preferences should have already been updated in the main loop before calling this.
        speech.say(f"Preferences have been updated for {shopping_for_user_role.capitalize()}.")
        profile_manager.display_profile(shopping_for_user_role) # Display the updated profile
        return {"status": "interaction_completed_for_current_query"}

    if intent_from_llm in {"find_product", "find_store"}:
        # Construct query for Knowledge Graph Service
        kg_query = {
            "intent": intent_from_llm,
            "item_types": parsed_llm_data.get("item_types", []),
            "attributes": parsed_llm_data.get("attributes", {}),
            "store_name": parsed_llm_data.get("store_name") # Used if LLM identified a specific store for product search or direct lookup
        }
        query_results_from_kg = kg_service.execute_structured_query(kg_query)
        
        current_robot_pose = nav_manager.get_current_pose()
        
        # Generate recommendations based on KG results, user profile, and robot pose
        recommendations_data = recommender.generate_recommendations(
            parsed_llm_data, # Contains original intent, items, attributes from LLM
            query_results_from_kg, # Contains stores/products found in KG
            active_profile_for_recs, # Profile of the person being shopped for
            current_robot_pose
        )

        # Prepare a list of store options with details for the iterative handler
        ranked_store_options_for_iteration = []
        if intent_from_llm == "find_store" and recommendations_data.get("stores"):
            for store_recommendation in recommendations_data["stores"]: # These are already scored and sorted
                ranked_store_options_for_iteration.append({
                    "store_info": store_recommendation, # Contains name, map_x, map_y, etc.
                    "product_count_if_any": None # Not applicable for pure "find_store"
                })
        
        elif intent_from_llm == "find_product" and recommendations_data.get("products_in_stores_ranked"):
            for product_location_recommendation in recommendations_data["products_in_stores_ranked"]:
                ranked_store_options_for_iteration.append({
                    "store_info": product_location_recommendation["store_details"], 
                    "product_count_if_any": len(product_location_recommendation.get("products_found", []))
                })
        
        if not ranked_store_options_for_iteration:
            speech.say(f"Sorry, I couldn't find any suitable stores or products based on your request for {_generate_request_summary(parsed_llm_data)} right now.")
            return {"status": "interaction_completed_for_current_query"}

        # Delegate to the iterative store visit handler
        return handle_iterative_store_visits(
            original_request_parsed_data=parsed_llm_data,
            ranked_store_options_with_details=ranked_store_options_for_iteration,
            shopping_for_user_role=shopping_for_user_role,
            active_profile_for_recommendations=active_profile_for_recs
        )

    # Fallback for unhandled intents (should be rare if LLM is well-prompted)
    speech.say(f"I'm not quite sure how to handle the intent: '{intent_from_llm}'. Can you try rephrasing?")
    return {"status": "interaction_completed_for_current_query"} # Or perhaps "error_unhandled_intent"


def run_shopping_session():
    global current_speaker_role

    speech.wait_for_wake_word()
    initialize_speaker_if_needed() # Establishes current_speaker_role for the session

    if not current_speaker_role: # Should be set by initialize_speaker_if_needed
        print("CRITICAL: Speaker role not set after initialization. Exiting.")
        speech.say("I'm having a startup issue because the speaker role wasn't identified. Please try restarting me.")
        return

    speech.say(f"Hello {current_speaker_role.capitalize()}! How can I help you or your family today? (You can say 'stop' to end our session.)")
    current_user_input_for_llm = speech.listen_and_get_text()

    while True:
        # Universal stop command check
        if not current_user_input_for_llm or current_user_input_for_llm.strip().lower() == "stop":
            current_parsed_llm_output = {"intent": "stop_interaction"} # Force stop intent
        else:
            # For each new primary user utterance, call the LLM.
            # No shopping_for_user_profile_context is passed here because each new query might be for a different target.
            # The LLM should attempt to extract the target from THIS specific utterance.
            current_parsed_llm_output = llm_manager.generate_structured_query(
                current_user_input_for_llm,
                current_speaker_role_if_known=current_speaker_role
            )

        if not current_parsed_llm_output or current_parsed_llm_output.get("error"):
            speech.say("I had a little trouble understanding that. Could you please try rephrasing your request?")
            current_user_input_for_llm = speech.listen_and_get_text()
            if current_user_input_for_llm.strip().lower() == "stop": # Allow stopping even after an error
                current_parsed_llm_output = {"intent": "stop_interaction"} # Process stop
            else:
                continue # Restart loop with new input, skipping further processing of the errorneous parse

        if current_parsed_llm_output.get("intent") == "stop_interaction":
            speech.say("Alright, ending our shopping trip. Have a great day!")
            break # Exit the main while loop

        # --- Determine who THIS SPECIFIC request is for ---
        shopping_for_user_role_for_this_query = None
        llm_extracted_target = current_parsed_llm_output.get("shopping_for_user")
        intent_for_this_query = current_parsed_llm_output.get("intent")

        if llm_extracted_target == "self":
            shopping_for_user_role_for_this_query = current_speaker_role
        elif llm_extracted_target in VALID_USER_KEYS:
            shopping_for_user_role_for_this_query = llm_extracted_target
        else:
            # LLM didn't specify a valid target, or specified null. Clarify if the intent needs a target.
            relevant_intents_for_clarification = {"find_product", "find_store", "show_profile", "update_profile_only"}
            if intent_for_this_query in relevant_intents_for_clarification:
                speech.say(f"I understood your request as: \"{current_user_input_for_llm}\".") # Acknowledge original request
                clarified_target = clarify_shopping_target()
                if clarified_target: # User provided a valid target
                    shopping_for_user_role_for_this_query = clarified_target
                else: # Clarification failed or user indicated stop/cancel during clarification
                    speech.say("Okay, what would you like to do instead then?")
                    current_user_input_for_llm = speech.listen_and_get_text()
                    continue # Restart main loop with new input
            else: 
                # Intent doesn't strictly need a target (e.g., a general greeting if supported, or an error)
                # For robustness, assume it's for the speaker if no other target is sensible for the intent.
                shopping_for_user_role_for_this_query = current_speaker_role

        speech.say(f"Okay, I'll work on that for {shopping_for_user_role_for_this_query.capitalize()}.")

        # --- Update basic preferences if LLM extracted them for THIS query's target ---
        profile_updates_from_llm = current_parsed_llm_output.get("updated_profile_for_shopping_user")
        if profile_updates_from_llm: # Check if it's not None and not empty
            profile_manager.update_basic_preferences(shopping_for_user_role_for_this_query, profile_updates_from_llm)
            # speech.say(f"I've noted down those preferences for {shopping_for_user_role_for_this_query.capitalize()}.") # Can be verbose

        active_profile_for_this_query = profile_manager.get_profile(shopping_for_user_role_for_this_query)
        
        # --- Process the intent for THIS query ---
        session_status = process_intent(
            intent_for_this_query,
            current_parsed_llm_output, # The structured data from LLM for the current utterance
            shopping_for_user_role_for_this_query,
            active_profile_for_this_query
        )

        print(f"[DEBUG] Main loop received session status: {session_status}")
        print("-" * 60) # Separator for console readability

        # --- Decide next main loop action based on what process_intent (and its sub-handlers) returned ---
        if session_status.get("status") == "new_request":
            current_user_input_for_llm = session_status.get("detail", "") # Get the new query text
            # Loop will continue, and this new input will be parsed by LLM at the top
        elif session_status.get("status") == "stop_session":
            # Iterative handler decided the whole session should stop.
            # Message already given by handler or the stop_interaction block above.
            break # Exit the main while loop
        # Covers "completed_recommendations_for_query", "interaction_completed_for_current_query", or any other/default status
        else: 
            speech.say(f"Is there anything else I can help you or the family with, {current_speaker_role.capitalize()}? (Or say 'stop')")
            current_user_input_for_llm = speech.listen_and_get_text()
            # Loop will continue, LLM will parse this new input

    # --- End of session ---
    nav_manager.shutdown()
    speech.shutdown()
    profile_manager._save_profiles() # Ensure all profile changes are written to disk
    print("Session complete. All components shut down.")


if __name__ == "__main__":
    PREFS_FILE = "family_preferences_main.json"
    # Clear previous test file for a clean run, good for testing initialization
    if os.path.exists(PREFS_FILE):
        try:
            os.remove(PREFS_FILE)
            print(f"Removed old preferences file: {PREFS_FILE} for fresh test run.")
        except OSError as e:
            print(f"Warning: Could not remove old preferences file {PREFS_FILE}: {e}")
            
    # Re-initialize profile manager to ensure it creates a new file if one was removed or didn't exist
    # This also ensures it loads with default structures for valid users if the file is new/empty.
    profile_manager = UserProfileManager(users_file=PREFS_FILE) 
    # Note: Other global components (kg_service, recommender, nav_manager, llm_manager) are already initialized.
    # If profile_manager was a dependency for them in a different way, re-injection might be needed.
    # Here, recommender takes profile_manager at init, so the global instance is used.

    # --- Comprehensive script for iterative flow ---
    # This script tests speaker ID, multi-turn requests, store visits, feedback, and changing shopping targets.
    # Assumes graph_data_generator.py creates:
    # - "Kids Playworld" (S9) selling "Toy" (I11)
    # - "Toy Emporium" (hypothetical S10, or modify another store) also selling "Toy" (I11) for multi-option test
    # - "Chic Boutique" (S4) selling "Dress" (I4)
    # - "Urban Stylez" (S1) also selling "Dress" (I4) or some other clothing for multi-option test
    
    # To make the test fully work, ensure your graph_data_generator.py supports these.
    # If "Toy Emporium" doesn't exist, the "continue_current_request" for toys might not offer a second option.
    # For testing, you can modify graph_data_generator.py to ensure these stores and items exist.
    # For example, add to stores_data:
    # {"id": "S10", "name": "Toy Emporium", "type": "Large Toy Store", "location_text": "Floor 3, Unit 301", "sells_base_classes": ["I11"], "map_x": 5.0, "map_y": 15.0, "map_theta": 0.0},
    # And ensure "I11" is "Toy".

    test_script_for_dialogue = [
        # 1. Speaker Identification
        "I am the Mother.", # Who is speaking? -> Mother (current_speaker_role = "mother")

        # 2. Initial request (toys for child), first store visit (Kids Playworld)
        "I'm looking for a toy for my child.", # Main query
            # LLM: find_product, item_types=["Toy"], shopping_for_user="child"
            # Robot: "Okay, I'll work on that for Child."
            # Robot proposes Kids Playworld: "...Kids Playworld. Shall we go?"
        "Yes, let's go.", # Confirm navigation to Kids Playworld
            # Robot navigates... "Arrived... ready for feedback?"
        "Okay, I'm ready now.", # User signals readiness for feedback questions
            # Robot: "Were you able to find a toy for the child here at Kids Playworld?"
        "No, not quite the right one this time.", # Fulfillment status for Kids Playworld
            # LLM: {"fulfilled": false}
            # Robot: "Oh, I'm sorry. Any other feedback about Kids Playworld?"
        "It was a bit small, but the staff tried hard to help.", # General feedback for Kids Playworld
            # LLM: parses feedback for profile (shop_review_update, item_feedback_updates)
            # Robot: "Thanks! ... What next? Try another toy store (if Toy Emporium exists and has toys)...?"
        "Are there any other toy stores available?", # Next action: continue current request for toys
            # LLM: {"intent": "continue_current_request"}
            # Robot (assuming "Toy Emporium" is found as another option): "Okay, the next option is Toy Emporium. Shall we go?"
        
        # 3. Second store visit for the same toy request (Toy Emporium)
        "Sure, let's try that one.", # Confirm navigation to Toy Emporium
            # Robot navigates to Toy Emporium... "Arrived... ready for feedback?"
        "I've had a good look around.", # User signals readiness
            # Robot: "Did you find a toy for the child here at Toy Emporium?"
        "Yes! We found a fantastic building block set here. Perfect!", # Fulfillment status for Toy Emporium
            # LLM: {"fulfilled": true} -> any_request_fulfilled_overall becomes true
            # Robot: "That's great! Any other feedback about Toy Emporium?"
        "They had a wonderful selection and it was very well organized. Much better.", # General feedback for Toy Emporium
            # LLM: parses feedback for profile
            # Robot: "Thanks! ... What next? Look for something else, or stop for now?" (as request was fulfilled)
        
        # 4. New request (dress for self - mother)
        "Now, I'd like to find a red dress for myself.", # Next action: new_request
            # LLM: {"intent": "new_request", "new_query_text": "find a red dress for myself"}
            # Main loop gets this, calls LLM again for this new query.
            # New LLM parse: find_product, item_types=["Dress"], attributes={"color": "Red"}, shopping_for_user="self" (which is mother)
            # Robot: "Okay, I'll work on that for Mother."
            # Robot proposes Chic Boutique: "...Chic Boutique. Shall we go?"
        "No, let's skip Chic Boutique for now.", # Decline first option for dress
            # Robot: "Okay, we'll skip Chic Boutique."
            # Robot (assuming "Urban Stylez" is another option for dresses): "The next option for a red dress is Urban Stylez. Shall we go?"
        "Yes, let's check Urban Stylez.", # Accept second option
            # Robot navigates to Urban Stylez... "Arrived... ready for feedback?"
        "Finished looking here.",
            # Robot: "Were you able to find a red dress for yourself here at Urban Stylez?"
        "Unfortunately no, they didn't have it in my size, only very small ones.",
            # LLM: {"fulfilled": false} -> any_request_fulfilled_overall for *this dress request* is false
            # Robot: "Oh, sorry to hear that. Any other feedback about Urban Stylez?"
        "The store was nice and bright though, good layout.",
            # LLM: parses feedback
            # Robot: "Thanks. ... What next? (assuming no more dress stores or user implies stopping this dress search)"
        
        # 5. Stop interaction
        "That's all for today, thank you very much.", # Next action: stop
            # LLM (for next_action_decision): {"intent": "stop_interaction"}
            # Robot: "Alright, understood." -> handle_iterative_store_visits returns {"status": "completed_recommendations_for_query"} or {"status": "stop_session"}
            # Main loop then asks "Is there anything else..." or breaks.
        "stop" # Final "stop" to explicitly exit the very last prompt if any, or if previous was "completed_recommendations..."
    ]
    speech.set_script(test_script_for_dialogue)
    
    current_speaker_role = None # Reset global for the test run to trigger initialization
    run_shopping_session()

    # --- Display final profiles to check if updates were stored ---
    print("\n" + "="*20 + " Final User Profiles After Session " + "="*20)
    # Create a new manager instance to load from the file, ensuring we see what was saved.
    final_profile_manager_checker = UserProfileManager(users_file=PREFS_FILE) 
    for role_key in VALID_USER_KEYS:
        final_profile_manager_checker.display_profile(role_key)
    print("="*60)