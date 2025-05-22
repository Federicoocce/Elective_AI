# user_profile_manager.py
import json
import os
import copy

VALID_USER_KEYS = ["mother", "father", "child"]

DEFAULT_USER_PROFILE_STRUCTURE = {
    "favorite_colors": [],
    "favorite_brands": [],
    "preferred_categories": [],
    "shop_reviews": {},  # "Shop Name": {"rating": int, "notes_positive": [str], "notes_negative": [str], "last_visit_fulfilled": bool/None}
    "item_feedback": {}  # "Item Category Name": {"likes": [str], "dislikes": [str]}
}

class UserProfileManager:
    def __init__(self, users_file="user_preferences.json"):
        self.users_file = users_file
        self.profiles = self._load_profiles()

        # Ensure default users exist with full structure
        profile_changed_in_init = False
        for user_key in VALID_USER_KEYS:
             if user_key not in self.profiles:
                 self.profiles[user_key] = copy.deepcopy(DEFAULT_USER_PROFILE_STRUCTURE)
                 profile_changed_in_init = True
             else:
                 for default_field, default_value in DEFAULT_USER_PROFILE_STRUCTURE.items():
                     if default_field not in self.profiles[user_key]:
                         self.profiles[user_key][default_field] = copy.deepcopy(default_value)
                         profile_changed_in_init = True
        if profile_changed_in_init:
            self._save_profiles()


    def _load_profiles(self):
        if os.path.exists(self.users_file):
            try:
                with open(self.users_file, 'r') as f:
                    loaded_profiles = json.load(f)
                    return {str(k).lower(): v for k,v in loaded_profiles.items()}
            except json.JSONDecodeError:
                print(f"Warning: Could not decode JSON from {self.users_file}. Initializing with default structure.")
                return {} # Return empty, __init__ will populate defaults
        return {}

    def _save_profiles(self):
        try:
            with open(self.users_file, 'w') as f:
                json.dump(self.profiles, f, indent=2)
        except IOError as e:
            print(f"Error saving profiles to {self.users_file}: {e}")

    def get_profile(self, user_name):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"Warning: User '{user_name}' is not a pre-defined valid user. Returning a default temporary profile.")
            # For non-valid users, return a copy of the default structure but don't save it
            # or handle as an error, depending on desired behavior.
            # For this system, we probably only care about the VALID_USER_KEYS.
            # If an invalid user_name is passed, it might indicate an error in the calling logic.
            # However, to prevent crashes, let's return a default structure.
            temp_profile = copy.deepcopy(DEFAULT_USER_PROFILE_STRUCTURE)
            # It might be better to raise an error or return None if user_name is not in VALID_USER_KEYS
            # to enforce correct usage by the controller.
            # For robustness in the controller's current state:
            if user_name_lower not in self.profiles: # If it's a truly unknown (and invalid) user
                 print(f"Profile for '{user_name_lower}' (invalid) not found and will not be persisted. Using temporary default.")
                 return temp_profile # Return a non-persistent default
            # This line below should ideally not be hit if user_name_lower is not in VALID_USER_KEYS
            # and we are strict about it.

        # Ensure the profile exists and has all keys (should be handled by __init__)
        if user_name_lower not in self.profiles:
            self.profiles[user_name_lower] = copy.deepcopy(DEFAULT_USER_PROFILE_STRUCTURE)
            # self._save_profiles() # Save if a valid user was missing (shouldn't happen after init)

        profile_to_return = self.profiles[user_name_lower]
        
        # Ensure all top-level keys from DEFAULT_USER_PROFILE_STRUCTURE exist (defensive check)
        profile_structure_changed = False
        for key, default_value in DEFAULT_USER_PROFILE_STRUCTURE.items():
            if key not in profile_to_return:
                profile_to_return[key] = copy.deepcopy(default_value)
                profile_structure_changed = True
        if profile_structure_changed:
            self._save_profiles()

        return profile_to_return

    def update_basic_preferences(self, user_name, updates_from_llm):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"Error: Cannot update preferences for invalid user '{user_name}'.")
            return False
            
        profile = self.get_profile(user_name_lower) # Ensures profile exists
        updated_any = False

        for key, new_values_list_or_str in updates_from_llm.items():
            if key in profile and isinstance(profile[key], list): # e.g. favorite_colors
                values_to_add = []
                if isinstance(new_values_list_or_str, list):
                    values_to_add = new_values_list_or_str
                elif isinstance(new_values_list_or_str, str):
                    values_to_add = [new_values_list_or_str]

                for value in values_to_add:
                    if value not in profile[key]:
                        profile[key].append(value)
                        updated_any = True
            # Could add handling for other types of preferences if structure changes

        if updated_any:
            print(f"PROFILE: Basic preferences for {user_name_lower.capitalize()} updated.")
            self._save_profiles()
        return updated_any

    def update_profile_with_feedback(self, user_name, shop_name, parsed_feedback_from_llm):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"Error: Cannot update feedback for invalid user '{user_name}'.")
            return False

        profile = self.get_profile(user_name_lower)
        overall_updated = False

        # Update Shop Review
        if "shop_review_update" in parsed_feedback_from_llm and isinstance(parsed_feedback_from_llm["shop_review_update"], dict):
            review_data = parsed_feedback_from_llm["shop_review_update"]
            if shop_name not in profile["shop_reviews"]:
                profile["shop_reviews"][shop_name] = {} # Initialize if new shop for this user

            shop_review_entry = profile["shop_reviews"][shop_name]
            changed_shop_review = False

            if review_data.get("request_fulfilled") is not None:
                shop_review_entry["last_visit_fulfilled"] = review_data["request_fulfilled"]
                changed_shop_review = True
            if review_data.get("rating") is not None:
                shop_review_entry["rating"] = review_data["rating"]
                changed_shop_review = True
            
            for note_type_key in ["notes_positive", "notes_negative"]:
                if note_type_key in review_data and isinstance(review_data[note_type_key], list):
                    if note_type_key not in shop_review_entry or not isinstance(shop_review_entry[note_type_key], list):
                        shop_review_entry[note_type_key] = [] # Initialize list if not present
                    for note in review_data[note_type_key]:
                        if note not in shop_review_entry[note_type_key]: # Avoid duplicates
                            shop_review_entry[note_type_key].append(note)
                            changed_shop_review = True
            if changed_shop_review:
                overall_updated = True

        # Update Item Feedback
        if "item_feedback_updates" in parsed_feedback_from_llm and isinstance(parsed_feedback_from_llm["item_feedback_updates"], list):
            item_updates = parsed_feedback_from_llm["item_feedback_updates"]
            changed_item_feedback = False
            for item_update in item_updates:
                category_name = item_update.get("category_name")
                if not category_name: # Skip if category name is missing
                    continue

                if category_name not in profile["item_feedback"]:
                    profile["item_feedback"][category_name] = {"likes": [], "dislikes": []}
                
                item_feedback_entry = profile["item_feedback"][category_name]

                for feedback_type_key in ["likes", "dislikes"]:
                    if feedback_type_key in item_update and isinstance(item_update[feedback_type_key], list):
                        for detail in item_update[feedback_type_key]:
                            if detail not in item_feedback_entry[feedback_type_key]: # Avoid duplicates
                                item_feedback_entry[feedback_type_key].append(detail)
                                changed_item_feedback = True
            if changed_item_feedback:
                overall_updated = True

        if overall_updated:
            print(f"PROFILE: Feedback-based preferences for {user_name_lower.capitalize()} updated.")
            self._save_profiles()
        return overall_updated


    def display_profile(self, user_name):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"Cannot display profile for invalid user '{user_name}'.")
            return

        profile = self.get_profile(user_name_lower)
        print(f"\n Current preferences for {user_name_lower.capitalize()}:")
        has_preferences = False

        if profile.get("favorite_colors"):
            print(f"   Favorite colors: {', '.join(profile['favorite_colors'])}")
            has_preferences = True
        if profile.get("favorite_brands"):
            print(f"   Favorite brands: {', '.join(profile['favorite_brands'])}")
            has_preferences = True
        if profile.get("preferred_categories"):
            print(f"   Preferred categories: {', '.join(profile['preferred_categories'])}")
            has_preferences = True

        if profile.get("shop_reviews"):
            print("   Shop Reviews:")
            for shop, review in profile["shop_reviews"].items():
                review_details = []
                if "rating" in review and review["rating"] is not None: review_details.append(f"Rating: {review['rating']}/5")
                if "last_visit_fulfilled" in review and review["last_visit_fulfilled"] is not None: review_details.append(f"Fulfilled: {review['last_visit_fulfilled']}")
                if review.get("notes_positive"): review_details.append(f"Liked: {', '.join(review['notes_positive'])}")
                if review.get("notes_negative"): review_details.append(f"Disliked: {', '.join(review['notes_negative'])}")
                if review_details:
                    print(f"     - {shop}: {'; '.join(review_details)}")
                    has_preferences = True
                elif not review_details and shop: # If shop entry exists but is empty
                    print(f"     - {shop}: (No specific review details recorded)")


        if profile.get("item_feedback"):
            print("   Item Feedback:")
            for category, feedback in profile["item_feedback"].items():
                feedback_details = []
                if feedback.get("likes"): feedback_details.append(f"Likes: {', '.join(feedback['likes'])}")
                if feedback.get("dislikes"): feedback_details.append(f"Dislikes: {', '.join(feedback['dislikes'])}")
                if feedback_details:
                    print(f"     - {category}: {'; '.join(feedback_details)}")
                    has_preferences = True
                elif not feedback_details and category: # If category entry exists but is empty
                    print(f"     - {category}: (No specific item feedback recorded)")


        if not has_preferences:
            print("   No specific preferences recorded yet for this user.")

if __name__ == '__main__':
    TEST_PREFS_FILE = "test_family_prefs_main.json" # Use a distinct name
    if os.path.exists(TEST_PREFS_FILE):
        os.remove(TEST_PREFS_FILE)

    manager = UserProfileManager(users_file=TEST_PREFS_FILE)
    print("--- Testing UserProfileManager ---")
    print("Initial profiles (mother, father, child should exist by default):")
    for user_key_test in VALID_USER_KEYS:
        manager.display_profile(user_key_test)

    print("\nUpdating Mother's basic prefs...")
    mum_prefs_update = {"favorite_colors": ["Purple", "Silver"], "favorite_brands": ["Glamora"]}
    manager.update_basic_preferences("Mother", mum_prefs_update) # Test case-insensitivity of input name
    manager.display_profile("mother")

    print("\nUpdating Father's feedback...")
    feedback_data_father = {
        "shop_review_update": {
            "request_fulfilled": True, "rating": 4,
            "notes_positive": ["Good range of trousers"], "notes_negative": ["A bit warm inside"]
        },
        "item_feedback_updates": [
            {"category_name": "Trouser", "likes": ["Classic Co. brand"], "dislikes": []},
        ]
    }
    manager.update_profile_with_feedback("father", "Classic Comfort", feedback_data_father)
    manager.display_profile("FATHER")

    print("\nUpdating Child's basic prefs...")
    child_prefs_update = {"preferred_categories": ["Toy"], "favorite_colors": ["Red"]}
    manager.update_basic_preferences("child", child_prefs_update)
    manager.display_profile("child")
    
    print("\nAttempting to get profile for invalid user...")
    invalid_profile = manager.get_profile("guest") # Should print warning and return default
    print(f"Profile for 'guest' (should be default, non-persistent): {invalid_profile}")
    manager.display_profile("guest") # Should print "Cannot display..."

    print("\n--- Reloading profiles ---")
    manager_reloaded = UserProfileManager(users_file=TEST_PREFS_FILE)
    for user_key_test in VALID_USER_KEYS:
        manager_reloaded.display_profile(user_key_test)
