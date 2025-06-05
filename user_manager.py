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
        self.profiles = {} # Initialize as empty, reload_profiles_from_file will populate
        self.reload_profiles_from_file(initial_load=True) # Load on init

    def _ensure_default_structure(self, profiles_dict):
        """Ensures all valid users exist and have the default structure."""
        structure_changed = False
        for user_key in VALID_USER_KEYS:
            if user_key not in profiles_dict:
                profiles_dict[user_key] = copy.deepcopy(DEFAULT_USER_PROFILE_STRUCTURE)
                structure_changed = True
            else:
                # Ensure all top-level keys from DEFAULT_USER_PROFILE_STRUCTURE exist
                for default_field, default_value in DEFAULT_USER_PROFILE_STRUCTURE.items():
                    if default_field not in profiles_dict[user_key]:
                        profiles_dict[user_key][default_field] = copy.deepcopy(default_value)
                        structure_changed = True
                    # Ensure nested dicts like shop_reviews and item_feedback are dicts
                    elif isinstance(default_value, dict) and not isinstance(profiles_dict[user_key][default_field], dict):
                        profiles_dict[user_key][default_field] = copy.deepcopy(default_value)
                        structure_changed = True
                    # Ensure list types are lists
                    elif isinstance(default_value, list) and not isinstance(profiles_dict[user_key][default_field], list):
                        profiles_dict[user_key][default_field] = copy.deepcopy(default_value)
                        structure_changed = True

        return structure_changed

    def reload_profiles_from_file(self, initial_load=False):
        """Reloads profiles from the JSON file."""
        loaded_profiles_from_file = {}
        if os.path.exists(self.users_file):
            try:
                with open(self.users_file, 'r') as f:
                    raw_loaded = json.load(f)
                    loaded_profiles_from_file = {str(k).lower(): v for k, v in raw_loaded.items()}
            except json.JSONDecodeError:
                print(f"USER_MANAGER_WARNING: Could not decode JSON from {self.users_file}. Using empty profiles for this load.")
                loaded_profiles_from_file = {}
        else:
            if not initial_load:
                print(f"USER_MANAGER_WARNING: Profiles file {self.users_file} not found for reloading. Using empty profiles for this load.")
            loaded_profiles_from_file = {}

        # Ensure default structure for all valid users based on what was loaded (or not loaded)
        # This will create missing users or missing fields within existing users.
        self._ensure_default_structure(loaded_profiles_from_file)
        self.profiles = loaded_profiles_from_file # Assign the processed profiles

        # If it's the very first initialization and the file was created or modified to fit the default structure, save it.
        if initial_load:
            # Check if current self.profiles (normalized) differs from what might have been an incomplete file
            temp_profiles_on_disk = {}
            if os.path.exists(self.users_file):
                try:
                    with open(self.users_file, 'r') as f_check:
                        temp_profiles_on_disk = json.load(f_check)
                except json.JSONDecodeError: # File was bad, so it definitely needs saving
                    pass # Handled by the fact that self.profiles would be different

            # Normalize keys of temp_profiles_on_disk for fair comparison
            normalized_temp_profiles_on_disk = {str(k).lower(): v for k, v in temp_profiles_on_disk.items()}


            # If the normalized loaded profiles differ from what was on disk (or if disk was empty/corrupt)
            # and _ensure_default_structure potentially added/fixed things.
            if json.dumps(self.profiles, sort_keys=True) != json.dumps(normalized_temp_profiles_on_disk, sort_keys=True):
                 print(f"USER_MANAGER_INFO: Initial profile structure normalized/created from {self.users_file}. Saving back to ensure consistency.")
                 self._save_profiles()

        print(f"USER_MANAGER_INFO: Profiles {'loaded' if initial_load else 'reloaded'} from {self.users_file}")


    def _save_profiles(self):
        try:
            with open(self.users_file, 'w') as f:
                json.dump(self.profiles, f, indent=2)
            print(f"USER_MANAGER_INFO: Profiles saved to {self.users_file}")
        except IOError as e:
            print(f"USER_MANAGER_ERROR: Error saving profiles to {self.users_file}: {e}")

    def get_profile(self, user_name):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"USER_MANAGER_WARNING: User '{user_name}' is not a pre-defined valid user. Returning a temporary default profile.")
            return copy.deepcopy(DEFAULT_USER_PROFILE_STRUCTURE)

        if user_name_lower not in self.profiles:
            # This should ideally be caught by _ensure_default_structure during load/reload
            print(f"USER_MANAGER_ERROR: Profile for valid user '{user_name_lower}' unexpectedly missing from in-memory profiles. Re-creating default.")
            self.profiles[user_name_lower] = copy.deepcopy(DEFAULT_USER_PROFILE_STRUCTURE)
            # Consider if a save is needed here if this state is reached, though it shouldn't be.

        # The profile returned should be complete due to _ensure_default_structure in reload_profiles_from_file
        return self.profiles[user_name_lower]


    def update_basic_preferences(self, user_name, updates_from_llm):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"USER_MANAGER_ERROR: Cannot update preferences for invalid user '{user_name}'.")
            return False

        profile = self.get_profile(user_name_lower)
        updated_any = False

        for key, new_values_list_or_str in updates_from_llm.items():
            if key in profile and isinstance(profile[key], list):
                values_to_add = []
                if isinstance(new_values_list_or_str, list):
                    values_to_add = new_values_list_or_str
                elif isinstance(new_values_list_or_str, str):
                    values_to_add = [new_values_list_or_str]

                for value in values_to_add:
                    if value not in profile[key]:
                        profile[key].append(value)
                        updated_any = True
        if updated_any:
            print(f"USER_MANAGER_PROFILE: Basic preferences for {user_name_lower.capitalize()} updated in memory.")
        return updated_any

    def update_profile_with_feedback(self, user_name, shop_name, parsed_feedback_from_llm):
        user_name_lower = str(user_name).lower()
        if user_name_lower not in VALID_USER_KEYS:
            print(f"USER_MANAGER_ERROR: Cannot update feedback for invalid user '{user_name}'.")
            return False

        profile = self.get_profile(user_name_lower)
        overall_updated = False

        if "shop_review_update" in parsed_feedback_from_llm and isinstance(parsed_feedback_from_llm["shop_review_update"], dict):
            review_data = parsed_feedback_from_llm["shop_review_update"]
            if shop_name not in profile["shop_reviews"]:
                profile["shop_reviews"][shop_name] = {} # Initialize if new shop for this user

            shop_review_entry = profile["shop_reviews"][shop_name]
            changed_shop_review = False

            # Ensure notes lists exist
            if "notes_positive" not in shop_review_entry: shop_review_entry["notes_positive"] = []
            if "notes_negative" not in shop_review_entry: shop_review_entry["notes_negative"] = []


            if review_data.get("request_fulfilled") is not None:
                shop_review_entry["last_visit_fulfilled"] = review_data["request_fulfilled"]
                changed_shop_review = True
            if review_data.get("rating") is not None:
                shop_review_entry["rating"] = review_data["rating"]
                changed_shop_review = True

            for note_type_key in ["notes_positive", "notes_negative"]:
                if note_type_key in review_data and isinstance(review_data[note_type_key], list):
                    if not isinstance(shop_review_entry.get(note_type_key), list): # Ensure list exists
                        shop_review_entry[note_type_key] = []
                    for note in review_data[note_type_key]:
                        if note not in shop_review_entry[note_type_key]:
                            shop_review_entry[note_type_key].append(note)
                            changed_shop_review = True
            if changed_shop_review:
                overall_updated = True

        if "item_feedback_updates" in parsed_feedback_from_llm and isinstance(parsed_feedback_from_llm["item_feedback_updates"], list):
            item_updates = parsed_feedback_from_llm["item_feedback_updates"]
            changed_item_feedback = False
            for item_update in item_updates:
                category_name = item_update.get("category_name")
                if not category_name:
                    continue

                if category_name not in profile["item_feedback"]:
                    profile["item_feedback"][category_name] = {"likes": [], "dislikes": []}

                item_feedback_entry = profile["item_feedback"][category_name]

                for feedback_type_key in ["likes", "dislikes"]:
                    if feedback_type_key in item_update and isinstance(item_update[feedback_type_key], list):
                        for detail in item_update[feedback_type_key]:
                            if detail not in item_feedback_entry[feedback_type_key]:
                                item_feedback_entry[feedback_type_key].append(detail)
                                changed_item_feedback = True
            if changed_item_feedback:
                overall_updated = True

        if overall_updated:
            print(f"USER_MANAGER_PROFILE: Feedback-based preferences for {user_name_lower.capitalize()} updated in memory.")
        return overall_updated


    def display_profile(self, user_name):
        user_name_lower = str(user_name).lower()
        # Get profile ensures structure, so we can directly access keys if they should exist
        profile_to_display = self.get_profile(user_name_lower)

        # If get_profile returned a temporary default for an invalid user, it won't be in self.profiles
        if user_name_lower not in VALID_USER_KEYS: # Already handled by get_profile's print
            return


        print(f"\n Current preferences for {user_name_lower.capitalize()}:")
        has_preferences = False

        # Use .get with default for safety, though get_profile should ensure keys
        if profile_to_display.get("favorite_colors"):
            print(f"   Favorite colors: {', '.join(profile_to_display['favorite_colors'])}")
            has_preferences = True
        if profile_to_display.get("favorite_brands"):
            print(f"   Favorite brands: {', '.join(profile_to_display['favorite_brands'])}")
            has_preferences = True
        if profile_to_display.get("preferred_categories"):
            print(f"   Preferred categories: {', '.join(profile_to_display['preferred_categories'])}")
            has_preferences = True

        shop_reviews_data = profile_to_display.get("shop_reviews", {})
        if shop_reviews_data:
            print("   Shop Reviews:")
            for shop, review in shop_reviews_data.items():
                review_details = []
                if "rating" in review and review["rating"] is not None: review_details.append(f"Rating: {review['rating']}/5")
                if "last_visit_fulfilled" in review and review["last_visit_fulfilled"] is not None: review_details.append(f"Fulfilled: {review['last_visit_fulfilled']}")
                if review.get("notes_positive"): review_details.append(f"Liked: {', '.join(review['notes_positive'])}")
                if review.get("notes_negative"): review_details.append(f"Disliked: {', '.join(review['notes_negative'])}")
                if review_details:
                    print(f"     - {shop}: {'; '.join(review_details)}")
                    has_preferences = True
                elif not review_details and shop:
                    print(f"     - {shop}: (No specific review details recorded)")

        item_feedback_data = profile_to_display.get("item_feedback", {})
        if item_feedback_data:
            print("   Item Feedback:")
            for category, feedback in item_feedback_data.items():
                feedback_details = []
                if feedback.get("likes"): feedback_details.append(f"Likes: {', '.join(feedback['likes'])}")
                if feedback.get("dislikes"): feedback_details.append(f"Dislikes: {', '.join(feedback['dislikes'])}")
                if feedback_details:
                    print(f"     - {category}: {'; '.join(feedback_details)}")
                    has_preferences = True
                elif not feedback_details and category:
                    print(f"     - {category}: (No specific item feedback recorded)")

        if not has_preferences:
            print("   No specific preferences recorded yet for this user.")

if __name__ == '__main__':
    TEST_PREFS_FILE = "test_family_prefs_main.json"
    ORIG_TEST_PREFS_CONTENT = None
    if os.path.exists(TEST_PREFS_FILE):
        with open(TEST_PREFS_FILE, 'r') as f_orig:
            ORIG_TEST_PREFS_CONTENT = f_orig.read()

    if os.path.exists(TEST_PREFS_FILE):
        os.remove(TEST_PREFS_FILE)
    # Create a specific initial state for testing the reload and normalization
    initial_test_state = {
        "mother": {"favorite_colors": ["InitialBlue"], "shop_reviews": {"StoreX": {"rating": 3}}},
        "father": {"favorite_brands": ["InitialBrandY"]},
        # Child is missing, _ensure_default_structure should add it
    }
    with open(TEST_PREFS_FILE, 'w') as f_test_init:
        json.dump(initial_test_state, f_test_init, indent=2)
    print(f"--- Test setup: Created {TEST_PREFS_FILE} with initial state ---")


    manager = UserProfileManager(users_file=TEST_PREFS_FILE)
    print("\n--- Profiles after initial UserProfileManager instantiation ---")
    # This will have loaded initial_test_state and normalized it (added child, added missing fields)
    for user_key_test in VALID_USER_KEYS:
        manager.display_profile(user_key_test)

    # Check if the file was updated by __init__ if normalization occurred
    print(f"\n--- Content of {TEST_PREFS_FILE} after manager init (should be normalized) ---")
    if os.path.exists(TEST_PREFS_FILE):
        with open(TEST_PREFS_FILE, 'r') as f_content:
            print(f_content.read())
    else:
        print(f"{TEST_PREFS_FILE} not found.")


    print("\n--- Simulate updating Mother's prefs (in memory) ---")
    mum_prefs_update = {"favorite_colors": ["Purple", "Silver"], "favorite_brands": ["Glamora"]} # Additive
    manager.update_basic_preferences("Mother", mum_prefs_update)
    print("Mother's profile (in memory after update):")
    manager.display_profile("mother")

    print("\n--- Simulate saving profiles at end of an iteration ---")
    manager._save_profiles()
    print(f"--- Content of {TEST_PREFS_FILE} after save ---")
    if os.path.exists(TEST_PREFS_FILE):
        with open(TEST_PREFS_FILE, 'r') as f_content:
            print(f_content.read())


    print("\n--- Simulate start of a new iteration: Reloading profiles on the SAME manager instance ---")
    # Create a different state on disk to verify reload works
    new_disk_state = {
        "mother": {"favorite_colors": ["ReloadedGreen"], "preferred_categories": ["Toy"]},
        "father": {"favorite_brands": ["ReloadedBrandZ"]},
        "child": {"favorite_colors": ["ReloadedRed"]}
    }
    with open(TEST_PREFS_FILE, 'w') as f_new_disk:
        json.dump(new_disk_state, f_new_disk, indent=2)
    print(f"--- Test: Changed content of {TEST_PREFS_FILE} on disk to simulate external update ---")

    manager.reload_profiles_from_file() # Explicit reload

    print("\n--- Profiles on the manager instance AFTER explicit reload ---")
    for user_key_test in VALID_USER_KEYS:
        manager.display_profile(user_key_test) # Should reflect new_disk_state

    # Clean up / Restore
    if ORIG_TEST_PREFS_CONTENT:
        with open(TEST_PREFS_FILE, 'w') as f_restore:
            f_restore.write(ORIG_TEST_PREFS_CONTENT)
        print(f"\n--- Test cleanup: Restored original content to {TEST_PREFS_FILE} ---")
    elif os.path.exists(TEST_PREFS_FILE):
        os.remove(TEST_PREFS_FILE)
        print(f"\n--- Test cleanup: Removed {TEST_PREFS_FILE} created during test ---")