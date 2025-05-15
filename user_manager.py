# user_manager.py
import json
import os

class UserManager:
    def __init__(self):
        self.users_file = "user_preferences.json"
        self.preferences = self._load_preferences()

    def _load_preferences(self):
        if os.path.exists(self.users_file):
            with open(self.users_file, 'r') as f:
                return json.load(f)
        return {"mum": {}, "dad": {}}

    def save_preferences(self):
        with open(self.users_file, 'w') as f:
            json.dump(self.preferences, f)

    def get_preferences(self, user_id):
        return self.preferences.get(user_id.lower(), {})
    
    def update_preferences(self, user_id, new_prefs):
        user_id = user_id.lower()
        if user_id not in self.preferences:
            return
        # Merge new preferences with existing ones
        self.preferences[user_id].update(new_prefs)
        self.save_preferences()