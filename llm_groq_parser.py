# llm_groq_parser.py
import os
from groq import Groq
import json
from graph_data_generator import colors_list, fictional_brands_list, fashion_mnist_classes
from dotenv import load_dotenv

load_dotenv()

class GroqQueryParser:
    def __init__(self):
        api_key = os.environ.get("GROQ_API_KEY")
        if not api_key:
            raise ValueError("GROQ_API_KEY not found in environment variables. Check your .env file")
        
        self.client = Groq(api_key=api_key)
        self.valid_colors_lower = [c.lower() for c in colors_list]
        self.valid_brands = fictional_brands_list # Already TitleCased
        self.valid_brands_lower_map = {b.lower(): b for b in self.valid_brands}
        self.valid_item_categories = list(set(c["name"] for c in fashion_mnist_classes)) # Already in correct case
        self.store_names_list = [] # Original casing
        self.store_names_lower_map = {}

    def set_store_names(self, store_names_from_graph):
        self.store_names_list = list(store_names_from_graph)
        self.store_names_lower_map = {s.lower(): s for s in self.store_names_list}

    def generate_structured_query(self, user_query, user_profile=None):
        user_profile_json_string = json.dumps(user_profile if user_profile else {})

        prompt = f"""You are a helpful mall assistant. Analyze the user's query and their profile to understand their request and potentially update their preferences.
Return STRICTLY VALID JSON with the following fields:
- "intent": (string) "find_product", "find_store", "update_profile_only", or "show_profile".
- "attributes": (object, optional) Product attributes like {{"color": "Red", "brand": "Urban Threads"}}. Values should be capitalized if applicable (e.g. color).
- "item_types": (array of strings, optional) Product categories like ["Dress", "Bag"] from the available list.
- "store_name": (string, optional) A store name, exactly matching one from the available list.
- "updated_profile": (object, optional) If the query reveals new or changed GENERAL preferences for the user, include this.
  A general preference is stated with phrases like "I like...", "My favorite is...", "I often wear...", "I'm interested in...".
  Do NOT infer a general preference from a specific, one-time search request (e.g., "find a red t-shirt" does not mean red is a new favorite color, unless they also say "I love red").
  Structure: {{"favorite_colors": ["Red"], "favorite_brands": ["Urban Threads"], "preferred_categories": ["Dress"]}}.
  Values in these arrays should be drawn from the available lists and correctly cased. Append new, validated preferences.

USER PROFILE:
{user_profile_json_string}

AVAILABLE ITEM CATEGORIES: {self.valid_item_categories}
AVAILABLE STORE NAMES: {self.store_names_list}
AVAILABLE COLORS (lowercase for your reference, output capitalized): {self.valid_colors_lower}
AVAILABLE BRANDS (output in this casing): {self.valid_brands}


RULES FOR RESPONSE:
1. Output ONLY a single, valid JSON object. No markdown, no explanations, no ```json wrapper.
2. Use double quotes for all keys and string values in the JSON.
3. If updating profile, ensure values for colors, brands, item_types are from the available lists or are general terms, and use correct casing as specified.
4. If intent is "update_profile_only" or "show_profile", other fields like "attributes", "item_types", "store_name" might be empty or null.

Examples:
Query: "Find Sport & Street store"
User Profile: {{}}
Output: {{"intent": "find_store", "store_name": "Active & Street"}}

Query: "Where to buy a leather bag? I love Urban Threads."
User Profile: {{"favorite_colors": ["Blue"]}}
Output: {{"intent": "find_product", "attributes": {{}}, "item_types": ["Bag"], "updated_profile": {{"favorite_brands": ["Urban Threads"]}}}}

Query: "I'm looking for a red t-shirt."
User Profile: {{"favorite_brands": ["Glamora"]}}
Output: {{"intent": "find_product", "attributes": {{"color": "Red"}}, "item_types": ["T-shirt/top"]}}

Query: "I think my favorite color is green now."
User Profile: {{"favorite_colors": ["Blue"]}}
Output: {{"intent": "update_profile_only", "updated_profile": {{"favorite_colors": ["Green"]}}}}

Query: "I really like dresses and bags for casual outings."
User Profile: {{}}
Output: {{"intent": "update_profile_only", "updated_profile": {{"preferred_categories": ["Dress", "Bag"]}}}}
        
Query: "What are my preferences?"
User Profile: {{"favorite_colors": ["Blue"], "favorite_brands": ["Urban Threads"]}}
Output: {{"intent": "show_profile"}}

Current Query: "{user_query}"
Respond ONLY with valid JSON:"""

        try:
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant", # or "llama3-8b-8192" or other suitable models
                temperature=0.1, # Lower temperature for more deterministic JSON
                max_tokens=400,
                response_format={"type": "json_object"}
            )
            
            raw_output = chat_completion.choices[0].message.content
            # print(f"LLM Raw Output: {raw_output}") # For debugging
            
            # Basic cleaning, though response_format should handle most of it
            cleaned_output = raw_output.strip()
            if cleaned_output.startswith("```json"):
                cleaned_output = cleaned_output[7:]
            if cleaned_output.endswith("```"):
                cleaned_output = cleaned_output[:-3]
            cleaned_output = cleaned_output.strip()

            parsed_json = json.loads(cleaned_output)
            return self._validate_output(parsed_json)
            
        except json.JSONDecodeError as e:
            print(f"JSON Decode Error: {e} in output: '{cleaned_output}'")
            return None
        except Exception as e:
            print(f"Groq API or other Error: {str(e)}")
            return None

    def _validate_output(self, output_json):
        validated = {
            "intent": output_json.get("intent", "find_product"), # Default intent
            "attributes": {},
            "item_types": [],
            "store_name": None,
            "updated_profile": {} 
        }
        # print(f"Validating LLM output: {output_json}") # For debugging

        # Validate intent
        valid_intents = ["find_product", "find_store", "update_profile_only", "show_profile"]
        if validated["intent"] not in valid_intents:
            print(f"Warning: LLM returned invalid intent '{validated['intent']}'. Defaulting to 'find_product'.")
            validated["intent"] = "find_product"


        # Validate attributes
        if isinstance(output_json.get("attributes"), dict):
            raw_attributes = output_json["attributes"]
            # Color validation
            color_val = raw_attributes.get("color")
            if isinstance(color_val, str) and color_val.lower() in self.valid_colors_lower:
                validated["attributes"]["color"] = color_val.capitalize()
            
            # Brand validation
            brand_val = raw_attributes.get("brand")
            if isinstance(brand_val, str) and brand_val.lower() in self.valid_brands_lower_map:
                validated["attributes"]["brand"] = self.valid_brands_lower_map[brand_val.lower()]

        # Validate item types
        raw_item_types = output_json.get("item_types", [])
        if isinstance(raw_item_types, list):
            validated["item_types"] = [
                item_type for item_type in raw_item_types
                if isinstance(item_type, str) and item_type in self.valid_item_categories
            ]
        elif isinstance(raw_item_types, str) and raw_item_types in self.valid_item_categories: # if LLM sends a single string
            validated["item_types"] = [raw_item_types]


        # Validate store name
        raw_store_name = output_json.get("store_name")
        if isinstance(raw_store_name, str):
            matched_store = self.store_names_lower_map.get(raw_store_name.lower())
            if matched_store:
                validated["store_name"] = matched_store
            # else:
                # print(f"Warning: LLM suggested store '{raw_store_name}' not in known stores.")


        # Validate updated_profile
        raw_updated_profile = output_json.get("updated_profile")
        if isinstance(raw_updated_profile, dict):
            profile_updates_collector = {}

            # Favorite Colors
            if "favorite_colors" in raw_updated_profile:
                colors_data = raw_updated_profile["favorite_colors"]
                valid_new_colors = []
                if isinstance(colors_data, list):
                    for color_name in colors_data:
                        if isinstance(color_name, str) and color_name.lower() in self.valid_colors_lower:
                            valid_new_colors.append(color_name.capitalize())
                elif isinstance(colors_data, str) and colors_data.lower() in self.valid_colors_lower:
                    valid_new_colors.append(colors_data.capitalize())
                if valid_new_colors:
                    profile_updates_collector["favorite_colors"] = valid_new_colors
            
            # Favorite Brands
            if "favorite_brands" in raw_updated_profile:
                brands_data = raw_updated_profile["favorite_brands"]
                valid_new_brands = []
                if isinstance(brands_data, list):
                    for brand_name in brands_data:
                        if isinstance(brand_name, str) and brand_name.lower() in self.valid_brands_lower_map:
                            valid_new_brands.append(self.valid_brands_lower_map[brand_name.lower()])
                elif isinstance(brands_data, str) and brands_data.lower() in self.valid_brands_lower_map:
                    valid_new_brands.append(self.valid_brands_lower_map[brands_data.lower()])
                if valid_new_brands:
                    profile_updates_collector["favorite_brands"] = valid_new_brands

            # Preferred Categories
            if "preferred_categories" in raw_updated_profile:
                categories_data = raw_updated_profile["preferred_categories"]
                valid_new_categories = []
                if isinstance(categories_data, list):
                    for cat_name in categories_data:
                        if isinstance(cat_name, str) and cat_name in self.valid_item_categories:
                            valid_new_categories.append(cat_name)
                elif isinstance(categories_data, str) and cat_name in self.valid_item_categories: # Fix: cat_name was not defined here
                     if cat_name in self.valid_item_categories: # Double check after fixing scope
                        valid_new_categories.append(cat_name)
                if valid_new_categories:
                    profile_updates_collector["preferred_categories"] = valid_new_categories
            
            if profile_updates_collector:
                validated["updated_profile"] = profile_updates_collector
        
        # print(f"Validated output: {validated}") # For debugging
        return validated