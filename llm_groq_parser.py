# llm_groq_parser.py
import os
from groq import Groq
import json
from graph_data_generator import colors_list, fictional_brands_list, fashion_mnist_classes # For lists
from dotenv import load_dotenv

load_dotenv() # Loads variables from .env file into environment

class GroqQueryParser:
    def __init__(self):
        api_key = os.environ.get("GROQ_API_KEY")

        
        self.client = Groq(api_key=api_key)


        # These lists will be populated by set_knowledge_base_lists
        self.valid_colors_lower = []
        self.valid_brands = []
        self.valid_brands_lower_map = {}
        self.valid_item_categories = []
        self.store_names_list = []
        self.store_names_lower_map = {}

    def set_knowledge_base_lists(self, store_names, item_categories, brands, colors):
        """Sets the lists of known entities from the knowledge graph or constants."""
        self.store_names_list = list(store_names)
        self.store_names_lower_map = {s.lower(): s for s in self.store_names_list}

        self.valid_item_categories = list(item_categories)
        self.valid_brands = list(brands)
        self.valid_brands_lower_map = {b.lower(): b for b in self.valid_brands}
        
        self.valid_colors = list(colors) # Store with original casing
        self.valid_colors_lower = [c.lower() for c in self.valid_colors]


    def generate_structured_query(self, user_query, current_speaker_role_if_known=None, shopping_for_user_profile_context=None):
        if not self.client and not os.environ.get("CI_RUN"): # Check if client is available
            print("Error: Groq client not initialized. Cannot make API call.")
            # Return a default/error structure or raise an exception
            return {"error": "Groq client not initialized"}

        user_profile_json_string = json.dumps(shopping_for_user_profile_context if shopping_for_user_profile_context else {})
        
        speaker_info_str = ""
        if current_speaker_role_if_known:
            speaker_info_str = f"The person speaking is likely '{current_speaker_role_if_known}'."
        else:
            speaker_info_str = "The primary speaker is usually 'mother', but the current speaker's specific role is not yet confirmed."

        prompt = f"""You are a helpful mall assistant. Analyze the user's query.
{speaker_info_str}

Return STRICTLY VALID JSON with the following fields:
- "intent": (string) "find_product", "find_store", "update_profile_only", "show_profile", or "stop_interaction".
- "shopping_for_user": (string, optional) The user for whom items are being sought or whose profile is being referenced.
    - If the query explicitly states (e.g., "for my mother", "for the child", "for dad"), extract this. Output "mother", "father", or "child".
    - If the user says "for me" or "for myself", set this field to "self". This implies the items/profile are for the SPEAKER.
    - If not mentioned, output null or omit the field.
- "attributes": (object, optional) Product attributes for search (e.g., {{"color": "Red", "brand": "Urban Threads"}}).
- "item_types": (array of strings, optional) Product categories for search (e.g., ["Dress", "Bag"]).
- "store_name": (string, optional) A store name for search or direct lookup.
- "updated_profile_for_shopping_user": (object, optional) If the query reveals new or changed GENERAL preferences for the "shopping_for_user" (or for the speaker if "shopping_for_user" is "self"), include this.
    - Example: Query "My child loves blue" with shopping_for_user="child" -> updated_profile_for_shopping_user: {{"favorite_colors": ["Blue"]}}.
    - Example: Query "I want a red t-shirt for myself. I love red." with shopping_for_user="self" -> updated_profile_for_shopping_user: {{"favorite_colors": ["Red"]}}.
    - Structure: {{"favorite_colors": ["Red"], "favorite_brands": ["Urban Threads"], "preferred_categories": ["Apparel"]}}.

USER PROFILE (This is the profile of the 'shopping_for_user', if one is established and provided for context):
{user_profile_json_string}

AVAILABLE ITEM CATEGORIES: {self.valid_item_categories}
AVAILABLE STORE NAMES: {self.store_names_list}
AVAILABLE COLORS (lowercase for your reference, output capitalized according to this list: {self.valid_colors}): {self.valid_colors_lower}
AVAILABLE BRANDS (output in this casing from list: {self.valid_brands}): {self.valid_brands}
RECOGNIZED USER ROLES: ["mother", "father", "child"]

RULES FOR RESPONSE:
1. Output ONLY a single, valid JSON object. No markdown, no explanations.
2. "updated_profile_for_shopping_user" applies ONLY to the "shopping_for_user". If the speaker mentions their own preference but is shopping for someone else, do NOT put the speaker's preference in this field.
3. If "shopping_for_user" is "self", the main system will later ask who "self" is if unknown, or use the current_speaker_role_if_known.
4. If the user indicates they want to stop (e.g. "no more", "nothing else thanks", "stop"), set intent to "stop_interaction".

Examples:
Query: "I need a shirt for my father. He prefers Classic Co."
Output: {{"intent": "find_product", "shopping_for_user": "father", "item_types": ["Shirt"], "attributes": {{"brand": "Classic Co."}}, "updated_profile_for_shopping_user": {{"favorite_brands": ["Classic Co."] }} }}

Query: "I'm looking for a new coat for myself. I've started liking the color black."
Output: {{"intent": "find_product", "shopping_for_user": "self", "item_types": ["Coat"], "attributes": {{"color": "Black"}}, "updated_profile_for_shopping_user": {{"favorite_colors": ["Black"]}} }}

Query: "What are Mum's preferences?"
Output: {{"intent": "show_profile", "shopping_for_user": "mother"}}

Query: "My child prefers green trousers. I myself actually like red." (Shopping for child)
Output: {{"intent": "find_product", "shopping_for_user": "child", "item_types": ["Trouser"], "attributes": {{"color": "Green"}}, "updated_profile_for_shopping_user": {{"favorite_colors": ["Green"]}} }}

Query: "No more for today, thanks."
Output: {{"intent": "stop_interaction"}}

Current Query: "{user_query}"
Respond ONLY with valid JSON:"""

        try:
            if not self.client: # Should have been caught earlier, but defensive check
                raise ConnectionError("Groq client not available to make API call.")

            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant",
                temperature=0.1,
                max_tokens=550,
                response_format={"type": "json_object"}
            )
            raw_output = chat_completion.choices[0].message.content
            cleaned_output = raw_output.strip()
            if cleaned_output.startswith("```json"): cleaned_output = cleaned_output[7:]
            if cleaned_output.endswith("```"): cleaned_output = cleaned_output[:-3]
            cleaned_output = cleaned_output.strip()

            parsed_json = json.loads(cleaned_output)
            return self._validate_output_v3(parsed_json)

        except json.JSONDecodeError as e:
            print(f"LLM JSON Decode Error: {e} in output: '{cleaned_output if 'cleaned_output' in locals() else raw_output if 'raw_output' in locals() else 'Unavailable'}'")
            return {"error": "JSON Decode Error", "details": str(e)}
        except Exception as e:
            print(f"Groq API or other LLM Error: {str(e)}")
            return {"error": "Groq API Error", "details": str(e)}
    

    def _validate_output_v3(self, output_json):
        def normalize_color(color):
            return next((c for c in self.valid_colors if c.lower() == color.lower()), color.capitalize())

        def normalize_brand(brand):
            return self.valid_brands_lower_map.get(brand.lower())

        validated = {
            "intent": "find_product",
            "shopping_for_user": None,
            "attributes": {},
            "item_types": [],
            "store_name": None,
            "updated_profile_for_shopping_user": {}
        }

        intent = output_json.get("intent")
        if intent in {"find_product", "find_store", "update_profile_only", "show_profile", "stop_interaction"}:
            validated["intent"] = intent
        else:
            print(f"Warning: LLM returned invalid intent '{intent}'. Defaulting to 'find_product'.")

        user = output_json.get("shopping_for_user")
        if isinstance(user, str) and user.lower() in {"mother", "father", "child", "self"}:
            validated["shopping_for_user"] = user.lower()

        attrs = output_json.get("attributes", {})
        if isinstance(attrs, dict):
            color = attrs.get("color")
            brand = attrs.get("brand")
            if isinstance(color, str) and color.lower() in self.valid_colors_lower:
                validated["attributes"]["color"] = normalize_color(color)
            if isinstance(brand, str) and brand.lower() in self.valid_brands_lower_map:
                validated["attributes"]["brand"] = normalize_brand(brand)

        item_types = output_json.get("item_types")
        if isinstance(item_types, list):
            validated["item_types"] = [i for i in item_types if isinstance(i, str) and i in self.valid_item_categories]
        elif isinstance(item_types, str) and item_types in self.valid_item_categories:
            validated["item_types"] = [item_types]

        store = output_json.get("store_name")
        if isinstance(store, str):
            validated["store_name"] = self.store_names_lower_map.get(store.lower())

        profile_update = output_json.get("updated_profile_for_shopping_user")
        if isinstance(profile_update, dict):
            validated["updated_profile_for_shopping_user"] = self._extract_profile_updates(profile_update)

        return validated


    def _extract_profile_updates(self, raw_profile_dict):
        def collect_valid(items, valid_check, transform):
            if isinstance(items, list):
                return [transform(i) for i in items if isinstance(i, str) and valid_check(i)]
            elif isinstance(items, str) and valid_check(items):
                return [transform(items)]
            return []

        collector = {}

        colors = raw_profile_dict.get("favorite_colors")
        valid_colors = collect_valid(colors, lambda c: c.lower() in self.valid_colors_lower,
                                    lambda c: next((v for v in self.valid_colors if v.lower() == c.lower()), c.capitalize()))
        if valid_colors:
            collector["favorite_colors"] = valid_colors

        brands = raw_profile_dict.get("favorite_brands")
        valid_brands = collect_valid(brands, lambda b: b.lower() in self.valid_brands_lower_map,
                                    lambda b: self.valid_brands_lower_map[b.lower()])
        if valid_brands:
            collector["favorite_brands"] = valid_brands

        categories = raw_profile_dict.get("preferred_categories")
        valid_cats = collect_valid(categories, lambda c: c in self.valid_item_categories, lambda c: c)
        if valid_cats:
            collector["preferred_categories"] = valid_cats

        return collector


    def parse_feedback_to_profile_update(self, feedback_text, shop_name, item_categories_queried=None):
        if not self.client and not os.environ.get("CI_RUN"):
            print("Error: Groq client not initialized. Cannot parse feedback.")
            return {"error": "Groq client not initialized"}

        item_categories_queried_str = json.dumps(item_categories_queried if item_categories_queried else [])

        prompt = f"""
You are a profile update assistant. The user has just visited '{shop_name}' and provided feedback.
The user was initially looking for item categories: {item_categories_queried_str}.
Analyze the feedback to extract:
1. "request_fulfilled": Whether their primary shopping goal at this shop was fulfilled (true/false/null if unclear from text).
2. "rating": A sentiment-based rating for the shop (1-5, where 1 is very negative, 3 neutral, 5 very positive). Infer if not explicit. Default to null if uninferable.
3. "notes_positive": Specific things liked about the shop itself (e.g., "ambiance", "service", "layout", "good selection of X").
4. "notes_negative": Specific things disliked about the shop itself (e.g., "bit messy", "couldn't find Y", "poor service").
5. "item_feedback_updates": A list of objects. For each object, provide:
    - "category_name": The name of an item category from AVAILABLE ITEM CATEGORIES that the feedback pertains to.
    - "likes": A list of strings describing specific liked attributes, brands, or features for that item category.
    - "dislikes": A list of strings describing specific disliked attributes, brands, or features for that item category, or items they couldn't find.

User Feedback: "{feedback_text}"

AVAILABLE ITEM CATEGORIES: {self.valid_item_categories}
AVAILABLE BRANDS: {self.valid_brands}

Return STRICTLY VALID JSON with the following structure:
{{
  "shop_review_update": {{
    "request_fulfilled": true | false | null,
    "rating": 1 | 2 | 3 | 4 | 5 | null,
    "notes_positive": ["string"],
    "notes_negative": ["string"]
  }},
  "item_feedback_updates": [
    {{
      "category_name": "string from AVAILABLE ITEM CATEGORIES",
      "likes": ["string"],
      "dislikes": ["string"]
    }}
  ]
}}
If feedback is very generic (e.g., "it was okay"), try to infer at least a neutral rating (3).
If feedback is negative like "didn't find anything I wanted", set request_fulfilled to false.
If user says "found a great T-shirt", link "T-shirt" to "T-shirt/top" if that's the category.
Extract brand names if mentioned in likes/dislikes for items.
Be concise in notes. Empty lists are acceptable.

Output ONLY a single, valid JSON object. No markdown, no explanations.
"""
        try:
            if not self.client:
                raise ConnectionError("Groq client not available to make API call.")

            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant",
                temperature=0.2,
                max_tokens=600,
                response_format={"type": "json_object"}
            )
            raw_output = chat_completion.choices[0].message.content
            cleaned_output = raw_output.strip()
            if cleaned_output.startswith("```json"): cleaned_output = cleaned_output[7:]
            if cleaned_output.endswith("```"): cleaned_output = cleaned_output[:-3]
            cleaned_output = cleaned_output.strip()

            parsed_json = json.loads(cleaned_output)
            # Basic validation for feedback structure could be added here
            return parsed_json
        except json.JSONDecodeError as e:
            print(f"LLM Feedback JSON Decode Error: {e} in output: '{cleaned_output if 'cleaned_output' in locals() else raw_output if 'raw_output' in locals() else 'Unavailable'}'")
            return {"error": "JSON Decode Error", "details": str(e)}
        except Exception as e:
            print(f"Groq API or other LLM Feedback Error: {str(e)}")
            return {"error": "Groq API Error", "details": str(e)}


if __name__ == '__main__':
    print("Testing llm_groq_parser.py (V3 logic)...")
    parser = GroqQueryParser()
    # These lists would normally come from KnowledgeGraphService in main_robot_controller
    parser.set_knowledge_base_lists(
        store_names=["Urban Stylez", "Classic Comfort", "Footwear Palace", "Active & Street", "Kids Playworld"],
        item_categories=[c["name"] for c in fashion_mnist_classes],
        brands=fictional_brands_list,
        colors=colors_list
    )

    test_queries = [
        ("I need a shirt for my father. He prefers Classic Co.", "mother", None),
        ("I'm looking for a new coat for myself. I've started liking the color black.", "mother", None),
        ("My child prefers green trousers. I myself actually like red.", "mother", {"item_feedback": {}}), # profile_ctx is for child
        ("What are Mum's preferences?", "father", None),
        ("No more, thanks.", "mother", None)
    ]

    for query, speaker, profile_ctx_for_llm in test_queries:
        print(f"\nTest Query: \"{query}\" (Speaker: {speaker if speaker else 'Unknown'})")
        structured = parser.generate_structured_query(query,
                                                      current_speaker_role_if_known=speaker,
                                                      shopping_for_user_profile_context=profile_ctx_for_llm)
        print(f"Structured Output: {json.dumps(structured, indent=2)}")

    print("\nTesting feedback parsing...")
    feedback_text = "It was great, I found an amazing blue Dress from Glamora! The staff were helpful."
    shop = "Chic Boutique"
    items_sought = ["Dress"]
    parsed_feedback = parser.parse_feedback_to_profile_update(feedback_text, shop, items_sought)
    print(f"Parsed Feedback: {json.dumps(parsed_feedback, indent=2)}")