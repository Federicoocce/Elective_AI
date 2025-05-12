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
        self.valid_colors = [c.lower() for c in colors_list]
        self.valid_brands = [b.lower() for b in fictional_brands_list]
        self.valid_categories = list({c["name"] for c in fashion_mnist_classes})
        self.store_names = []

    def set_store_names(self, store_names):
        self.store_names = [s.lower() for s in store_names]

    def generate_structured_query(self, user_query):
        prompt = f"""Analyze this mall query and return STRICTLY VALID JSON with:
- intent (find_product/find_store)
- attributes (color, brand)
- item_types (from {self.valid_categories})
- store_name (from {self.store_names})

RULES:
1. Use double quotes ONLY
2. Never use markdown
3. Store names must match exactly: {self.store_names}

Examples:
Query: "Find Sport & Street store"
Output: {{"intent": "find_store", "store_name": "Active & Street"}}

Query: "Where to buy a leather bag?"
Output: {{"intent": "find_product", "attributes": {{}}, "item_types": ["Bag"]}}

Current Query: "{user_query}"
Respond ONLY with valid JSON:"""

        try:
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant",
                temperature=0.2,
                max_tokens=300,
                response_format={"type": "json_object"}
            )
            
            raw_output = chat_completion.choices[0].message.content
            cleaned_output = raw_output.strip() \
                .replace('```json', '') \
                .replace('```', '') \
                .replace("'", '"')  # Fix single quotes
            
            return self._validate_output(json.loads(cleaned_output))
            
        except json.JSONDecodeError as e:
            print(f"JSON Error in: {cleaned_output}")
            return None
        except Exception as e:
            print(f"Groq Error: {str(e)}")
            return None

    def _validate_output(self, output):
        validated = {
            "intent": output.get("intent", "find_product"),
            "attributes": {},
            "item_types": [],
            "store_name": None
        }
        print(f"Validating output: {output}")
        # Validate attributes
        if isinstance(output.get("attributes"), dict):
            # Color validation
            color = output["attributes"].get("color", "")
            if isinstance(color, str) and color.lower() in self.valid_colors:
                validated["attributes"]["color"] = color.capitalize()
            
            # Brand validation
            brand = output["attributes"].get("brand", "")
            if isinstance(brand, str) and brand.lower() in self.valid_brands:
                validated["attributes"]["brand"] = brand.capitalize()

        # Validate item types
        item_types = output.get("item_types", [])
        if isinstance(item_types, list):
            validated["item_types"] = [
                t for t in item_types 
                if isinstance(t, str) and t in self.valid_categories
            ]

        # Validate store name
        store_name = output.get("store_name", "")
        if isinstance(store_name, str):
            store_lower = store_name.lower()
            matched_store = next((s for s in self.store_names if s.lower() == store_lower), None)
            if matched_store:
                validated["store_name"] = matched_store

        return validated