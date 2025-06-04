# automated_benchmarking_llm.py
import os
from groq import Groq
import json
import random
from dotenv import load_dotenv

load_dotenv()

class AutomatedBenchmarkingLLM:
    def __init__(self, persona="mother"):
        api_key = os.environ.get("GROQ_API_KEY")
        self.client = Groq(api_key=api_key) if api_key else None
        if not self.client and not os.environ.get("CI_RUN"):
            print("BENCHMARKING_LLM_WARNING: GROQ_API_KEY not found. LLM calls will be mocked or fail.")
        
        self.persona = persona
        self.current_shopping_goal_summary = None 
        self.kg_entities = { 
            "item_categories": [], "brands": [], "colors": [], "store_names": []
        }
        self.sample_products_from_kg = [] # New attribute for existing products
        self.last_item_found_at_store = False 

    def set_knowledge_base(self, item_categories, brands, colors, store_names, sample_products=None): # Added sample_products
        self.kg_entities["item_categories"] = item_categories
        self.kg_entities["brands"] = brands
        self.kg_entities["colors"] = colors
        self.kg_entities["store_names"] = store_names
        self.sample_products_from_kg = sample_products if sample_products else []
        print(f"BENCHMARKING_LLM: Knowledge base set. Persona: {self.persona}. Sample products: {len(self.sample_products_from_kg)}")


    def _call_groq_api_for_simulation(self, prompt, max_tokens=100):
        if not self.client:
            print("BENCHMARKING_LLM_ERROR: Groq client not initialized. Cannot make API call.")
            if "initial shopping query" in prompt.lower(): return "I need a new coat."
            if "who is this request for" in prompt.lower(): return f"It's for the {self.persona}." # Fallback if specific not hit
            if "shall we go there" in prompt.lower(): return "Yes, let's go."
            if "when you're ready" in prompt.lower(): return "Okay, I'm ready now."
            if "did you find what you were looking for" in prompt.lower(): return "Yes, I found a great one!"
            if "any other feedback" in prompt.lower(): return "The selection was good."
            if "what next" in prompt.lower(): return "Let's look for shoes now."
            return "Okay."

        try:
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant", 
                temperature=0.7, 
                max_tokens=max_tokens,
            )
            response_text = chat_completion.choices[0].message.content.strip()
            if response_text.startswith('"') and response_text.endswith('"'):
                response_text = response_text[1:-1]
            return response_text
        except Exception as e:
            print(f"BENCHMARKING_LLM_ERROR: Groq API error: {str(e)}")
            return "I'm not sure how to respond to that right now." 

    def generate_initial_speaker_id(self):
        return f"I am the {self.persona}."

    def generate_initial_query(self):
        # Determine shopping target persona first
        shopping_target_persona = self.persona 
        if random.random() < 0.4: 
            possible_targets = [r for r in ["mother", "father", "child"] if r != self.persona]
            if possible_targets:
                shopping_target_persona = random.choice(possible_targets)

        query_subject = "for myself"
        if shopping_target_persona == "child": query_subject = "for my child"
        elif shopping_target_persona == "father": query_subject = "for my father"
        elif shopping_target_persona == "mother" and self.persona != "mother": query_subject = "for my mother"

        # 90% chance to generate a query for an existing product if samples are available
        if self.sample_products_from_kg and random.random() < 0.9:
            product_data = random.choice(self.sample_products_from_kg)
            item_type = product_data.get('base_class_name')
            color = product_data.get('color')
            brand = product_data.get('brand')
            
            query_parts = []
            # Make the query slightly less specific sometimes
            if color and color != "Various" and random.random() < 0.8: query_parts.append(color)
            if brand and random.random() < 0.6: query_parts.append(brand)
            # Always include item type if from sample product
            if item_type: query_parts.append(item_type)
            else: # Fallback if item_type was somehow missing from sample
                item_type = random.choice(self.kg_entities["item_categories"]) if self.kg_entities["item_categories"] else "something nice"
                query_parts.append(item_type)
            
            self.current_shopping_goal_summary = f"{' '.join(query_parts)} {query_subject}"
        else: # 10% chance for original random query (might not exist, or if no samples)
            item_type = random.choice(self.kg_entities["item_categories"]) if self.kg_entities["item_categories"] else "something nice"
            color = random.choice(self.kg_entities["colors"]) if self.kg_entities["colors"] else ""
            brand = random.choice(self.kg_entities["brands"]) if self.kg_entities["brands"] and random.random() < 0.3 else "" 
            
            query_parts = []
            if color: query_parts.append(color)
            if brand: query_parts.append(brand)
            query_parts.append(item_type)
            self.current_shopping_goal_summary = f"{' '.join(query_parts)} {query_subject}"
        
        prompt = f"""You are impersonating a '{self.persona}' in a mall, talking to a robot assistant.
Generate a natural, concise initial shopping query.
You are thinking about: "{self.current_shopping_goal_summary}".
Examples:
- "I'm looking for a {random.choice(self.kg_entities.get("colors", ["blue"]))} {random.choice(self.kg_entities.get("item_categories", ["shirt"]))}."
- "Can you help me find a {random.choice(self.kg_entities.get("item_categories", ["toy"]))} for the child?"
- "Where can I find {random.choice(self.kg_entities.get("brands", ["GenericBrand"]))} shoes?"
- "I need to buy {random.choice(self.kg_entities.get("item_categories", ["a gift"]))}."

Your query (just the query, no preamble):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=50)

    def respond_to_clarify_shopping_target(self, robot_query_summary):
        intended_target_role = self.persona 
        if self.current_shopping_goal_summary: # Check the summary of the *current* query being clarified
            csl = self.current_shopping_goal_summary.lower()
            if "for my child" in csl or "for the child" in csl : intended_target_role = "child"
            elif "for my father" in csl or "for dad" in csl: intended_target_role = "father"
            elif "for my mother" in csl or "for mum" in csl: intended_target_role = "mother"
            elif "for myself" in csl or "for me" in csl: intended_target_role = self.persona
            # If current_shopping_goal_summary doesn't specify "for X", it implies it's for self.persona

        response_options = {
            "mother": [f"It's for Mother.", f"Mother.", f"For the mother."],
            "father": [f"It's for Father.", f"Father.", f"For the father."],
            "child":  [f"It's for the Child.", f"The Child.", f"For my child."]
        }
        
        specific_response_idea = f"It's for {intended_target_role.capitalize()}." # Default idea
        if intended_target_role in response_options:
            specific_response_idea = random.choice(response_options[intended_target_role])

        prompt = f"""You are impersonating a '{self.persona}'. The robot asked who the request '{robot_query_summary}' is for.
You internally know the request is intended for '{intended_target_role}'.
Your goal is to respond naturally but clearly stating the role (Mother, Father, or Child) so the robot can parse it.
You are thinking of saying something like: '{specific_response_idea}'.
Examples of good, clear responses: "For Mother.", "It's for the child.", "Father."

Your response (just the response, ensure it's parsable by a system looking for 'mother', 'father', or 'child'):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=20)

    def decide_to_visit_store(self, store_name_suggested, item_context_summary):
        decision_prompt = "Yes, let's go." if random.random() < 0.9 else "No, let's try another option first."
        prompt = f"""You are impersonating a '{self.persona}'.
You are looking for: '{item_context_summary}'.
The robot suggested visiting '{store_name_suggested}' and asked 'Shall we go there?'.
Respond with a short, natural confirmation or rejection like '{decision_prompt}'.

Your response (just 'Yes/No' or a short phrase):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=15)

    def acknowledge_arrival_and_readiness(self):
        self.last_item_found_at_store = False 
        prompt = f"""You are impersonating a '{self.persona}'.
The robot has 'navigated' to a store and said: "We've arrived at [Store Name]. Please look around. Let me know when you're ready."
Respond naturally that you've finished looking or are ready to give feedback.
Examples: "Okay, I'm ready.", "I've had a look.", "Finished, thanks."

Your response (short and natural):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=15)

    def provide_fulfillment_feedback(self, item_context_summary, store_name):
        found_it = random.random() < 0.6 
        self.last_item_found_at_store = found_it
        
        item_name_only = item_context_summary.split(' for ')[0] 
        if " " in item_name_only and len(item_name_only.split(' ')) > 2 : 
            item_name_only = " ".join(item_name_only.split(' ')[-2:]) 
        elif " " in item_name_only:
            item_name_only = item_name_only.split(' ')[-1] 


        response_if_found = f"Yes, I found a great {item_name_only}!"
        response_if_not_found = f"No, they didn't have the right {item_name_only}."
        actual_response_idea = response_if_found if found_it else response_if_not_found

        prompt = f"""You are impersonating a '{self.persona}'.
You were looking for '{item_context_summary}' at '{store_name}'.
The robot asked: "Regarding {item_context_summary}, did you find what you were looking for at {store_name}?"
You internally decided: '{actual_response_idea}'.
Respond naturally.

Your response (short and natural):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def provide_general_store_feedback(self, store_name):
        positive_feedback = ["The staff were helpful.", "Good selection here.", "Nice store layout.", "I liked the prices."]
        negative_feedback = ["It was a bit messy.", "Couldn't find staff easily.", "Selection was limited for what I wanted.", "A bit too crowded."]
        
        if self.last_item_found_at_store and random.random() < 0.8: 
            feedback_idea = random.choice(positive_feedback)
        elif not self.last_item_found_at_store and random.random() < 0.7:
            feedback_idea = random.choice(negative_feedback)
        else:
            feedback_idea = "No particular feedback right now."

        prompt = f"""You are impersonating a '{self.persona}'.
You just visited '{store_name}'. The robot asked for general feedback or to say 'no feedback'.
You are thinking: '{feedback_idea}'.
Respond naturally.

Your response (short and natural, or 'no feedback'):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def decide_next_action(self, original_request_summary, was_last_item_fulfilled, has_more_options_for_current):
        action_choice = "stop"
        new_query_text_for_llm = None

        if was_last_item_fulfilled: 
            if random.random() < 0.6: 
                action_choice = "stop"
            else: 
                action_choice = "new_request"
        else: 
            if has_more_options_for_current and random.random() < 0.7: 
                action_choice = "continue_current_request"
            elif random.random() < 0.5: 
                action_choice = "new_request"
            else: 
                action_choice = "stop"

        if action_choice == "new_request":
            # When starting a new request, try to pick an existing item 70% of the time if samples available
            if self.sample_products_from_kg and random.random() < 0.7:
                new_product_sample = random.choice(self.sample_products_from_kg)
                new_item = new_product_sample.get('base_class_name', "something else")
                # Could add color/brand to make it more specific, but simple item type is fine for "what next"
            else: # Fallback to general category
                new_item = random.choice(self.kg_entities["item_categories"]) if self.kg_entities["item_categories"] else "something else"
            
            new_query_text_for_llm = f"Okay, now I'm looking for a {new_item}."
            self.current_shopping_goal_summary = new_item # Update goal for next potential clarification
        
        fulfillment_str = "was fulfilled" if was_last_item_fulfilled else "was not fulfilled"
        options_str = "there are more options" if has_more_options_for_current else "no more options"

        prompt = f"""You are impersonating a '{self.persona}'.
Your previous request was for '{original_request_summary}', which {fulfillment_str}.
For that previous request, {options_str}.
The robot asked what you want to do next (e.g., other suggestions, new request, or stop).

You have decided to: '{action_choice}'.
If starting a new request, you're thinking: '{new_query_text_for_llm if new_query_text_for_llm else ""}'.

Formulate a natural language response based on your decision.
Examples:
- (stop) "That's all for now, thanks."
- (continue) "Yes, what other options do you have for {original_request_summary.split(' for ')[0]}?"
- (new_request) "Okay, let's look for {random.choice(self.kg_entities.get("item_categories", ["a new item"]))} now."

Your response (short and natural):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=40)

    def respond_to_anything_else(self):
        if random.random() < 0.1: 
            # When starting a new request here, try to pick an existing item 70% of the time if samples available
            if self.sample_products_from_kg and random.random() < 0.7:
                new_product_sample = random.choice(self.sample_products_from_kg)
                new_item = new_product_sample.get('base_class_name', "a small thing")
            else: # Fallback to general category
                new_item = random.choice(self.kg_entities["item_categories"]) if self.kg_entities["item_categories"] else "a small thing"

            self.current_shopping_goal_summary = new_item # Update goal for next potential clarification
            return f"Actually, yes, I also need {new_item}."
        else:
            return "No, that's everything. Thank you!"