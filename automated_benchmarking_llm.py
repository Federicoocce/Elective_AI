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
        self.kg_entities = { 
            "item_categories": [], "brands": [], "colors": [], "store_names": []
        }
        self.sample_products_from_kg = []
        
        self.shopping_list = [] 
        self.current_target_item_index = -1
        self.revealed_details = {} 
        self.favorite_stores_for_this_session = []
        self.last_item_found_at_store = False 
        self.current_shopping_goal_summary = "" # General summary of what user is vague about
        self.interaction_count = 0
        self.found_items = 0 # Number of items actually found from the shopping list
        self.success = False # Overall success for this session (all items on list found)
    
    def get_session_metrics(self):
        # Success is true if all items in the (now single-item) list were found
        if self.shopping_list:
            self.success = all(item.get('found', False) for item in self.shopping_list)
            self.found_items = sum(1 for item in self.shopping_list if item.get('found', False))
        else: # No shopping list means no items to find, so not a success in terms of task completion
            self.success = False
            self.found_items = 0

        return {
            "interaction_count": self.interaction_count,
            "found_items": self.found_items, # This will be 0 or 1 for single item lists
            "success": self.success
        }

    def _get_current_target_product(self):
        if 0 <= self.current_target_item_index < len(self.shopping_list):
            return self.shopping_list[self.current_target_item_index]
        return None

    def _initialize_shopping_session_goals(self):
        self.shopping_list = []
        self.current_target_item_index = -1
        self.revealed_details = {}
        self.favorite_stores_for_this_session = []
        self.found_items = 0
        self.success = False
        self.interaction_count = 0 # Reset interaction count for the new session

        if not self.sample_products_from_kg: return

        num_items_to_shop_for = 1 # Force one item per session for focused benchmarking
        
        if not self.sample_products_from_kg: return
        if len(self.sample_products_from_kg) < num_items_to_shop_for :
            print(f"BENCHMARKING_LLM_WARNING: Not enough sample products ({len(self.sample_products_from_kg)}) to pick {num_items_to_shop_for}. Skipping list generation.")
            return

        if num_items_to_shop_for == 0: return # Cannot make a shopping list

        potential_targets = random.sample(self.sample_products_from_kg, k=num_items_to_shop_for)
        
        for product_data in potential_targets:
            self.shopping_list.append({
                "product_name": product_data.get('name'), 
                "category": product_data.get('base_class_name'),
                "color": product_data.get('color'),
                "brand": product_data.get('brand'),
                "found": False
            })
        
        if self.shopping_list:
            self.current_target_item_index = 0
            self._prepare_revealed_details_for_current_target()

        if self.kg_entities["store_names"]:
            num_fav_stores = random.randint(0, 1) # Reduced for simplicity
            if num_fav_stores > 0 and len(self.kg_entities["store_names"]) >= num_fav_stores:
                self.favorite_stores_for_this_session = random.sample(self.kg_entities["store_names"], k=num_fav_stores)
        
        print(f"BENCHMARKING_LLM_SESSION_GOALS: Persona: {self.persona}")
        print(f"  Shopping List ({len(self.shopping_list)} item):") # Singular item
        for i, item in enumerate(self.shopping_list):
            print(f"    {i+1}. {item['category']} (Color: {item['color']}, Brand: {item['brand']}) - Target Name: '{item['product_name']}'")
        print(f"  Favorite Stores for this session: {self.favorite_stores_for_this_session}")


    def _prepare_revealed_details_for_current_target(self):
        self.revealed_details = {} 
        target_product = self._get_current_target_product()
        if target_product:
            self.revealed_details['category'] = target_product['category']
            # Make it slightly more likely to reveal color/brand if they are specific
            if target_product['color'] and target_product['color'] != "Various" and random.random() < 0.4:
                self.revealed_details['color'] = target_product['color']
            if target_product['brand'] and random.random() < 0.3:
                 self.revealed_details['brand'] = target_product['brand']


    def set_knowledge_base(self, item_categories, brands, colors, store_names, sample_products=None):
        self.kg_entities["item_categories"] = item_categories
        self.kg_entities["brands"] = brands
        self.kg_entities["colors"] = colors
        self.kg_entities["store_names"] = store_names
        self.sample_products_from_kg = sample_products if sample_products else []
        print(f"BENCHMARKING_LLM: Knowledge base set for persona {self.persona}. Sample products available: {len(self.sample_products_from_kg)}")
        self._initialize_shopping_session_goals() # This will reset and create a new 1-item list

    def _call_groq_api_for_simulation(self, prompt, max_tokens=100):
        self.interaction_count += 1
        if not self.client:
            print(f"BENCHMARKING_LLM_ERROR: Groq client not initialized. Mocking LLM call for prompt starting with: '{prompt[:50]}...'")
            target_product = self._get_current_target_product()
            
            if "initial shopping query" in prompt:
                return f"I'm thinking about getting a {self.revealed_details.get('category', 'new item')}."
            if "who is this request for" in prompt:
                return f"It's for {self.persona}." # Mock directness
            if "shall we go there" in prompt:
                return "Yes, let's go." if random.random() < 0.85 else "No, not that one."
            if "when you're ready" in prompt:
                return random.choice(["Okay, I'm ready.", "I've had a look."])
            if "did you find what you were looking for" in prompt:
                if target_product and random.random() < 0.6: 
                    self.last_item_found_at_store = True
                    target_product['found'] = True
                    self.found_items = 1 # Since only one item
                    self.success = True
                    return f"Yes, I found a great {target_product['category']}!"
                else:
                    self.last_item_found_at_store = False
                    if target_product: target_product['found'] = False
                    self.found_items = 0
                    self.success = False
                    return f"No, not quite the {target_product['category'] if target_product else 'item'} I wanted."
            if "any other feedback" in prompt:
                if self.last_item_found_at_store: return "It was a good find!"
                else: return "The selection wasn't what I hoped for regarding the item."
            if "what next" in prompt: # For single item, this should lead to stopping
                return "That's all for now, thanks!"
            if "anything else" in prompt: # For single item, this is the end
                 return "No, that's everything. Thank you!"

            return "Okay." 

        try:
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant", temperature=0.6, max_tokens=max_tokens,
            )
            response_text = chat_completion.choices[0].message.content.strip().replace('"', '')
            return response_text
        except Exception as e:
            print(f"BENCHMARKING_LLM_ERROR: Groq API error: {str(e)}")
            return "I'm not sure how to respond to that right now."


    def generate_initial_speaker_id(self):
        return f"I am the {self.persona}."

    def generate_initial_query(self):
        target_product = self._get_current_target_product()
        if not target_product:
            self.current_shopping_goal_summary = "something nice for myself"
            query_idea = "I'm looking for something nice."
        else:
            # Initial query should be about the category, possibly color, but less likely brand.
            query_parts = [self.revealed_details['category']]
            if 'color' in self.revealed_details: query_parts.insert(0, self.revealed_details['color'])
            
            self.current_shopping_goal_summary = f"a {target_product['category']} for {self.persona}"
            if 'color' in self.revealed_details : self.current_shopping_goal_summary = f"a {self.revealed_details['color']} {target_product['category']} for {self.persona}"
            
            if 'color' in self.revealed_details:
                query_idea = f"I need a {self.revealed_details['color']} {self.revealed_details['category']}."
            else: # Only category revealed
                query_idea = f"I'm thinking about getting a {self.revealed_details['category']}."
                if random.random() < 0.3: query_idea = f"Can you help me find a {self.revealed_details['category']}?"
        
        prompt = f"""You are impersonating a '{self.persona}' who is starting to shop for one item.
Your current shopping goal is: "{self.current_shopping_goal_summary}".
Formulate a natural, slightly vague initial shopping query based on this idea. Don't reveal all details yet.
Examples:
- "I'm looking for a new coat."
- "I need some red shoes."
- "Where can I find T-shirts?"

Your query (just the query, no preamble):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def respond_to_clarify_shopping_target(self, robot_query_summary):
        prompt = f"""You are impersonating a '{self.persona}'. The robot asked who the request '{robot_query_summary}' is for.
You intend it for yourself ({self.persona}).
Respond NATURALLY and BRIEFLY, stating it's for you.
Examples: "For me.", "It's for myself.", "This one's for me."
Do NOT add extra terms like 'son', 'dear', etc. Just a direct statement about it being for you.

Your response (just the response, e.g., "For me."):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=10)

    def decide_to_visit_store(self, store_name_suggested, item_context_summary_from_robot):
        target_product = self._get_current_target_product()
        decision_reason = "general interest"
        agree_prob = 0.6 

        if store_name_suggested in self.favorite_stores_for_this_session:
            agree_prob = 0.95
            decision_reason = f"it's a favorite store ({store_name_suggested})"
        elif target_product and item_context_summary_from_robot:
            robot_suggestion_lower = item_context_summary_from_robot.lower()
            matches_category = target_product['category'].lower() in robot_suggestion_lower
            
            is_direct_match_suggestion = "matching item(s) at" in robot_suggestion_lower or \
                                         ("found" in robot_suggestion_lower and " at " in robot_suggestion_lower)

            if is_direct_match_suggestion and matches_category:
                agree_prob = 0.98
                decision_reason = "robot seems to have found items in the right category"
            elif matches_category and ("potential place" in robot_suggestion_lower):
                 agree_prob = 0.85
                 decision_reason = "robot suggested a relevant category store"
            elif matches_category : 
                 agree_prob = 0.75
                 decision_reason = "store might have the right category"

        final_decision_is_yes = random.random() < agree_prob
        decision_prompt_idea = "Yes, let's go." if final_decision_is_yes else "No, I'm not feeling that one. Any other ideas?"
        
        current_target_display = target_product['category'] if target_product else 'something'
        revealed_display = str(self.revealed_details)

        print(f"BENCHMARKING_LLM_DECISION: Visiting {store_name_suggested}? Current Target: {current_target_display}. Revealed: {revealed_display}. Robot said: '{item_context_summary_from_robot}'. Reason: {decision_reason}. Agree Prob: {agree_prob:.2f}. Decision: {'Yes' if final_decision_is_yes else 'No'}")

        prompt = f"""You are impersonating a '{self.persona}'.
You are generally looking for '{current_target_display}' and have possibly revealed: {revealed_display}.
The robot suggested visiting '{store_name_suggested}' based on its understanding: '{item_context_summary_from_robot}'.
You internally decided: '{decision_prompt_idea}' because of '{decision_reason}'.
Respond with your decision, keeping it concise.

Your response (short phrase reflecting your decision):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=15)

    def acknowledge_arrival_and_readiness(self):
        self.last_item_found_at_store = False
        responses = ["Okay, I'm ready.", "I've had a look.", "Finished looking.", "Ready when you are."]
        prompt = f"""You are '{self.persona}'. The robot said it arrived and asked you to let it know when you are ready.
You are now ready. Respond with a short, natural confirmation.
Examples: "Okay, I'm ready.", "I've had a look."

Your response:"""
        return random.choice(responses) if not self.client else self._call_groq_api_for_simulation(prompt, max_tokens=10)


    def provide_fulfillment_feedback(self, item_context_summary_from_robot, store_name):
        target_product = self._get_current_target_product()
        found_it_for_real = False
        reason_for_not_finding = "it just wasn't right."

        if not target_product:
            self.success = False # No target, no success
            return self._call_groq_api_for_simulation("I wasn't looking for anything specific, but thanks.", max_tokens=20)
        
        # Simulate if the store had the *specific* item
        if target_product['category'].lower() in item_context_summary_from_robot.lower():
            if random.random() < 0.65: # Higher chance to "find" if category is right
                found_it_for_real = True
                self.revealed_details['color'] = target_product['color']
                self.revealed_details['brand'] = target_product['brand']
            else:
                if target_product['color'] and target_product['color'] != "Various" and self.revealed_details.get('color') != target_product['color']:
                    reason_for_not_finding = f"they didn't have it in {target_product['color']}."
                    if random.random() < 0.7: self.revealed_details['color'] = target_product['color'] 
                elif target_product['brand'] and self.revealed_details.get('brand') != target_product['brand']:
                    reason_for_not_finding = f"I was hoping for the {target_product['brand']} brand."
                    if random.random() < 0.7: self.revealed_details['brand'] = target_product['brand']
                else:
                    reason_for_not_finding = "the style wasn't what I wanted."
        else: 
            reason_for_not_finding = f"they didn't seem to have the type of {target_product['category']} I was after."

        self.last_item_found_at_store = found_it_for_real
        if found_it_for_real:
            target_product['found'] = True 
            self.found_items = 1 # Only one item on list
            self.success = True
            response_idea = f"Yes! I found the perfect {target_product['category']}!"
            if random.random() < 0.3: response_idea = f"Yes, this {target_product['product_name']} is great!"
        else:
            target_product['found'] = False
            self.found_items = 0
            self.success = False
            response_idea = f"No, unfortunately. {reason_for_not_finding}"
            if random.random() < 0.3: response_idea = f"Hmm, not quite. {reason_for_not_finding}"

        print(f"BENCHMARKING_LLM_FULFILLMENT: Target: {target_product['category']} ('{target_product['product_name']}'). Revealed: {self.revealed_details}. Robot context: '{item_context_summary_from_robot}'. Found specific item: {found_it_for_real}. Response: '{response_idea}'")

        prompt = f"""You are '{self.persona}'. You were secretly hoping to find '{target_product['product_name']}' (a {target_product['category']}).
The robot helped you look based on its understanding: '{item_context_summary_from_robot}' at '{store_name}'.
The robot asks: "...did you find what you were looking for...?"
You internally decided: '{response_idea}'. (This reflects if you found your secret item).
Respond naturally, consistent with your decision. If you found it, be happy. If not, explain briefly as per your decision.

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=40)

    def provide_general_store_feedback(self, store_name):
        target_product = self._get_current_target_product()
        feedback_idea = "It was okay."

        if store_name in self.favorite_stores_for_this_session:
            feedback_idea = random.choice([f"I generally like {store_name}.", "It's one of my preferred stores."])
        elif self.last_item_found_at_store and target_product: 
            feedback_idea = random.choice([f"So glad I found my {target_product['category']} here!", f"Excellent, they had the {target_product['product_name']}!"])
        else: 
            if target_product: 
                 feedback_idea = random.choice([
                    f"They didn't have the specific {target_product['category']} I was looking for, unfortunately.",
                    f"The selection for {target_product['category']}s wasn't quite right for me today."
                ])
            else: # Should not happen if target_product always exists
                feedback_idea = "Not very impressed with the selection for what I had in mind."
        
        print(f"BENCHMARKING_LLM_STORE_FEEDBACK: Store: {store_name}. Fav?: {store_name in self.favorite_stores_for_this_session}. Item found here?: {self.last_item_found_at_store}. Feedback idea: '{feedback_idea}'")

        prompt = f"""You are '{self.persona}'. You just visited '{store_name}'.
The robot asked for general feedback about the store.
Your internal thought, based on whether you found your item ('{target_product['product_name'] if target_product else 'item'}') and if it's a favorite store, is: '{feedback_idea}'.
Respond naturally, reflecting this thought.

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def _move_to_next_target_item_or_stop(self): # This is mostly for internal state, decision logic is in decide_next_action
        current_target = self._get_current_target_product()
        if current_target and current_target['found']:
            # For single item list, this means all items found
            return "all_items_found" 
        # If not found, or no list (should not happen with 1 item list unless error)
        return "current_item_not_yet_found_or_no_list"


    def decide_next_action(self, original_request_summary_from_robot, was_last_item_fulfilled_by_robot, has_more_options_for_current_robot_request):
        # For single item shopping list, this will almost always lead to "stop" the session.
        action_choice = "stop"
        instruction_for_llm = "You've decided to stop shopping for now."
        example_response = "That's all for now, thanks."
        current_target_product = self._get_current_target_product()
        current_target_display = current_target_product['category'] if current_target_product else 'item'

        if current_target_product:
            if current_target_product['found']: # Item was found
                action_choice = "stop"
                instruction_for_llm = f"You found your '{current_target_display}'. You'll stop the shopping session now."
                example_response = "That's great, I found what I needed! Thanks."
                self.success = True # Re-affirm
            else: # Item not found
                if has_more_options_for_current_robot_request and random.random() < 0.6: # Chance to try again if options exist
                    action_choice = "continue_current_request"
                    instruction_for_llm = f"You haven't found your '{current_target_display}' yet. The robot has more options for '{original_request_summary_from_robot}'. You'll ask to see them."
                    example_response = f"Okay, what other suggestions do you have for the {current_target_display.lower()}?"
                else: # No more options, or decided to stop anyway
                    action_choice = "stop"
                    instruction_for_llm = f"You haven't found your '{current_target_display}' and decided to stop looking for it now."
                    example_response = "Alright, let's not worry about that one for today then. Thanks."
                    self.success = False # Item not found
        else: # No current target (should not happen in normal flow with 1 item list)
            action_choice = "stop"
            instruction_for_llm = "You have no active shopping goal, so you'll stop."
            example_response = "I think that's all, thanks."
            self.success = False

        # `was_last_item_fulfilled_by_robot` is the robot's perception from LLM parser,
        # `self.last_item_found_at_store` is the AutoLLMUser's internal "truth".
        # `next_move_signal` is less relevant for single item, action_choice above is key.
        next_move_signal = self._move_to_next_target_item_or_stop() # Update internal state for get_metrics

        print(f"BENCHMARKING_LLM_NEXT_ACTION: Prev robot request: '{original_request_summary_from_robot}'. Robot thinks fulfilled: {was_last_item_fulfilled_by_robot}. User truth (last store): {self.last_item_found_at_store}. Next move signal: {next_move_signal}. Chosen action: {action_choice}. Instruction: '{instruction_for_llm}'")

        prompt = f"""You are '{self.persona}'.
The robot's last search was for '{original_request_summary_from_robot}'.
Internally, your decision based on whether you found your item and if more options exist is: {instruction_for_llm}

Formulate a natural language response that clearly conveys ONLY this decision.
Example if you decided similarly: "{example_response}"

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=40)


    def respond_to_anything_else(self):
        # This is called when a sub-dialog (like finding one item) concludes.
        # For a single-item shopping list, this always means the session ends.
        current_target = self._get_current_target_product()
        final_message = "No, that's everything. Thank you!"

        if current_target:
            if current_target['found']:
                self.success = True # Ensure success is set if item found
                self.found_items = 1
                final_message = "No, that's all I needed. Thanks for your help!"
                print(f"BENCHMARKING_LLM_ANYTHING_ELSE: Single item '{current_target['category']}' was found. Ending session.")
            else:
                self.success = False # Ensure success is false if item not found
                self.found_items = 0
                final_message = "No, I think I'll stop looking for that now. Thanks anyway!"
                print(f"BENCHMARKING_LLM_ANYTHING_ELSE: Single item '{current_target['category']}' was not found or given up on. Ending session.")
        else:
             self.success = False
             self.found_items = 0
             print(f"BENCHMARKING_LLM_ANYTHING_ELSE: No target item. Ending session.")
        
        return final_message