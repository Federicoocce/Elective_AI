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

    def _get_current_target_product(self):
        if 0 <= self.current_target_item_index < len(self.shopping_list):
            return self.shopping_list[self.current_target_item_index]
        return None

    def _initialize_shopping_session_goals(self):
        self.shopping_list = []
        self.current_target_item_index = -1
        self.revealed_details = {}
        self.favorite_stores_for_this_session = []

        if not self.sample_products_from_kg: return

        num_items_to_shop_for = random.randint(1, 2)
        if len(self.sample_products_from_kg) < num_items_to_shop_for:
            num_items_to_shop_for = len(self.sample_products_from_kg)
        
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
            num_fav_stores = random.randint(0, 2)
            if num_fav_stores > 0 and len(self.kg_entities["store_names"]) >= num_fav_stores:
                self.favorite_stores_for_this_session = random.sample(self.kg_entities["store_names"], k=num_fav_stores)
        
        print(f"BENCHMARKING_LLM_SESSION_GOALS: Persona: {self.persona}")
        print(f"  Shopping List ({len(self.shopping_list)} items):")
        for i, item in enumerate(self.shopping_list):
            print(f"    {i+1}. {item['category']} (Color: {item['color']}, Brand: {item['brand']}) - Target Name: '{item['product_name']}'")
        print(f"  Favorite Stores for this session: {self.favorite_stores_for_this_session}")


    def _prepare_revealed_details_for_current_target(self):
        self.revealed_details = {} 
        target_product = self._get_current_target_product()
        if target_product:
            self.revealed_details['category'] = target_product['category']
            if random.random() < 0.3 and target_product['color'] and target_product['color'] != "Various":
                self.revealed_details['color'] = target_product['color']
            if random.random() < 0.2 and target_product['brand']:
                 self.revealed_details['brand'] = target_product['brand']


    def set_knowledge_base(self, item_categories, brands, colors, store_names, sample_products=None):
        self.kg_entities["item_categories"] = item_categories
        self.kg_entities["brands"] = brands
        self.kg_entities["colors"] = colors
        self.kg_entities["store_names"] = store_names
        self.sample_products_from_kg = sample_products if sample_products else []
        print(f"BENCHMARKING_LLM: Knowledge base set for persona {self.persona}. Sample products available: {len(self.sample_products_from_kg)}")
        self._initialize_shopping_session_goals()

    def _call_groq_api_for_simulation(self, prompt, max_tokens=100):
        if not self.client:
            # This mock section needs to be smarter or removed if relying purely on Groq when available.
            # For now, it's a very basic fallback.
            print(f"BENCHMARKING_LLM_ERROR: Groq client not initialized. Mocking LLM call for prompt starting with: '{prompt[:50]}...'")
            target_product = self._get_current_target_product()
            
            if "initial shopping query" in prompt:
                return f"I'm thinking about getting some {self.revealed_details.get('category', 'new clothes')}."
            if "who is this request for" in prompt:
                return f"It's for {self.persona}."
            if "shall we go there" in prompt: # Simplified for mock
                return "Yes, let's go." if random.random() < 0.85 else "Hmm, maybe not that one."
            if "when you're ready" in prompt:
                return random.choice(["Okay, I'm ready.", "I've had a look.", "Finished looking."])
            if "did you find what you were looking for" in prompt:
                # Mock fulfillment based on a chance if target_product exists
                if target_product and random.random() < 0.6: # 60% chance to "find" the generic category
                    self.last_item_found_at_store = True
                    target_product['found'] = True 
                    return f"Yes, I found a great {target_product['category']}!"
                else:
                    self.last_item_found_at_store = False
                    return f"No, not quite the {target_product['category'] if target_product else 'item'} I wanted."
            if "any other feedback" in prompt:
                if self.last_item_found_at_store: return "It was a good find!"
                else: return "The selection wasn't what I hoped for."
            if "what next" in prompt:
                # Simulate _move_to_next_target_item_or_stop logic for mock
                if target_product and target_product['found']:
                    # Check if there's a next item
                    next_idx = self.current_target_item_index + 1
                    if next_idx < len(self.shopping_list):
                        return f"Now I'm thinking about a {self.shopping_list[next_idx]['category']}."
                    else:
                        return "That's all for now, thanks!"
                else: # Current item not found or no items
                    return "Okay, let's stop looking for that then."
            if "anything else" in prompt:
                 return "No, that's everything. Thank you!"

            return "Okay, I understand." # Generic fallback for unhandled mock cases

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
            query_parts = [self.revealed_details['category']]
            if 'color' in self.revealed_details: query_parts.insert(0, self.revealed_details['color'])
            # Do not reveal brand in initial query to be more vague
            # if 'brand' in self.revealed_details: query_parts.insert(0, self.revealed_details['brand'])
            
            self.current_shopping_goal_summary = f"{' '.join(query_parts)} for {self.persona}"
            
            # Make initial query more natural and vague
            if 'color' in self.revealed_details and 'brand' in self.revealed_details:
                query_idea = f"I'm looking for a {self.revealed_details['color']} {self.revealed_details['brand']} {self.revealed_details['category']}."
            elif 'color' in self.revealed_details:
                query_idea = f"I need a {self.revealed_details['color']} {self.revealed_details['category']}."
            elif 'brand' in self.revealed_details:
                 query_idea = f"I'm trying to find a {self.revealed_details['brand']} {self.revealed_details['category']}."
            else: # Only category revealed
                query_idea = f"I'm thinking about getting some {self.revealed_details['category']}."
                if random.random() < 0.3: query_idea = f"Can you help me find {self.revealed_details['category']}?"
                elif random.random() < 0.3: query_idea = f"Where can I look for {self.revealed_details['category']}?"
        
        prompt = f"""You are impersonating a '{self.persona}' who is starting to shop.
You have a general idea: "{self.current_shopping_goal_summary}".
Formulate a natural, slightly vague initial shopping query based on this idea. Don't reveal all details if your idea is specific.
Examples of how you might phrase your thought:
- "I'm looking for some new clothes." (If thinking about a shirt)
- "I need a new coat." (If thinking about a specific coat)
- "Where are the shoe stores?" (If thinking about shoes)

Your query (just the query, no preamble):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def respond_to_clarify_shopping_target(self, robot_query_summary):
        prompt = f"""You are impersonating a '{self.persona}'. The robot asked who the request '{robot_query_summary}' is for.
You intend it for yourself ({self.persona}).
Respond naturally. Examples: "For me.", "It's for myself.", "This one's for me."

Your response (just the response):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=15)

    def decide_to_visit_store(self, store_name_suggested, item_context_summary_from_robot):
        target_product = self._get_current_target_product()
        decision_reason = "general interest"
        agree_prob = 0.6 

        if store_name_suggested in self.favorite_stores_for_this_session:
            agree_prob = 0.95
            decision_reason = f"it's a favorite store ({store_name_suggested})"
        elif target_product and item_context_summary_from_robot:
            robot_suggestion_lower = item_context_summary_from_robot.lower()
            # Check if robot's suggestion at least mentions the category user is targeting
            matches_category = target_product['category'].lower() in robot_suggestion_lower
            
            # More specific check: if robot says it "found X matching items"
            is_direct_match_suggestion = "matching item(s) at" in robot_suggestion_lower or \
                                         ("found" in robot_suggestion_lower and " at " in robot_suggestion_lower)

            if is_direct_match_suggestion and matches_category:
                agree_prob = 0.98
                decision_reason = "robot seems to have found items in the right category"
            elif matches_category and ("potential place" in robot_suggestion_lower):
                 agree_prob = 0.85
                 decision_reason = "robot suggested a relevant category store"
            elif matches_category : # Robot mentions category, but not strongly as a find
                 agree_prob = 0.75
                 decision_reason = "store might have the right category"


        final_decision_is_yes = random.random() < agree_prob
        decision_prompt = "Yes, let's go." if final_decision_is_yes else "No, I'm not feeling that one. Any other ideas?"
        
        current_target_display = target_product['category'] if target_product else 'something'
        revealed_display = str(self.revealed_details)

        print(f"BENCHMARKING_LLM_DECISION: Visiting {store_name_suggested}? Current Target: {current_target_display}. Revealed: {revealed_display}. Robot said: '{item_context_summary_from_robot}'. Reason: {decision_reason}. Agree Prob: {agree_prob:.2f}. Decision: {'Yes' if final_decision_is_yes else 'No'}")

        prompt = f"""You are impersonating a '{self.persona}'.
You are generally looking for '{current_target_display}' and have possibly revealed: {revealed_display}.
The robot suggested visiting '{store_name_suggested}' based on its understanding: '{item_context_summary_from_robot}'.
You internally decided: '{decision_prompt}' because of '{decision_reason}'.
Respond with your decision.

Your response (short phrase reflecting your decision):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=15)

    def acknowledge_arrival_and_readiness(self):
        self.last_item_found_at_store = False
        responses = ["Okay, I'm ready.", "I've had a look.", "Finished looking.", "Ready when you are.", "Alright, done looking."]
        prompt = f"""You are '{self.persona}'. The robot said it arrived and asked you to let it know when you are ready.
You are now ready. Respond with a short, natural confirmation.
Examples: "Okay, I'm ready.", "I've had a look.", "Finished."

Your response:"""
        return random.choice(responses) if not self.client else self._call_groq_api_for_simulation(prompt, max_tokens=10)


    def provide_fulfillment_feedback(self, item_context_summary_from_robot, store_name):
        target_product = self._get_current_target_product()
        found_it_for_real = False
        reason_for_not_finding = "it just wasn't right."

        if not target_product:
            return self._call_groq_api_for_simulation("I wasn't looking for anything specific, but thanks.", max_tokens=20)
        
        # Simulate if the store had the *specific* item the user secretly wants
        # This depends on whether the robot's general search led to a store that stocks it.
        # For this simulation, let's assume if the category is right, there's a chance.
        # The robot's `item_context_summary_from_robot` tells us what *it* thinks it found.
        # The user compares this to their internal specific `target_product`.
        
        # A very simplified check: does the robot's summary contain the target product's actual name?
        # This is a high bar. A more realistic check would be if the store *could* have the item.
        # Let's make it probabilistic based on category match.
        if target_product['category'].lower() in item_context_summary_from_robot.lower():
            # If the robot is on the right track (category), check if the specific item is "found"
            if random.random() < 0.6: # 60% chance user "finds" their specific hidden item
                found_it_for_real = True
                # Reveal more details now that it's "found"
                self.revealed_details['color'] = target_product['color']
                self.revealed_details['brand'] = target_product['brand']
            else:
                # Did not find the *specific* one, even if category was right. Reveal a new detail.
                if target_product['color'] and target_product['color'] != "Various" and self.revealed_details.get('color') != target_product['color']:
                    reason_for_not_finding = f"they didn't have it in {target_product['color']}."
                    if random.random() < 0.7: self.revealed_details['color'] = target_product['color'] 
                elif target_product['brand'] and self.revealed_details.get('brand') != target_product['brand']:
                    reason_for_not_finding = f"I was hoping for the {target_product['brand']} brand."
                    if random.random() < 0.7: self.revealed_details['brand'] = target_product['brand']
                else:
                    reason_for_not_finding = "the style wasn't what I wanted."
        else: # Robot was off-track with category
            reason_for_not_finding = f"they didn't seem to have the type of {target_product['category']} I was after."


        self.last_item_found_at_store = found_it_for_real
        if found_it_for_real:
            target_product['found'] = True 
            response_idea = f"Yes! I found the perfect {target_product['category']}! Exactly what I was looking for."
            # To make it more natural, sometimes a simpler "Yes, found it!"
            if random.random() < 0.5: response_idea = f"Yes, I found a great {target_product['category']}!"
        else:
            response_idea = f"No, unfortunately. {reason_for_not_finding}"
            if random.random() < 0.3: response_idea = f"Hmm, not quite. {reason_for_not_finding}"


        print(f"BENCHMARKING_LLM_FULFILLMENT: Target: {target_product['category']} ('{target_product['product_name']}'). Revealed: {self.revealed_details}. Robot context: '{item_context_summary_from_robot}'. Found specific item: {found_it_for_real}. Response: '{response_idea}'")

        prompt = f"""You are '{self.persona}'. You were secretly hoping to find something like '{target_product['product_name']}' (a {target_product['category']}).
The robot helped you look based on its understanding: '{item_context_summary_from_robot}' at '{store_name}'.
The robot asks: "...did you find what you were looking for...?"
You internally decided: '{response_idea}'. (This reflects if you found your secret item).
Respond naturally.

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=40)

    def provide_general_store_feedback(self, store_name):
        target_product = self._get_current_target_product()
        feedback_idea = "It was okay."

        if store_name in self.favorite_stores_for_this_session:
            feedback_idea = random.choice(["I love this store!", "Great atmosphere here.", "Always a good experience at " + store_name])
        elif self.last_item_found_at_store: 
            feedback_idea = random.choice(["So glad I found what I wanted here!", "Excellent, they had it!", "The selection for what I needed was perfect."])
        else: 
            if target_product: 
                 feedback_idea = random.choice([
                    f"They didn't have the {target_product['category']} I was looking for, unfortunately.",
                    "A bit disappointing for {target_product['category']}s.",
                    "The staff weren't very helpful when I asked about the " + target_product['category'] +"." if random.random() < 0.5 else "Selection was limited."
                ])
            else: 
                feedback_idea = random.choice(["Not very impressed.", "Could be better organized."])
        
        print(f"BENCHMARKING_LLM_STORE_FEEDBACK: Store: {store_name}. Fav?: {store_name in self.favorite_stores_for_this_session}. Item found here?: {self.last_item_found_at_store}. Feedback idea: '{feedback_idea}'")

        prompt = f"""You are '{self.persona}'. You just visited '{store_name}'.
The robot asked for general feedback. You are thinking: '{feedback_idea}'.
Respond naturally.

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def _move_to_next_target_item_or_stop(self):
        current_target = self._get_current_target_product()
        if current_target and current_target['found']:
            self.current_target_item_index += 1
            if self.current_target_item_index < len(self.shopping_list):
                self._prepare_revealed_details_for_current_target()
                return "new_target_item" 
            else:
                return "all_items_found" 
        return "current_item_not_yet_found_or_no_list"


    def decide_next_action(self, original_request_summary_from_robot, was_last_item_fulfilled_by_robot, has_more_options_for_current_robot_request):
        next_move_signal = self._move_to_next_target_item_or_stop()
        
        action_choice = "stop" 
        instruction_for_llm = "You've decided to stop shopping for now."
        example_response = "That's all for now, thanks."
        current_target_display = self._get_current_target_product()['category'] if self._get_current_target_product() else 'item'


        if next_move_signal == "new_target_item":
            action_choice = "new_request" 
            next_target_product = self._get_current_target_product() # This is now the NEW target
            
            query_parts = [self.revealed_details['category']] 
            if 'color' in self.revealed_details: query_parts.insert(0, self.revealed_details['color'])
            # Brand might be too specific for a "what next" vague query
            
            new_item_query_idea = f"Now I'm thinking about {' '.join(query_parts)}."
            if random.random() < 0.5: new_item_query_idea = f"What about looking for {' '.join(query_parts)}?"
            
            instruction_for_llm = f"You found your previous item. Now you want to look for your next item: '{next_target_product['category'] if next_target_product else 'something else'}'. You'll say something like: '{new_item_query_idea}'"
            example_response = new_item_query_idea
            self.current_shopping_goal_summary = f"{' '.join(query_parts)} for {self.persona}"


        elif next_move_signal == "all_items_found":
            action_choice = "stop"
            instruction_for_llm = "You've found everything on your shopping list!"
            example_response = "That's everything I needed, thank you so much!"

        else: # current_item_not_yet_found_or_no_list (or robot didn't fulfill current sub-request)
            if has_more_options_for_current_robot_request and random.random() < 0.7: # More likely to continue if options
                action_choice = "continue_current_request"
                instruction_for_llm = f"You haven't found your '{current_target_display}' yet. The robot has more options for '{original_request_summary_from_robot}'. You'll ask to see them."
                example_response = f"Okay, what other suggestions do you have for {original_request_summary_from_robot.split(' for ')[0]}?"
            else:
                action_choice = "stop" 
                instruction_for_llm = f"You haven't found your '{current_target_display}' and there are no more/no good options for '{original_request_summary_from_robot}'. You'll stop this search for now."
                example_response = "Alright, let's not worry about that one then."

        print(f"BENCHMARKING_LLM_NEXT_ACTION: Prev robot request: '{original_request_summary_from_robot}'. Robot thinks fulfilled: {was_last_item_fulfilled_by_robot}. User truth (last store): {self.last_item_found_at_store}. Next move signal: {next_move_signal}. Chosen action: {action_choice}. Instruction: '{instruction_for_llm}'")

        prompt = f"""You are '{self.persona}'.
The robot's last search was for '{original_request_summary_from_robot}'.
The robot thinks that search was '{'fulfilled' if was_last_item_fulfilled_by_robot else 'not fulfilled'}'.
Internally, your decision is: {instruction_for_llm}

Formulate a natural language response that clearly conveys ONLY this decision.
Example if you decided similarly: "{example_response}"

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=40)


    def respond_to_anything_else(self):
        # This is called when a sub-dialog (like finding one item) concludes,
        # or if the initial query processing completes without going into iterative visits.
        # Check if we should move to the next item on the main shopping list.
        
        # Advance to next item ONLY if current one was marked found.
        # If current not found, _move_to_next_target_item_or_stop will return "current_item_not_yet_found_or_no_list"
        current_target = self._get_current_target_product()
        if current_target and not current_target['found']:
            # If current target item is not found, and user chose to stop looking for it in decide_next_action,
            # then for "anything else", we should stop the whole session for this more focused benchmark.
            # OR, we could allow pivoting to the next item on the list here if the user is "giving up" on current one.
            # For now, let's make it simpler: if user stopped searching for current *unfound* item, they stop for the day.
            print(f"BENCHMARKING_LLM_ANYTHING_ELSE: Current target '{current_target['category']}' was not found, and user stopped that search. Ending session.")
            return "No, that's everything for today. Thank you!"

        next_move_signal = self._move_to_next_target_item_or_stop() 

        if next_move_signal == "new_target_item":
            next_target_product = self._get_current_target_product()
            
            query_parts = [self.revealed_details['category']]
            if 'color' in self.revealed_details: query_parts.insert(0, self.revealed_details['color'])
            
            new_item_query_idea = f"Actually, I was also thinking about {' '.join(query_parts)}."
            if random.random() < 0.3: new_item_query_idea = f"Yes, I also need to look for {' '.join(query_parts)}."

            self.current_shopping_goal_summary = f"{' '.join(query_parts)} for {self.persona}"
            print(f"BENCHMARKING_LLM_ANYTHING_ELSE: Moving to next target: {next_target_product['category']}. Query idea: '{new_item_query_idea}'")
            return new_item_query_idea
        else: # all_items_found or decided to stop
            print(f"BENCHMARKING_LLM_ANYTHING_ELSE: No more items or stopping. Signal: {next_move_signal}")
            return "No, that's everything. Thank you!"