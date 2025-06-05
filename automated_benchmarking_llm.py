# automated_benchmarking_llm.py
import os
from groq import Groq
import json
import random
from dotenv import load_dotenv
import re

load_dotenv()

# Assuming VALID_USER_KEYS is defined elsewhere, e.g. in user_manager.py
# For standalone use or clarity, define it here if not imported.
# from user_manager import VALID_USER_KEYS # Ideal
VALID_USER_KEYS = ["mother", "father", "child"] # Fallback definition

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
        self.session_wide_favorite_stores = [] # For storing globally set favorite stores (used for TARGET ITEM selection)
        self.current_iteration_num = -1       # For tracking current benchmark iteration

        self.shopping_list = []
        self.current_target_item_index = -1
        self.revealed_details = {}
        # self.favorite_stores_for_this_session = [] # REMOVED - LLM user should not have internal session favorites
        self.last_item_found_at_store = False
        self.current_shopping_goal_summary = ""
        self.interaction_count = 0
        self.found_items = 0
        self.success = False

    def get_session_metrics(self):
        return {
            "interaction_count": self.interaction_count,
            "found_items": self.found_items,
            "total_items_in_list": len(self.shopping_list) if self.shopping_list else 0,
            "success": self.success,
            "persona": self.persona
        }

    def _get_current_target_product(self):
        if self.shopping_list and 0 <= self.current_target_item_index < len(self.shopping_list):
            return self.shopping_list[self.current_target_item_index]
        return None

    def _initialize_shopping_session_goals(self):
        self.shopping_list = []
        self.current_target_item_index = -1
        self.revealed_details = {}
        # self.favorite_stores_for_this_session = [] # REMOVED
        self.found_items = 0
        self.success = False
        self.interaction_count = 0

        if self.persona == "child":
            num_items_to_shop_for = 0
            self.success = True
        elif self.persona in ["mother", "father"]:
            num_items_to_shop_for = 1
        else:
            num_items_to_shop_for = 1

        if not self.sample_products_from_kg and num_items_to_shop_for > 0:
            print(f"BENCHMARKING_LLM_WARNING: No sample products available to create a shopping list of {num_items_to_shop_for} items.")
            return

        products_to_sample_from = self.sample_products_from_kg
        is_fav_store_iteration_for_target_item = self.current_iteration_num < 5 and self.persona in ["mother", "father"]

        if is_fav_store_iteration_for_target_item and self.session_wide_favorite_stores:
            print(f"BENCHMARKING_LLM_FAV_STORE_LOGIC: Iteration {self.current_iteration_num}, Persona {self.persona}. Targeting item from global favorite stores: {self.session_wide_favorite_stores}")

            fav_store_products = [
                p for p in self.sample_products_from_kg
                if p.get('store_name') in self.session_wide_favorite_stores
            ]

            if fav_store_products:
                products_to_sample_from = fav_store_products
                print(f"BENCHMARKING_LLM_FAV_STORE_LOGIC: Found {len(fav_store_products)} products from global favorite stores to sample from.")
            else:
                print(f"BENCHMARKING_LLM_WARNING: No sample products found in global favorite stores {self.session_wide_favorite_stores}. Persona {self.persona}, iter {self.current_iteration_num} will fall back to all sample products.")

        if num_items_to_shop_for == 0:
            print(f"BENCHMARKING_LLM_SESSION_GOALS: Persona: {self.persona}")
            print(f"  Shopping List ({num_items_to_shop_for} items): Empty as per persona configuration.")
            # print(f"  Favorite Stores for this session (LLM internal): N/A") # Adjusted print
            return

        if len(products_to_sample_from) < num_items_to_shop_for :
            print(f"BENCHMARKING_LLM_WARNING: Not enough products in 'products_to_sample_from' ({len(products_to_sample_from)}) to pick {num_items_to_shop_for}. Reducing list size.")
            num_items_to_shop_for = len(products_to_sample_from)
            if num_items_to_shop_for == 0:
                if is_fav_store_iteration_for_target_item:
                    print("BENCHMARKING_LLM_ERROR: CRITICAL - No products to sample for favorite store item objective. num_items_to_shop_for became 0.")
                return

        potential_targets_product_data = []
        if num_items_to_shop_for > 0 and products_to_sample_from:
            potential_targets_product_data = random.sample(products_to_sample_from, k=num_items_to_shop_for)

        for product_data in potential_targets_product_data:
            target_recipient = random.choice(VALID_USER_KEYS)
            self.shopping_list.append({
                "product_name": product_data.get('name'),
                "category": product_data.get('base_class_name'),
                "color": product_data.get('color'),
                "brand": product_data.get('brand'),
                "store_id": product_data.get('store_id'),
                "store_name": product_data.get('store_name'),
                "target_recipient_persona": target_recipient,
                "found": False
            })

        if self.shopping_list:
            self.current_target_item_index = 0
            self._prepare_revealed_details_for_current_target()

        # REMOVED LOGIC FOR self.favorite_stores_for_this_session
        # if is_fav_store_iteration_for_target_item and self.session_wide_favorite_stores:
        #     print(f"BENCHMARKING_LLM_FAV_STORE_LOGIC: Persona {self.persona} (iter {self.current_iteration_num}) target item selection was influenced by global favorites. LLM user itself has no special session favorites.")
        # elif self.kg_entities["store_names"] and self.persona != "child": # Logic for other iterations or personas (No LLM internal favs)
        #     pass


        print(f"BENCHMARKING_LLM_SESSION_GOALS: Persona: {self.persona}")
        print(f"  Shopping List ({len(self.shopping_list)} item(s)):")
        for i, item in enumerate(self.shopping_list):
            print(f"    {i+1}. {item['category']} (Color: {item['color']}, Brand: {item['brand']})")
            print(f"       - Target Name: '{item['product_name']}'")
            print(f"       - Target Store: '{item.get('store_name', 'Any/Unknown')}' (ID: {item.get('store_id', 'N/A')})")
            print(f"       - Item for: {item['target_recipient_persona']}")
        # print(f"  Favorite Stores for this session (LLM internal): N/A") # Adjusted print


    def _prepare_revealed_details_for_current_target(self):
        self.revealed_details = {}
        target_product = self._get_current_target_product()
        if target_product:
            self.revealed_details['category'] = target_product['category']
            if target_product['color'] and target_product['color'] != "Various" and random.random() < 0.4:
                self.revealed_details['color'] = target_product['color']
            if target_product['brand'] and random.random() < 0.3:
                 self.revealed_details['brand'] = target_product['brand']


    def set_knowledge_base(self, item_categories, brands, colors, store_names, sample_products=None, global_favorite_stores=None, current_iteration_num=None):
        self.kg_entities["item_categories"] = item_categories
        self.kg_entities["brands"] = brands
        self.kg_entities["colors"] = colors
        self.kg_entities["store_names"] = store_names
        self.sample_products_from_kg = sample_products if sample_products else []
        self.session_wide_favorite_stores = global_favorite_stores if global_favorite_stores else []
        self.current_iteration_num = current_iteration_num if current_iteration_num is not None else -1

        print(f"BENCHMARKING_LLM: Knowledge base set for persona {self.persona}. Iteration: {self.current_iteration_num}. Global Fav Stores (for item targeting): {self.session_wide_favorite_stores}. Sample products: {len(self.sample_products_from_kg)}")
        self._initialize_shopping_session_goals()

    def _call_groq_api_for_simulation(self, prompt, max_tokens=100):
        self.interaction_count += 1
        if not self.client:
            # Simplified mock logic based on prompt content
            # HCRLF placeholder means "hard carriage return line feed" to avoid actual newlines in f-string
            prompt_start_safe = prompt[:60].replace('\n', ' ').replace('\r', '')
            print(f"BENCHMARKING_LLM_MOCK: Prompt starts with: '{prompt_start_safe}'")
            target_product = self._get_current_target_product()

            if "initial shopping query" in prompt:
                if not target_product: return "I don't need anything right now, thanks."
                recipient_display_mock = "myself"
                if target_product.get('target_recipient_persona') and target_product.get('target_recipient_persona') != self.persona:
                    recipient_display_mock = f"my {target_product.get('target_recipient_persona')}"
                return f"I'm looking for a {self.revealed_details.get('category', 'new item')} for {recipient_display_mock}."

            if "who is this request for" in prompt:
                if target_product and target_product.get('target_recipient_persona'):
                    return f"It's for {target_product.get('target_recipient_persona')}."
                return f"It's for {self.persona}." # Fallback

            if "did you find what you were looking for" in prompt:
                # Simplified mock: 60% chance of "finding" if target_product exists
                if target_product and random.random() < 0.6:
                    self.last_item_found_at_store = True
                    # Actual 'found' status set by calling method using provide_fulfillment_feedback's logic
                    return f"Yes, I found a great {target_product['category']}!"
                else:
                    self.last_item_found_at_store = False
                    return f"No, not quite the {target_product['category'] if target_product else 'item'} I wanted."

            if "what next" in prompt:
                 if "You found your item" in prompt or "stop looking" in prompt: return "That's all for now, thanks!"
                 if "ask to see them" in prompt : return "Sure, what else?"
                 return "Okay, I'll stop then."

            if "anything else" in prompt: return "No, thanks."
            if "shall we go there" in prompt: return "Yes, let's go."
            if "when you're ready" in prompt: return "Okay, I'm ready."
            if "any other feedback" in prompt: return "It was fine."
            return "Okay."


        try:
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant", temperature=0.5, max_tokens=max_tokens,
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
            self.current_shopping_goal_summary = "nothing in particular"
            if self.persona == "child":
                return random.choice(["I don't think I need anything right now.", "Just looking around!", "Nope, I'm good."])
            return "I'm not looking for anything specific at the moment."

        item_category = target_product['category']
        target_recipient = target_product['target_recipient_persona']

        recipient_display = "myself" # Default
        if target_recipient == self.persona:
            recipient_display = random.choice(["myself", "me"])
        elif target_recipient == "father" and self.persona == "mother":
            recipient_display = random.choice(["my husband", "my father", "father"])
        elif target_recipient == "mother" and self.persona == "father":
            recipient_display = random.choice(["my wife", "my mother", "mother"])
        else: # For child, or cross-gender parent if not covered above
            recipient_display = f"my {target_recipient}"

        self.current_shopping_goal_summary = f"a {item_category} for {recipient_display}"
        # Add revealed details to summary if any
        if 'color' in self.revealed_details: self.current_shopping_goal_summary = f"a {self.revealed_details['color']} {item_category} for {recipient_display}"
        if 'brand' in self.revealed_details: self.current_shopping_goal_summary += f" (Brand: {self.revealed_details['brand']})"


        prompt = f"""You are impersonating a '{self.persona}'.
Your current shopping goal is to find: a {item_category}.
This item is intended for {recipient_display}.
Formulate a natural, BRIEF, and slightly vague initial shopping query for this item.
Your query should be a single short sentence.
If the item is for "{recipient_display}", your query should clearly reflect that.
Examples:
- "I'm looking for a new coat for myself."
- "I need some red shoes for my child."
- "Where can I find T-shirts for my husband?"
- "I'm looking for a gift for my father."
- "I need to find a {item_category} for {recipient_display}."

Your query (just the query, no preamble):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)

    def respond_to_clarify_shopping_target(self, robot_query_summary):
        target_product = self._get_current_target_product()
        intended_recipient = self.persona # Default to self
        if target_product and target_product.get('target_recipient_persona'):
            intended_recipient = target_product['target_recipient_persona']

        recipient_display = "me"
        if intended_recipient == self.persona:
            recipient_display = random.choice(["me", "myself"])
        else:
            recipient_display = f"my {intended_recipient}"
            if intended_recipient == "father" and self.persona == "mother" and random.random() <0.3: recipient_display = "my husband"
            if intended_recipient == "mother" and self.persona == "father" and random.random() <0.3: recipient_display = "my wife"


        prompt = f"""You are impersonating a '{self.persona}'. The robot asked who the request '{robot_query_summary}' is for.
The item you are currently thinking about is actually for {intended_recipient.capitalize()}.
Respond NATURALLY and VERY BRIEFLY, stating who it's for (e.g., "For me," "For my child," "It's for my husband.").
Do NOT add extra terms. Just a direct statement.

Your response (e.g., "For {recipient_display}."):"""
        example_mock_response = f"For {recipient_display}."
        if self.client: # Only use LLM if available
             return self._call_groq_api_for_simulation(prompt, max_tokens=10)
        else: # Fallback to mock if no LLM
            self.interaction_count +=1 # Manual increment for mock
            return example_mock_response


    def decide_to_visit_store(self, store_name_suggested, item_context_summary_from_robot):
        target_product = self._get_current_target_product()
        if not target_product:
            print(f"BENCHMARKING_LLM_DECISION: Persona '{self.persona}' has no target item. Declining visit to {store_name_suggested}.")
            return "No thanks, I'm just looking around."

        agree_prob = 0.7 # Base probability to visit any suggested store
        decision_reason = "general exploration"

        # Did robot suggest the correct store?
        correct_store_for_target = target_product.get('store_name')
        if correct_store_for_target and store_name_suggested == correct_store_for_target:
            agree_prob = 0.98 # Very high chance if robot suggests the EXACT store for the target item
            decision_reason = f"robot suggested the correct store '{correct_store_for_target}' for my item"
        # REMOVED check against self.favorite_stores_for_this_session
        elif target_product and item_context_summary_from_robot:
            # General check if context matches category (less important if store is wrong)
            if target_product['category'].lower() in item_context_summary_from_robot.lower():
                 agree_prob = max(agree_prob, 0.80) # Slightly increase if category matches, even if store is not the known target
                 decision_reason += " and category seems relevant"

        final_decision_is_yes = random.random() < agree_prob
        decision_prompt_idea = "Yes, let's go." if final_decision_is_yes else "No, not that one."

        current_target_display = target_product['category']
        revealed_display = str(self.revealed_details)

        # print(f"BENCHMARKING_LLM_DECISION: Visiting {store_name_suggested}? Target: {current_target_display} (for {target_product['target_recipient_persona']}) at '{correct_store_for_target}'. Revealed: {revealed_display}. LLM's session favs: {self.favorite_stores_for_this_session}. Robot said: '{item_context_summary_from_robot}'. Reason: {decision_reason}. Agree Prob: {agree_prob:.2f}. Decision: {'Yes' if final_decision_is_yes else 'No'}")
        print(f"BENCHMARKING_LLM_DECISION: Visiting {store_name_suggested}? Target: {current_target_display} (for {target_product['target_recipient_persona']}) at '{correct_store_for_target}'. Revealed: {revealed_display}. Robot said: '{item_context_summary_from_robot}'. Reason: {decision_reason}. Agree Prob: {agree_prob:.2f}. Decision: {'Yes' if final_decision_is_yes else 'No'}")


        prompt = f"""You are impersonating a '{self.persona}'.
The robot suggested visiting '{store_name_suggested}' to look for '{item_context_summary_from_robot}'.
Your actual target item is '{target_product['product_name']}' which you know is at '{correct_store_for_target}'.
You DO NOT have a list of 'favorite stores for this session'. Your decision is based on how well the robot's suggestion matches your hidden goal for this item.
Your internal decision, based on whether the robot suggested the right store or other factors (like it being plausible for your item category), is: '{decision_prompt_idea}'.
Based on your internal decision, respond with a very short phrase that clearly says yes or no to visiting THIS store.
Examples of 'yes' responses: "Yes, let's go.", "Okay.", "Sure."
Examples of 'no' responses: "No, not that one.", "No thanks.", "Not now."

Your response (must be very brief and clear):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=10)

    def acknowledge_arrival_and_readiness(self):
        self.last_item_found_at_store = False # Reset for this new store visit
        responses = ["Okay, I'm ready.", "I've had a look.", "Ready.", "Looked."]
        prompt = f"""You are '{self.persona}'. The robot said it arrived and asked you to let it know when you are ready.
You are now ready. Respond with a very short, natural confirmation.
Examples: "Okay, I'm ready.", "I've had a look.", "Ready."

Your response:"""
        if not self.client: self.interaction_count +=1 # Manual increment for mock
        return random.choice(responses) if not self.client else self._call_groq_api_for_simulation(prompt, max_tokens=8)


    def provide_fulfillment_feedback(self, item_context_summary_from_robot, store_name):
        target_product = self._get_current_target_product()
        found_it_for_real = False
        reason_for_not_finding = "it just wasn't right."

        if not target_product:
            self.success = True # Child with no list, already marked as success in init
            self.found_items = 0
            return self._call_groq_api_for_simulation("I wasn't looking for anything specific, but thanks.", max_tokens=15)

        correct_target_store_name = target_product.get('store_name')
        is_at_correct_store = correct_target_store_name and (store_name == correct_target_store_name)

        if is_at_correct_store:
            if target_product['category'].lower() in item_context_summary_from_robot.lower():
                if random.random() < 0.95:
                    found_it_for_real = True
                else:
                    reason_for_not_finding = "they were out of stock or it didn't quite match."
            else:
                reason_for_not_finding = f"this store ({store_name}) doesn't seem to have {target_product['category']}s, though I expected it here."
        else:
            reason_for_not_finding = f"I was looking for my {target_product['category']} at '{correct_target_store_name}', not here at {store_name}."
            if target_product['category'].lower() in item_context_summary_from_robot.lower() and random.random() < 0.1:
                found_it_for_real = True
                reason_for_not_finding = f"surprisingly found it at {store_name} although I expected it at {correct_target_store_name}!"
            else:
                 if target_product['category'].lower() in item_context_summary_from_robot.lower():
                     reason_for_not_finding += " This place has similar items, but not what I need."


        self.last_item_found_at_store = found_it_for_real
        if found_it_for_real:
            target_product['found'] = True
            self.success = True
            self.found_items = 1
            response_idea = f"Yes! Found the perfect {target_product['category']}!"
            if random.random() < 0.3: response_idea = f"Yes, this {target_product['product_name']} is great!"
        else:
            target_product['found'] = False
            self.success = False
            self.found_items = 0
            response_idea = f"No, unfortunately. {reason_for_not_finding}"
            if random.random() < 0.3: response_idea = f"Hmm, not quite. {reason_for_not_finding}"

        print(f"BENCHMARKING_LLM_FULFILLMENT: Target: {target_product['category']} ('{target_product['product_name']}') for {target_product['target_recipient_persona']} (Expected at: '{correct_target_store_name}'). Visited: '{store_name}'. Robot context: '{item_context_summary_from_robot}'. Found specific item: {found_it_for_real}. Response idea: '{response_idea}'")

        prompt = f"""You are '{self.persona}'. You were secretly hoping to find '{target_product['product_name']}' (a {target_product['category']}) which you know should be at '{correct_target_store_name}'.
The robot helped you look at '{store_name}' and asks: "...did you find what you were looking for...?"
Your internal decision/observation is: "{response_idea}".
Respond naturally and briefly, consistent with your decision. If not found, your reason should be short and can reflect if the store was not the one you expected for this item.

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=40)

    def provide_general_store_feedback(self, store_name):
        target_product = self._get_current_target_product()
        feedback_idea = "It was okay."

        # REMOVED: if store_name in self.favorite_stores_for_this_session:
        # feedback_idea = random.choice([f"I generally like {store_name}.", "It's a good store."])
        # ELIF becomes IF
        if self.last_item_found_at_store and target_product and store_name == target_product.get('store_name'):
            feedback_idea = random.choice([f"Glad I found my {target_product['category']} here as expected!", f"Excellent, they had the {target_product['product_name']}!"])
        elif target_product and store_name != target_product.get('store_name') and self.last_item_found_at_store:
             feedback_idea = f"Surprisingly good find, though I didn't expect my item here at {store_name}."
        elif target_product and store_name != target_product.get('store_name') and not self.last_item_found_at_store:
             feedback_idea = f"As expected, my item wasn't at {store_name}. I was looking for it at {target_product.get('store_name')}."
        else:
            if target_product:
                 feedback_idea = random.choice([
                    f"Didn't have the {target_product['category']} I wanted at {store_name}.",
                    f"Selection for {target_product['category']}s wasn't great at {store_name}."
                ])
            else:
                feedback_idea = random.choice(["Selection wasn't very interesting for me.", "It's a nice store, but nothing for me today."])

        # print(f"BENCHMARKING_LLM_STORE_FEEDBACK: Store: {store_name}. LLM Session Fav?: {store_name in self.favorite_stores_for_this_session}. Item found here?: {self.last_item_found_at_store}. Feedback idea: '{feedback_idea}'")
        print(f"BENCHMARKING_LLM_STORE_FEEDBACK: Store: {store_name}. Item found here?: {self.last_item_found_at_store}. Feedback idea: '{feedback_idea}'")


        prompt = f"""You are '{self.persona}'. You just visited '{store_name}'.
The robot asked for general feedback about the store.
Your internal thought/summary is: '{feedback_idea}'. This is based on your shopping experience for your target item.
Respond naturally, reflecting this thought. Keep your feedback brief.
Example if similar: "It was good." or "Selection wasn't great, as my item should be at [other store]."

Your response:"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=30)


    def decide_next_action(self, original_request_summary_from_robot, was_last_item_fulfilled_by_robot, has_more_options_for_current_robot_request):
        current_target_product = self._get_current_target_product()
        current_target_display = current_target_product['category'] if current_target_product else 'item'

        if not current_target_product:
            instruction_for_llm = "You have no active shopping goal. You'll stop."
            example_response = "I think that's all, thanks."
        elif self.last_item_found_at_store:
            instruction_for_llm = f"You found your '{current_target_display}'. You'll stop now."
            example_response = "Great, I found it! That's all for now."
        else:
            if has_more_options_for_current_robot_request and random.random() < 0.5:
                instruction_for_llm = f"You haven't found your '{current_target_display}'. Robot has more options for it. You'll ask to see them."
                example_response = f"Any other places for the {current_target_display.lower()}?"
            else:
                instruction_for_llm = f"You couldn't find your '{current_target_display}' or decided to stop looking for it. You'll stop the session."
                example_response = "Okay, I'll stop for today. Thanks."

        print(f"BENCHMARKING_LLM_NEXT_ACTION (Single Item Session): Prev robot req: '{original_request_summary_from_robot}'. Robot context (fulfilled by robot): {was_last_item_fulfilled_by_robot}. User truth (LLM item found): {self.last_item_found_at_store}. Instruction: '{instruction_for_llm}'")

        prompt = f"""You are '{self.persona}'. You are shopping for one main item: '{current_target_display}'.
The robot's last search was regarding '{original_request_summary_from_robot}'.
Robot context indicated: item was previously {'fulfilled' if was_last_item_fulfilled_by_robot else 'not fulfilled'}, and robot {'has' if has_more_options_for_current_robot_request else 'does not have'} more options for this item.
Your internal decision, based on whether YOU believe you found your item (ground truth: {'found' if self.last_item_found_at_store else 'not found'}) or want to continue looking for THIS item, is: {instruction_for_llm}

Formulate a natural language response that clearly and BRIEFLY conveys ONLY this decision.
Example if you decided similarly: "{example_response}"

Your response (short and direct):"""
        return self._call_groq_api_for_simulation(prompt, max_tokens=25)


    def respond_to_anything_else(self):
        current_target = self._get_current_target_product()
        final_message = "No, that's everything. Thank you!"

        if current_target:
            if self.success:
                final_message = "No, that's all I needed. Thanks!"
                log_msg = f"BENCHMARKING_LLM_ANYTHING_ELSE: Single item '{current_target['category']}' (for {current_target['target_recipient_persona']}) was found. Ending session."
            else:
                final_message = "No, I'll stop for now. Thanks anyway!"
                log_msg = f"BENCHMARKING_LLM_ANYTHING_ELSE: Single item '{current_target['category']}' (for {current_target['target_recipient_persona']}) was not found or given up on. Ending session."
        else:
             log_msg = f"BENCHMARKING_LLM_ANYTHING_ELSE: No shopping list for persona {self.persona}. Ending session."
             if self.persona == "child":
                final_message = "No, I'm good. Thanks!"
                self.success = True

        print(log_msg)
        if not self.client: self.interaction_count +=1 # Manual increment for mock
        return final_message