# mall_query_engine.py
from graph_data_generator import create_mall_graph
from llm_groq_parser import GroqQueryParser
import json # For potentially pretty printing profiles

# --- User Profile Management ---
USER_PROFILES = {
    "Mum": {"favorite_colors": [], "favorite_brands": [], "preferred_categories": []},
    "Dad": {"favorite_colors": [], "favorite_brands": [], "preferred_categories": []}
}

def update_user_profile(user_name, updates_from_llm):
    if user_name not in USER_PROFILES:
        print(f"Warning: User {user_name} not found in profiles. Cannot update.")
        return

    current_profile = USER_PROFILES[user_name]
    updated_any = False

    for key, new_values_list in updates_from_llm.items():
        if key in current_profile: # e.g., "favorite_colors", "favorite_brands", etc.
            if not isinstance(current_profile[key], list): # Ensure it's a list
                current_profile[key] = []
            
            if isinstance(new_values_list, list):
                for value in new_values_list:
                    if value not in current_profile[key]:
                        current_profile[key].append(value)
                        updated_any = True
            elif isinstance(new_values_list, str): # If LLM sends a single string
                if new_values_list not in current_profile[key]:
                    current_profile[key].append(new_values_list)
                    updated_any = True
    
    if updated_any:
        print(f"📝 Profile for {user_name} updated.")
        # print(f"New profile for {user_name}: {json.dumps(USER_PROFILES[user_name], indent=2)}")


def display_user_profile(user_name):
    if user_name not in USER_PROFILES:
        print(f"No profile found for {user_name}.")
        return

    profile = USER_PROFILES[user_name]
    print(f"\n📄 Current preferences for {user_name}:")
    has_preferences = False
    for key, values in profile.items():
        if values: # Only print if there are preferences for this key
            # Make key more readable: "favorite_colors" -> "Favorite colors"
            readable_key = key.replace('_', ' ').capitalize()
            print(f"   {readable_key}: {', '.join(values)}")
            has_preferences = True
    if not has_preferences:
        print("   No specific preferences recorded yet.")

# --- Query Processing ---
def process_natural_language_request(graph, nl_request, user_name, parser):
    if user_name not in USER_PROFILES:
        print(f"Error: User '{user_name}' is not recognized. Please use 'Mum' or 'Dad'.")
        return

    current_user_profile = USER_PROFILES[user_name]
    
    # (Re)Initialize Groq parser with store names (if not already set or if they can change)
    # For this setup, store names are static, so could be set once.
    # However, good practice if it could change.
    if not parser.store_names_list: # Set store names if not already done
        store_names = set()
        for node_id, data in graph.nodes(data=True):
            if data.get('label_node') == 'Store':
                store_names.add(data['name'])
        parser.set_store_names(store_names)

    structured_query = parser.generate_structured_query(nl_request, current_user_profile)
    
    if not structured_query:
        print("🤖 I'm sorry, I could not understand the request clearly.")
        return
    
    # Handle profile updates first
    if "updated_profile" in structured_query and structured_query["updated_profile"]:
        update_user_profile(user_name, structured_query["updated_profile"])

    # Handle intents
    intent = structured_query["intent"]

    if intent == "show_profile":
        display_user_profile(user_name)
        return

    if intent == "update_profile_only":
        print("🤖 Okay, I've noted your preferences.")
        display_user_profile(user_name) # Show updated profile
        return

    results = execute_structured_query(graph, structured_query)
    
    if intent == "find_store":
        if results.get("stores"):
            print("\nStore information:")
            for store in results["stores"]:
                print(f"🏬 {store['name']}")
                print(f"   Type: {store['type']}")
                print(f"   Location: {store['location']}\n")
        else:
            print("🤖 No matching stores found based on your query.")
            
    elif intent == "find_product":
        if results.get("products"):
            print("\nAvailable products matching your criteria:")
            for store_name_res, items in results["products"].items():
                print(f"\n📍 In Store: {store_name_res}")
                for item in items[:3]:  # Show max 3 items per store for brevity
                    print(f"  - {item}")
            if not results["products"]: # Double check if products list was empty after processing
                 print("🤖 No products found matching your specific criteria.")
        else:
            print("🤖 No products found matching your criteria.")


def execute_structured_query(graph, structured_query):
    intent = structured_query["intent"]
    
    if intent == "find_store":
        target_store_name = structured_query.get("store_name")
        results = {"stores": []}
        
        for node_id, data in graph.nodes(data=True):
            if data.get('label_node') == 'Store':
                store_name_graph = data['name']
                # If a specific store is targeted, match it. Otherwise, list all stores (though LLM should provide name if intent is find_store for a specific one)
                if (not target_store_name) or (target_store_name and target_store_name.lower() == store_name_graph.lower()):
                    results["stores"].append({
                        "name": store_name_graph,
                        "type": data['type'],
                        "location": data['location']
                    })
        return results
        
    elif intent == "find_product":
        target_categories = structured_query.get("item_types", [])
        target_attributes = structured_query.get("attributes", {})
        # Potentially use target_store_name if specified for product search in a specific store
        # target_store_for_product = structured_query.get("store_name") 

        stores_with_matching_items = {}

        for node_id, data in graph.nodes(data=True):
            if data.get('label_node') == 'Product':
                # Category match:
                # Product must be in one of the target categories if any are specified.
                # If no categories specified, product's category is implicitly accepted.
                category_match = not target_categories or data.get('base_class_name') in target_categories
                if not category_match:
                    continue

                # Attribute match:
                all_attributes_match = True
                if target_attributes: # Only check attributes if some are specified
                    all_attributes_match = all(
                        data.get(attr, "").lower() == str(value).lower() # Ensure value is string for comparison
                        for attr, value in target_attributes.items()
                    )
                
                if all_attributes_match: # True if target_attributes is empty or all specified attributes match
                    for successor_id in graph.successors(node_id): # Get stores selling this product
                        store_node_data = graph.nodes[successor_id]
                        if store_node_data.get('label_node') == 'Store':
                            store_name_graph = store_node_data['name']
                            product_info = f"{data['name']} (Brand: {data['brand']}, Color: {data['color']}, Material: {data['material']})"
                            
                            if store_name_graph not in stores_with_matching_items:
                                stores_with_matching_items[store_name_graph] = []
                            if product_info not in stores_with_matching_items[store_name_graph]: # Avoid duplicates
                                stores_with_matching_items[store_name_graph].append(product_info)
        
        return {"products": stores_with_matching_items}
    return {}

if __name__ == "__main__":
    print("Loading mall knowledge graph...")
    mall_graph = create_mall_graph()
    print("Graph loaded successfully.\n")

    # Initialize Groq parser (can be done once)
    query_parser = GroqQueryParser()
    # Set store names for the parser (once)
    store_names_from_graph = set()
    for _, data in mall_graph.nodes(data=True):
        if data.get('label_node') == 'Store':
            store_names_from_graph.add(data['name'])
    query_parser.set_store_names(store_names_from_graph)


    # --- Test Scenarios ---
    # User can be "Mum" or "Dad"
    current_user = "Mum"

    test_requests_with_users = [
        ("Mum", "Take me to Urban Stylez"),
        ("Dad", "Where can I find some black trousers? I prefer Classic Co. brand."),
        ("Mum", "I'm looking for a nice dress. My favorite color is Blue."),
        ("Dad", "Find Classic Comfort store"),
        ("Mum", "Show me black sneakers. I like Evergreen Basics for shoes."),
        ("Dad", "Where is the Footwear Palace?"),
        ("Mum", "I need a green T-shirt/top. Also, I'm starting to like the brand Active Gear."),
        ("Dad", "Locate stores that sell Bags."), # Note: Bag is an item category
        ("Mum", "What are my current preferences?"),
        ("Dad", "I'm interested in Wool material items. Especially pullovers."),
        ("Mum", "I think my favorite brand is now Glamora, I really love their style."),
        ("Dad", "What do you know about my tastes?"),
        ("Mum", "I want a red bag from CarryAll."),
        ("Dad", "Show me yellow sneakers"),
        ("Mum", "I don't need anything specific, but I like Apparel items."), # Testing preference update for category
        ("Dad", "My favorite color is definitely Navy."),
        ("Mum", "What were Dad's preferences again?"), # Will show Dad's profile
        ("Dad", "I'm looking for a Shirt. Any color is fine."),
        ("Mum", "Actually, I prefer Pink and Purple for colors."),
        ("Dad", "Show my profile details."),
    ]

    for user, req in test_requests_with_users:
        print(f"\n▶️  {user} asks: '{req}'")
        process_natural_language_request(mall_graph, req, user, query_parser)
        if user == "Mum" and "preferences" in req.lower() or "tastes" in req.lower() or "profile" in req.lower():
             pass # display_user_profile is called by process_natural_language_request
        elif user == "Dad" and "preferences" in req.lower() or "tastes" in req.lower() or "profile" in req.lower():
             pass
        print("-" * 70)

    print("\n--- Final User Profiles ---")
    display_user_profile("Mum")
    display_user_profile("Dad")