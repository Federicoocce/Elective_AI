# mall_query_engine.py
import re
from graph_data_generator import create_mall_graph, colors_list, fictional_brands_list # Import necessary items

# --- Keyword Mappings & Preprocessing ---
ITEM_CATEGORY_KEYWORDS = {
    "shoes": ["Sandal", "Sneaker", "Ankle boot"], "footwear": ["Sandal", "Sneaker", "Ankle boot"],
    "boots": ["Ankle boot"], "sneakers": ["Sneaker"], "sandals": ["Sandal"],
    "top": ["T-shirt/top", "Shirt", "Pullover"], "t-shirt": ["T-shirt/top"],
    "shirt": ["Shirt", "T-shirt/top"], "pullover": ["Pullover"], "sweater": ["Pullover"],
    "trousers": ["Trouser"], "pants": ["Trouser"], "jeans": ["Trouser"],
    "dress": ["Dress"], "coat": ["Coat"], "jacket": ["Coat"], "bag": ["Bag"],
    "handbag": ["Bag"], "backpack": ["Bag"]
}
# Convert list of colors and brands from graph_data_generator to sets for faster lookups
COLOR_KEYWORDS_SET = set(c.lower() for c in colors_list)
BRAND_KEYWORDS_SET = set(b.lower() for b in fictional_brands_list)


def preprocess_text(text):
    text = text.lower()
    text = re.sub(r'[^\w\s-]', '', text) # Allow hyphens in names
    tokens = text.split()
    # Basic stop word removal can be added here if needed
    # stop_words = {"a", "new", "pair", "of", "for", "him", "her", "some", "i'm", "looking"}
    # tokens = [token for token in tokens if token not in stop_words]
    return tokens

def parse_request_to_structured_query(request_text):
    """
    Translates natural language request into a structured query dictionary.
    The "query" is a dictionary of criteria.
    Example: {'item_base_classes': {'Sandal', 'Sneaker'}, 'attributes': {'color': 'Red'}}
    """
    tokens = preprocess_text(request_text)
    
    structured_query = {
        "item_base_classes": set(),
        "attributes": {} # e.g., {"color": "Red", "brand": "Nike"}
    }

    # Identify Item Categories
    for token in tokens:
        if token in ITEM_CATEGORY_KEYWORDS:
            structured_query["item_base_classes"].update(ITEM_CATEGORY_KEYWORDS[token])
        elif token.endswith('s') and token[:-1] in ITEM_CATEGORY_KEYWORDS: # Simple plural check
            structured_query["item_base_classes"].update(ITEM_CATEGORY_KEYWORDS[token[:-1]])

    # Identify Attributes (Color, Brand - can be expanded)
    for token in tokens:
        if token in COLOR_KEYWORDS_SET:
            structured_query["attributes"]["color"] = token.capitalize() # Store with original capitalization
        # Could add brand detection here similarly if brands are more distinct
        # For example, if brands are multi-word, this simple token check is insufficient
        # Example for brand: (would need more robust parsing)
        # for brand_keyword in BRAND_KEYWORDS_SET:
        #     if brand_keyword in request_text.lower(): # Check substring for multi-word brands
        #         structured_query["attributes"]["brand"] = brand_keyword.title() # Or actual brand name from a map


    if not structured_query["item_base_classes"] and not structured_query["attributes"]:
        print(f"Debug: Could not parse any specific criteria from '{request_text}' with tokens: {tokens}")
        return None # No valid query criteria found

    return structured_query


def execute_structured_query(graph, structured_query):
    """
    Executes the structured query against the NetworkX graph.
    Returns a dictionary of {store_name: [list_of_matching_product_details]}
    """
    if not structured_query or not structured_query.get("item_base_classes"):
        # If only attributes are given, we might need a different logic
        # For now, require at least an item category for simplicity
        print("Query execution requires at least an item category.")
        return {}

    target_base_classes = structured_query["item_base_classes"]
    target_attributes = structured_query["attributes"]
    
    stores_with_matching_items = {}

    for node_id, data in graph.nodes(data=True):
        if data.get('label_node') == 'Product':
            # 1. Check base class match
            if not data.get('base_class_name') in target_base_classes:
                continue # Product is not of the desired base category

            # 2. Check attribute matches
            all_attributes_match = True
            for attr_key, attr_value in target_attributes.items():
                if data.get(attr_key, "").lower() != attr_value.lower(): # Case-insensitive attribute match
                    all_attributes_match = False
                    break
            
            if not all_attributes_match:
                continue # Product does not match all specified attributes

            # If we reach here, the product matches all criteria
            # Now find stores selling this product
            for successor_id in graph.successors(node_id):
                store_node_data = graph.nodes[successor_id]
                if store_node_data.get('label_node') == 'Store':
                    store_name = store_node_data.get('name')
                    if store_name not in stores_with_matching_items:
                        stores_with_matching_items[store_name] = []
                    
                    product_info = (f"{data.get('product_name')} (Brand: {data.get('brand')}, "
                                    f"Base: {data.get('base_class_name')}, Color: {data.get('color')})")
                    if product_info not in stores_with_matching_items[store_name]:
                        stores_with_matching_items[store_name].append(product_info)
                        
    return stores_with_matching_items


def process_natural_language_request(graph, nl_request):
    print(f"\nProcessing Natural Language Request: '{nl_request}'")
    
    structured_query_criteria = parse_request_to_structured_query(nl_request)
    
    if not structured_query_criteria:
        print("Could not understand the request or find specific criteria.")
        return

    print(f"Translated to Structured Query: {structured_query_criteria}")
    
    results = execute_structured_query(graph, structured_query_criteria)
    
    if results:
        print("\nFound the following stores and potential items:")
        for store, items in results.items():
            print(f"\n--- {store} ---")
            if items:
                for item_detail in items[:5]: # Show top 5
                    print(f"  - {item_detail}")
            else: # Should ideally not happen if store is a key
                print("  (No specific items listed for this store, though it matched general criteria)")
    else:
        print("Sorry, no stores or products found matching your precise criteria.")
        if structured_query_criteria.get("item_base_classes"):
            item_desc = ", ".join(structured_query_criteria["item_base_classes"])
            attr_desc = ""
            if structured_query_criteria.get("attributes"):
                attrs = [f"{k} {v}" for k,v in structured_query_criteria["attributes"].items()]
                attr_desc = f"with attributes ({', '.join(attrs)})"
            print(f"I was looking for {item_desc} {attr_desc}.")


# --- Main execution block ---
if __name__ == "__main__":
    print("Loading mall knowledge graph...")
    mall_graph = create_mall_graph() # Load the graph
    print("Graph loaded successfully.")

    # Example requests
    requests = [
        "Martino needs a new pair of shoes.",
        "I'm looking for a red t-shirt for him.",
        "Find some blue trousers.",
        "Where can I buy a black coat?",
        "Are there any Evergreen Basics bags?", # This tests brand, but our current brand parsing is too simple.
        "Martino wants a fancy dress", # "fancy" is not an attribute
        "Any green sneakers?"
    ]

    for req in requests:
        process_natural_language_request(mall_graph, req)
        print("-" * 50)