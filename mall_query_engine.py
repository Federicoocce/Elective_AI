# mall_query_engine.py
from graph_data_generator import create_mall_graph
from llm_groq_parser import GroqQueryParser

def process_natural_language_request(graph, nl_request):
    # Extract store names from graph
    store_names = set()
    for node_id, data in graph.nodes(data=True):
        if data.get('label_node') == 'Store':
            store_names.add(data['name'])

    # Initialize Groq parser
    parser = GroqQueryParser()
    parser.set_store_names(store_names)
    structured_query = parser.generate_structured_query(nl_request)
    
    if not structured_query:
        print("Could not understand the request.")
        return
    
    results = execute_structured_query(graph, structured_query)
    
    if structured_query["intent"] == "find_store":
        if results["stores"]:
            print("\nStore information:")
            for store in results["stores"]:
                print(f"🏬 {store['name']}")
                print(f"   Type: {store['type']}")
                print(f"   Location: {store['location']}\n")
        else:
            print("No matching stores found.")
            
    elif structured_query["intent"] == "find_product":
        if results["products"]:
            print("\nAvailable products:")
            for store, items in results["products"].items():
                print(f"\n📍 {store}")
                for item in items[:3]:  # Show max 3 items per store
                    print(f"  - {item}")
        else:
            print("No products found matching your criteria.")

def execute_structured_query(graph, structured_query):
    if structured_query["intent"] == "find_store":
        target_store = structured_query["store_name"]
        results = {"stores": []}
        
        for node_id, data in graph.nodes(data=True):
            if data.get('label_node') == 'Store':
                store_name = data['name']
                if (not target_store) or (target_store and target_store.lower() == store_name.lower()):
                    results["stores"].append({
                        "name": store_name,
                        "type": data['type'],
                        "location": data['location']
                    })
        return results
        
    elif structured_query["intent"] == "find_product":
        target_categories = structured_query["item_types"]
        target_attributes = structured_query["attributes"]
        stores_with_matching_items = {}

        for node_id, data in graph.nodes(data=True):
            if data.get('label_node') == 'Product' and data.get('base_class_name') in target_categories:
                all_attributes_match = all(
                    data.get(attr, "").lower() == value.lower()
                    for attr, value in target_attributes.items()
                )
                
                if all_attributes_match:
                    for successor_id in graph.successors(node_id):
                        store_data = graph.nodes[successor_id]
                        if store_data.get('label_node') == 'Store':
                            store_name = store_data['name']
                            product_info = f"{data['name']} (Brand: {data['brand']}, Color: {data['color']})"
                            
                            if store_name not in stores_with_matching_items:
                                stores_with_matching_items[store_name] = []
                            if product_info not in stores_with_matching_items[store_name]:
                                stores_with_matching_items[store_name].append(product_info)
        
        return {"products": stores_with_matching_items}
    return {}

if __name__ == "__main__":
    print("Loading mall knowledge graph...")
    mall_graph = create_mall_graph()
    print("Graph loaded successfully.\n")

    test_requests = [
        "Take me to Urban Stylez",
        "Where can I find red shoes?",
        "Find Classic Comfort store",
        "Show me black sneakers from Evergreen Basics",
        "Where is the Footwear Palace?",
        "I need a green dress",
        "Locate stores that sell bags",
        "Where can I get a winter coat?",
        "Show me pink sandals from Glamora",
        "Find Sport & Street store",
        "Where to buy a leather bag?",
        "Show me yellow sneakers"
    ]

    for req in test_requests:
        print(f"\nProcessing request: '{req}'")
        process_natural_language_request(mall_graph, req)
        print("-" * 50)