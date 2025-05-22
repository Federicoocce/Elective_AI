# mall_query_engine.py (KnowledgeGraphService)

from .graph_data_generator import create_mall_graph, fashion_mnist_classes, fictional_brands_list, colors_list # For entity extraction
import json # For pretty printing in __main__

class KnowledgeGraphService:
    def __init__(self):
        print("KGS: Loading mall knowledge graph...")
        self.graph = create_mall_graph()
        print("KGS: Graph loaded successfully.")
        self.store_names_from_graph, \
        self.item_categories_from_graph, \
        self.brands_from_graph, \
        self.colors_from_graph = self._extract_graph_entities()
        print(f"KGS: Extracted {len(self.store_names_from_graph)} stores, {len(self.item_categories_from_graph)} categories, {len(self.brands_from_graph)} brands, {len(self.colors_from_graph)} colors.")

    def _extract_graph_entities(self):
        store_names = set()
        # Use the definitive lists from graph_data_generator for categories, brands, colors
        # as the graph might not contain *all* possible ones if generation is random.
        item_categories = set(c["name"] for c in fashion_mnist_classes)
        brands = set(fictional_brands_list)
        colors = set(colors_list)

        for _, data in self.graph.nodes(data=True):
            if data.get('label_node') == 'Store':
                store_names.add(data['name'])
            # Products in graph will use subsets of the above, so no need to re-extract from products here
            # This ensures the LLM gets the full list of possibilities it was trained on for generation.
        return list(store_names), list(item_categories), list(brands), list(colors)

    def get_all_store_names(self):
        return self.store_names_from_graph

    def get_all_item_categories(self):
        return self.item_categories_from_graph

    def get_all_brands(self):
        return self.brands_from_graph

    def get_all_colors(self):
        return self.colors_from_graph

    def get_shop_details(self, shop_name_query):
        for node_id, data in self.graph.nodes(data=True):
            if data.get('label_node') == 'Store' and data['name'].lower() == shop_name_query.lower():
                return {
                    "id": node_id,
                    "name": data['name'],
                    "type": data['type'],
                    "location_text": data.get('location'),
                    "map_x": data.get('map_x'),
                    "map_y": data.get('map_y'),
                    "map_theta": data.get('map_theta')
                }
        return None

    def execute_structured_query(self, structured_query):
        intent = structured_query.get("intent")

        if intent == "find_store":
            target_store_name = structured_query.get("store_name")
            results = {"stores": []}
            for node_id, data in self.graph.nodes(data=True):
                if data.get('label_node') == 'Store':
                    store_name_graph = data['name']
                    if (not target_store_name) or \
                       (target_store_name and target_store_name.lower() == store_name_graph.lower()):
                        results["stores"].append({
                            "id": node_id, "name": store_name_graph, "type": data['type'],
                            "location_text": data.get('location'), "map_x": data.get('map_x'),
                            "map_y": data.get('map_y'), "map_theta": data.get('map_theta')
                        })
            return results

        elif intent == "find_product":
            target_categories = structured_query.get("item_types", [])
            target_attributes = structured_query.get("attributes", {})
            target_store_for_product_search = structured_query.get("store_name")
            stores_with_matching_items = {}

            for product_node_id, product_data in self.graph.nodes(data=True):
                if product_data.get('label_node') == 'Product':
                    category_match = not target_categories or product_data.get('base_class_name') in target_categories
                    if not category_match: continue

                    all_attributes_match = True
                    if target_attributes:
                        for attr, query_value in target_attributes.items():
                            if query_value: # Ensure query value is not None or empty for comparison
                                product_attr_value = product_data.get(attr)
                                if product_attr_value is None or \
                                   str(product_attr_value).lower() != str(query_value).lower():
                                    all_attributes_match = False
                                    break
                    if not all_attributes_match: continue

                    for store_node_id in self.graph.successors(product_node_id):
                        store_node_data = self.graph.nodes[store_node_id]
                        if store_node_data.get('label_node') == 'Store':
                            store_name_graph = store_node_data['name']
                            if target_store_for_product_search and \
                               target_store_for_product_search.lower() != store_name_graph.lower():
                                continue

                            product_info_dict = {
                                "id": product_node_id, "name": product_data['name'],
                                "brand": product_data['brand'], "color": product_data['color'],
                                "material": product_data['material'], "base_class_name": product_data['base_class_name']
                            }
                            if store_name_graph not in stores_with_matching_items:
                                stores_with_matching_items[store_name_graph] = {
                                    "store_details": {
                                        "id": store_node_id, "name": store_node_data['name'],
                                        "type": store_node_data['type'], "location_text": store_node_data.get('location'),
                                        "map_x": store_node_data.get('map_x'), "map_y": store_node_data.get('map_y'),
                                        "map_theta": store_node_data.get('map_theta')
                                    }, "products_found": []}
                            if product_info_dict not in stores_with_matching_items[store_name_graph]["products_found"]:
                                stores_with_matching_items[store_name_graph]["products_found"].append(product_info_dict)
            return {"products_in_stores": stores_with_matching_items}
        return {}

if __name__ == "__main__":
    print("--- Testing KnowledgeGraphService ---")
    kg_service = KnowledgeGraphService()
    
    print("\nShop details for 'Urban Stylez':")
    print(json.dumps(kg_service.get_shop_details("Urban Stylez"), indent=2))

    print("\nFinding black trousers:")
    query_trousers = {"intent": "find_product", "item_types": ["Trouser"], "attributes": {"color": "Black"}}
    results_trousers = kg_service.execute_structured_query(query_trousers)
    # print(json.dumps(results_trousers, indent=2)) # Can be very verbose
    for store_name, data in results_trousers.get("products_in_stores", {}).items():
        print(f"  Found in {store_name}: {len(data['products_found'])} black trousers.")
        for prod in data["products_found"][:1]: # Show one example
             print(f"    Example: {prod['name']}")


    print("\nFinding Kids Playworld store:")
    query_kids_store = {"intent": "find_store", "store_name": "Kids Playworld"}
    print(json.dumps(kg_service.execute_structured_query(query_kids_store), indent=2))