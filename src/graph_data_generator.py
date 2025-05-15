# graph_data_generator.py
import random
import networkx as nx

# --- [Data Generation Code - Copied from previous versions] ---
# 0. Base Fashion MNIST Class Definitions
fashion_mnist_classes = [
    {"id": "I1", "name": "T-shirt/top", "category": "Apparel"}, {"id": "I2", "name": "Trouser", "category": "Apparel"},
    {"id": "I3", "name": "Pullover", "category": "Apparel"}, {"id": "I4", "name": "Dress", "category": "Apparel"},
    {"id": "I5", "name": "Coat", "category": "Outerwear"}, {"id": "I6", "name": "Sandal", "category": "Footwear"},
    {"id": "I7", "name": "Shirt", "category": "Apparel"}, {"id": "I8", "name": "Sneaker", "category": "Footwear"},
    {"id": "I9", "name": "Bag", "category": "Accessory"}, {"id": "I10", "name": "Ankle boot", "category": "Footwear"}
]

fictional_brands_list = ["Urban Threads", "Classic Co.", "Active Gear", "Glamora", "Evergreen Basics", "SoleMate", "Terra Firma", "CarryAll", "Chic Steps"]
colors_list = ["Red", "Blue", "Green", "Black", "White", "Grey", "Brown", "Pink", "Yellow", "Purple", "Beige", "Navy", "Olive", "Charcoal", "Cream"]
materials_map = {
    "Apparel": ["Cotton", "Polyester Blend", "Wool", "Silk", "Linen", "Denim", "Knit", "Fleece", "Rayon"],
    "Outerwear": ["Wool Blend", "Down", "Synthetic Shell", "Leather", "Faux Fur"],
    "Footwear": ["Leather", "Suede", "Canvas", "Mesh", "Synthetic", "Rubber Sole"],
    "Accessory": ["Leather", "Canvas", "Nylon", "Faux Leather", "Metal Hardware"]
}
product_name_templates = {
    "T-shirt/top": ["Graphic Print Tee", "Essential V-Neck", "Long-Sleeve Basic", "Striped Polo", "Cropped Top"],
    "Trouser": ["Slim-Fit Chinos", "Classic Denim Jeans", "Wide-Leg Trousers", "Cargo Pants", "Comfort Joggers"],
    "Pullover": ["Cable Knit Sweater", "Hooded Sweatshirt", "Fine Gauge Pullover", "Fleece Zip-Up", "Chunky Cardigan"],
    "Dress": ["Floral Maxi Dress", "Little Black Dress", "Shirt Dress", "Summer Sundress", "Evening Gown"],
    "Coat": ["Wool Peacoat", "Quilted Puffer Jacket", "Classic Trench Coat", "Faux Fur Lined Parka", "Lightweight Bomber"],
    "Sandal": ["Strappy Heeled Sandal", "Comfort Slide", "Leather Espadrille", "Sporty Sandal", "Flip-Flop"],
    "Shirt": ["Crisp Cotton Shirt", "Silk Blouse", "Casual Denim Shirt", "Flannel Button-Down", "Formal Oxford"],
    "Sneaker": ["Retro Runner Sneaker", "High-Top Canvas Shoe", "Performance Trainer", "Chunky Fashion Sneaker", "Minimalist Sneaker"],
    "Bag": ["Leather Tote Bag", "Crossbody Messenger", "Travel Backpack", "Evening Clutch", "Canvas Shopper"],
    "Ankle boot": ["Heeled Leather Boot", "Chelsea Boot", "Hiking Style Boot", "Suede Ankle Boot", "Combat Boot"]
}
# --- [End of constants for data generation] ---


def generate_fictional_products_and_stores():
    fictional_products_data = []
    product_id_counter = 1
    for fm_class in fashion_mnist_classes:
        base_class_id = fm_class["id"]; base_class_name = fm_class["name"]; category_type = fm_class["category"]
        num_instances_per_class = random.randint(3, 5) # Products per category
        for _ in range(num_instances_per_class):
            product_template_list = product_name_templates.get(base_class_name, [f"Generic {base_class_name}"])
            specific_name_base = random.choice(product_template_list); chosen_color = random.choice(colors_list)
            product_name = f"{chosen_color} {specific_name_base}"
            fictional_products_data.append({
                "product_id": f"P{product_id_counter}", "base_class_id": base_class_id, "base_class_name": base_class_name,
                "product_name": product_name, "brand": random.choice(fictional_brands_list), "color": chosen_color,
                "material": random.choice(materials_map[category_type]), "source_dataset_class": "Fashion MNIST"
            })
            product_id_counter += 1

    stores_data = [
        {"id": "S1", "name": "Urban Stylez", "type": "Trendy Apparel", "location": "Floor 1, Unit 101", "sells_base_classes": ["I1", "I2", "I3", "I4", "I5", "I6", "I7", "I8", "I9", "I10"]},
        {"id": "S2", "name": "Classic Comfort", "type": "Classic & Timeless Apparel", "location": "Floor 1, Unit 102", "sells_base_classes": ["I2", "I3", "I5", "I7"]},
        {"id": "S3", "name": "Footwear Palace", "type": "Shoe Store", "location": "Floor 2, Unit 201", "sells_base_classes": ["I6", "I8", "I10", "I9"]},
        {"id": "S4", "name": "Chic Boutique", "type": "High Fashion & Occasionwear", "location": "Floor 2, Unit 202", "sells_base_classes": ["I1", "I4", "I5", "I6", "I7", "I9", "I10"]},
        {"id": "S5", "name": "Outerwear Essentials", "type": "Outerwear Specialty Store", "location": "Floor 1, Unit 103", "sells_base_classes": ["I3", "I5", "I10"]},
        {"id": "S6", "name": "The Bag & Accessory Nook", "type": "Bags & Accessories Store", "location": "Ground Floor, Kiosk K1", "sells_base_classes": ["I9"]},
        {"id": "S7", "name": "Everyday Apparel", "type": "Basic & Casual Clothing", "location": "Floor 1, Unit 104", "sells_base_classes": ["I1", "I2", "I3", "I7"]},
        {"id": "S8", "name": "Active & Street", "type": "Sportswear & Streetwear", "location": "Floor 2, Unit 203", "sells_base_classes": ["I1", "I2", "I3", "I8", "I9"]}
    ]

    relationships_data = []
    for product in fictional_products_data:
        product_id = product["product_id"]; base_class_id_of_product = product["base_class_id"]
        possible_stores = [store for store in stores_data if base_class_id_of_product in store["sells_base_classes"]]
        if possible_stores:
            num_stores_for_this_product = random.randint(1, min(len(possible_stores), 2))
            selected_stores = random.sample(possible_stores, num_stores_for_this_product)
            for store in selected_stores:
                relationships_data.append((product_id, "SOLD_IN", store["id"]))
    return fictional_products_data, stores_data, relationships_data


def create_mall_graph():
    """Creates and returns the detailed mall knowledge graph."""
    products, stores, relationships = generate_fictional_products_and_stores()
    
    mall_graph = nx.DiGraph()

    for product in products:
        mall_graph.add_node(
            product['product_id'], label_node='Product', name=product['product_name'],
            base_class_id=product['base_class_id'], base_class_name=product['base_class_name'],
            brand=product['brand'], color=product['color'], material=product['material']
        )
    for store in stores:
        mall_graph.add_node(
            store['id'], label_node='Store', name=store['name'],
            type=store['type'], location=store['location']
        )
    for p_id, rel, s_id in relationships:
        if mall_graph.has_node(p_id) and mall_graph.has_node(s_id):
            mall_graph.add_edge(p_id, s_id, label_edge=rel)
            
    print(f"Graph created with {mall_graph.number_of_nodes()} nodes and {mall_graph.number_of_edges()} edges.")
    return mall_graph

if __name__ == '__main__':
    # This part is just for testing this file independently
    graph = create_mall_graph()
    print("Graph generation test successful.")
    # Example: print a few nodes
    # for i, (node_id, data) in enumerate(graph.nodes(data=True)):
    #     if i < 5:
    #         print(f"Node: {node_id}, Data: {data}")
    #     else:
    #         break