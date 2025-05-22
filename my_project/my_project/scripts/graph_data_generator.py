# graph_data_generator.py
import re
import random
import networkx as nx

# Handle Python 2/3 compatibility for xrange
try:
    _ = xrange(1)
except NameError:
    xrange = range

# 0. Base Fashion MNIST Class Definitions (and Toy)
fashion_mnist_classes = [
    {"id": "I1", "name": "T-shirt/top", "category": "Apparel"},
    {"id": "I2", "name": "Trouser", "category": "Apparel"},
    {"id": "I3", "name": "Pullover", "category": "Apparel"},
    {"id": "I4", "name": "Dress", "category": "Apparel"},
    {"id": "I5", "name": "Coat", "category": "Outerwear"},
    {"id": "I6", "name": "Sandal", "category": "Footwear"},
    {"id": "I7", "name": "Shirt", "category": "Apparel"},
    {"id": "I8", "name": "Sneaker", "category": "Footwear"},
    {"id": "I9", "name": "Bag", "category": "Accessory"},
    {"id": "I10", "name": "Ankle boot", "category": "Footwear"},
    {"id": "I11", "name": "Toy", "category": "Kids"}
]

fictional_brands_list = ["Urban Threads", "Classic Co.", "Active Gear", "Glamora", "Evergreen Basics",
                        "SoleMate", "Terra Firma", "CarryAll", "Chic Steps", "WinterReady", "SummerBreeze",
                        "Urban Explorer", "Formal Touch", "Kids Corner", "Sporty Life", "GenericBrand", "NewBrand", "ToyBrand"]

colors_list = ["Red", "Blue", "Green", "Black", "White", "Grey", "Brown", "Pink", "Yellow",
              "Purple", "Beige", "Navy", "Olive", "Charcoal", "Cream", "Teal", "Maroon", "Mustard"]

materials_map = {
    "Apparel": ["Cotton", "Polyester Blend", "Wool", "Silk", "Linen", "Denim", "Knit", "Fleece", "Rayon"],
    "Outerwear": ["Wool Blend", "Down", "Synthetic Shell", "Leather", "Faux Fur"],
    "Footwear": ["Leather", "Suede", "Canvas", "Mesh", "Synthetic", "Rubber Sole"],
    "Accessory": ["Leather", "Canvas", "Nylon", "Faux Leather", "Metal Hardware"],
    "Kids": ["Plastic", "Plush", "Wood", "Safe Materials"]
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
    "Ankle boot": ["Heeled Leather Boot", "Chelsea Boot", "Hiking Style Boot", "Suede Ankle Boot", "Combat Boot"],
    "Toy": ["Action Figure", "Building Blocks Set", "Plush Animal", "Educational Toy", "RC Car"]
}

def load_store_coordinates(filename):
    """Loads store coordinates from a text file."""
    coords_list = []
    with open(filename, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            numbers = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
            if len(numbers) >= 2:
                try:
                    x = float(numbers[0])
                    y = float(numbers[1])
                    coords_list.append({'x': x, 'y': y, 'theta': 0.0})
                except ValueError:
                    print(f"Warning: Invalid numbers in line {line_num}: '{line}'. Skipping.")
            else:
                print(f"Warning: Line {line_num} has {len(numbers)} numbers (expected 2). Skipping.")
    print("tu madre a pecora",coords_list)
    return coords_list

def generate_fictional_products_and_stores():
    fictional_products_data = []
    product_id_counter = 1
    for fm_class in fashion_mnist_classes:
        base_class_id = fm_class["id"]
        base_class_name = fm_class["name"]
        category_type = fm_class["category"]
        num_instances_per_class = random.randint(2, 5)
        for _ in xrange(num_instances_per_class):
            product_template_list = product_name_templates.get(base_class_name, ["Generic {}".format(base_class_name)])
            specific_name_base = random.choice(product_template_list)
            chosen_color = random.choice(colors_list)
            
            if base_class_name == "Toy":
                product_name = specific_name_base
            else:
                product_name = "{} {}".format(chosen_color, specific_name_base)

            fictional_products_data.append({
                "product_id": "P{}".format(product_id_counter),
                "base_class_id": base_class_id,
                "base_class_name": base_class_name,
                "product_name": product_name,
                "brand": random.choice(fictional_brands_list),
                "color": chosen_color if base_class_name != "Toy" else "Various",
                "material": random.choice(materials_map[category_type]),
                "source_dataset_class": "Fictional"
            })
            product_id_counter += 1

    stores_data = [
        {"id": "S1", "name": "Urban Stylez", "type": "Trendy Apparel", "location_text": "Floor 1, Unit 101", "sells_base_classes": ["I1", "I2", "I3", "I4", "I5", "I8", "I9"]},
        {"id": "S2", "name": "Classic Comfort", "type": "Classic & Timeless Apparel", "location_text": "Floor 1, Unit 102", "sells_base_classes": ["I2", "I3", "I5", "I7"]},
        {"id": "S3", "name": "Footwear Palace", "type": "Shoe Store", "location_text": "Floor 2, Unit 201", "sells_base_classes": ["I6", "I8", "I10"]},
        {"id": "S4", "name": "Chic Boutique", "type": "High Fashion & Occasionwear", "location_text": "Floor 2, Unit 202", "sells_base_classes": ["I4", "I5", "I9", "I6"]},
        {"id": "S5", "name": "Outerwear Essentials", "type": "Outerwear Specialty Store", "location_text": "Floor 1, Unit 103", "sells_base_classes": ["I3", "I5"]},
        {"id": "S6", "name": "The Bag & Accessory Nook", "type": "Bags & Accessories Store", "location_text": "Ground Floor, Kiosk K1", "sells_base_classes": ["I9"]},
        {"id": "S7", "name": "Everyday Apparel", "type": "Basic & Casual Clothing", "location_text": "Floor 1, Unit 104", "sells_base_classes": ["I1", "I2", "I3", "I7"]},
        {"id": "S8", "name": "Active & Street", "type": "Sportswear & Streetwear", "location_text": "Floor 2, Unit 203", "sells_base_classes": ["I1", "I2", "I8", "I9"]},
        {"id": "S9", "name": "Kids Playworld", "type": "Toy Store", "location_text": "Floor 2, Unit 205", "sells_base_classes": ["I11"]}
    ]

    # Load and assign coordinates
    coords_list = load_store_coordinates('/home/ubuntu/tiago_public_ws/src/my_project/my_project/scripts/location.txt')
    for idx, store_info in enumerate(stores_data):
        if idx < len(coords_list):
            store_info.update({
                "map_x": coords_list[idx]['x'],
                "map_y": coords_list[idx]['y'],
                "map_theta": coords_list[idx]['theta']
            })
        else:
            store_info.update({"map_x": 0.0, "map_y": 0.0, "map_theta": 0.0})
            print(f"Warning: Store {store_info['id']} at index {idx} has default coordinates.")

    relationships_data = []
    for product in fictional_products_data:
        product_id = product["product_id"]
        base_class_id_of_product = product["base_class_id"]
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
            type=store['type'], location=store['location_text'],
            map_x=store.get('map_x'), map_y=store.get('map_y'), map_theta=store.get('map_theta')
        )
    for p_id, rel, s_id in relationships:
        if mall_graph.has_node(p_id) and mall_graph.has_node(s_id):
            mall_graph.add_edge(p_id, s_id, label_edge=rel)

    return mall_graph

if __name__ == '__main__':
    graph = create_mall_graph()
    print("Graph created with {} nodes and {} edges.".format(graph.number_of_nodes(), graph.number_of_edges()))
    
    # Print first 3 stores with coordinates
    print("\nSample store nodes:")
    store_count = 0
    for node_id, data in graph.nodes(data=True):
        if data.get('label_node') == 'Store' and store_count < 3:
            print(f"Store {node_id}:")
            print(f"  Name: {data['name']}")
            print(f"  Coordinates: ({data['map_x']}, {data['map_y']})")
            print(f"  Location: {data['location']}\n")
            store_count += 1

    # Print first 3 products
    print("\nSample product nodes:")
    product_count = 0
    for node_id, data in graph.nodes(data=True):
        if data.get('label_node') == 'Product' and product_count < 3:
            print(f"Product {node_id}:")
            print(f"  Name: {data['name']}")
            print(f"  Brand: {data['brand']}")
            print(f"  Color: {data['color']}\n")
            product_count += 1