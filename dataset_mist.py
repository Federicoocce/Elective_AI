import json
import random
import networkx as nx
import matplotlib.pyplot as plt # Import for plotting

# --- [Data Generation Code - Same as before] ---
# 0. Base Fashion MNIST Class Definitions
fashion_mnist_classes = [
    {"id": "I1", "name": "T-shirt/top", "category": "Apparel"}, {"id": "I2", "name": "Trouser", "category": "Apparel"},
    {"id": "I3", "name": "Pullover", "category": "Apparel"}, {"id": "I4", "name": "Dress", "category": "Apparel"},
    {"id": "I5", "name": "Coat", "category": "Outerwear"}, {"id": "I6", "name": "Sandal", "category": "Footwear"},
    {"id": "I7", "name": "Shirt", "category": "Apparel"}, {"id": "I8", "name": "Sneaker", "category": "Footwear"},
    {"id": "I9", "name": "Bag", "category": "Accessory"}, {"id": "I10", "name": "Ankle boot", "category": "Footwear"}
]
fashion_mnist_class_map = {item['id']: item for item in fashion_mnist_classes}

fictional_brands = ["Urban Threads", "Classic Co.", "Active Gear", "Glamora", "Evergreen Basics", "SoleMate", "Terra Firma", "CarryAll", "Chic Steps"]
colors = ["Red", "Blue", "Green", "Black", "White", "Grey", "Brown", "Pink", "Yellow", "Purple", "Beige", "Navy", "Olive", "Charcoal", "Cream"]
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

# 1. Fictional Product Instances Dataset
fictional_products_data = []
product_id_counter = 1
for fm_class in fashion_mnist_classes:
    base_class_id = fm_class["id"]; base_class_name = fm_class["name"]; category_type = fm_class["category"]
    num_instances_per_class = random.randint(2, 6) # Reduced for a slightly less cluttered plot
    for _ in range(num_instances_per_class):
        product_template_list = product_name_templates.get(base_class_name, [f"Generic {base_class_name}"])
        specific_name_base = random.choice(product_template_list); chosen_color = random.choice(colors)
        product_name = f"{chosen_color} {specific_name_base}"
        fictional_products_data.append({
            "product_id": f"P{product_id_counter}", "base_class_id": base_class_id, "base_class_name": base_class_name,
            "product_name": product_name, "brand": random.choice(fictional_brands), "color": chosen_color,
            "material": random.choice(materials_map[category_type]), "source_dataset_class": "Fashion MNIST"
        })
        product_id_counter += 1

# 2. Store Nodes (Entities)
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

# 3. Relationship Dataset (Edges)
new_relationships_data = []
for product in fictional_products_data:
    product_id = product["product_id"]; base_class_id_of_product = product["base_class_id"]
    possible_stores = [store for store in stores_data if base_class_id_of_product in store["sells_base_classes"]]
    if possible_stores:
        num_stores_for_this_product = random.randint(1, min(len(possible_stores), 2))
        selected_stores = random.sample(possible_stores, num_stores_for_this_product)
        for store in selected_stores:
            new_relationships_data.append((product_id, "SOLD_IN", store["id"]))
# --- [End of Data Generation Code] ---


# --- Main part of the script with NetworkX and Plotting ---
if __name__ == "__main__":
    print("--- Fictional Product Instances Dataset ---")
    print(f"Total fictional products generated: {len(fictional_products_data)}")
    print("\n--- Stores Dataset ---")
    print(f"Total stores defined: {len(stores_data)}")
    print("\n--- New Relationships Dataset ---")
    print(f"Total 'SOLD_IN' relationships generated: {len(new_relationships_data)}")

    print("\n--- Loading Fictional Detailed Data into a NetworkX Graph ---")

    mall_graph_detailed = nx.DiGraph() # Directed graph

    # Add fictional product nodes
    for product in fictional_products_data:
        mall_graph_detailed.add_node(
            product['product_id'],
            label_node='Product', # Using 'label_node' consistently
            name=product['product_name'],
            base_class_id=product['base_class_id'],
            base_class_name=product['base_class_name'],
            brand=product['brand'],
            color=product['color'],
            material=product['material']
        )

    # Add store nodes
    for store in stores_data:
        mall_graph_detailed.add_node(
            store['id'],
            label_node='Store', # Using 'label_node' consistently
            name=store['name'], # 'name' is fine here as it's the store name
            type=store['type'],
            location=store['location']
        )

    # Add new relationship edges
    for product_id, rel_type, store_id in new_relationships_data:
        if rel_type == "SOLD_IN":
            if mall_graph_detailed.has_node(product_id) and mall_graph_detailed.has_node(store_id):
                mall_graph_detailed.add_edge(product_id, store_id, label_edge=rel_type) # Using 'label_edge'
            else:
                print(f"Warning: Node not found for relationship: ({product_id}, {store_id})")

    print(f"Detailed graph created with {mall_graph_detailed.number_of_nodes()} nodes and {mall_graph_detailed.number_of_edges()} edges.")

    # --- Plotting the NetworkX Graph ---
    print("\n--- Plotting the graph (this might take a moment for larger graphs) ---")
    plt.figure(figsize=(18, 12)) # Adjust figure size as needed

    # Position nodes using a layout algorithm
    # Kamada-Kawai layout usually works well for this kind of network
    # Other options: nx.spring_layout, nx.circular_layout, nx.shell_layout, etc.
    pos = nx.kamada_kawai_layout(mall_graph_detailed)
    # For very dense graphs, spring_layout might be better but slower:
    # pos = nx.spring_layout(mall_graph_detailed, k=0.15, iterations=20)


    # Define colors for node types
    node_colors = []
    for node in mall_graph_detailed.nodes(data=True):
        if node[1]['label_node'] == 'Product':
            node_colors.append('skyblue')
        elif node[1]['label_node'] == 'Store':
            node_colors.append('lightgreen')
        else:
            node_colors.append('grey') # Fallback, should not happen with current data

    # Define labels for nodes (can be node IDs or names)
    # Using just node IDs for simplicity on the plot, as full names can clutter.
    # You can customize this to show mall_graph_detailed.nodes[node]['name'] if preferred.
    labels = {node: node for node in mall_graph_detailed.nodes()}


    # Draw nodes
    nx.draw_networkx_nodes(mall_graph_detailed, pos, node_color=node_colors, node_size=500, alpha=0.9)

    # Draw edges
    nx.draw_networkx_edges(mall_graph_detailed, pos, arrowstyle='-|>', arrowsize=15, edge_color='gray', alpha=0.6)

    # Draw labels
    nx.draw_networkx_labels(mall_graph_detailed, pos, labels=labels, font_size=8)

    # Draw edge labels (e.g., "SOLD_IN")
    # This can get very cluttered on dense graphs. Use with caution or selectively.
    # edge_labels = nx.get_edge_attributes(mall_graph_detailed, 'label_edge')
    # nx.draw_networkx_edge_labels(mall_graph_detailed, pos, edge_labels=edge_labels, font_size=6)


    plt.title("Mall Product-Store Knowledge Graph (NetworkX)", fontsize=15)
    plt.axis('off') # Turn off the axis
    plt.tight_layout() # Adjust plot to prevent labels from being cut off
    plt.show()

    print("Plot display attempted. If it doesn't show, ensure Matplotlib is working correctly in your environment.")

    # --- Example Queries (remain the same) ---
    if fictional_products_data:
        first_product_id = fictional_products_data[0]['product_id']
        if mall_graph_detailed.has_node(first_product_id):
            product_details = mall_graph_detailed.nodes[first_product_id]
            print(f"\nStores selling '{product_details['name']}' (ID: {first_product_id}):")
            # In NetworkX, graph.out_edges(node) gives outgoing edges.
            # Or more directly, successors for a DiGraph:
            for store_node_id in mall_graph_detailed.successors(first_product_id):
                # We can check edge data if we stored it, e.g. mall_graph_detailed.edges[first_product_id, store_node_id]['label_edge']
                store_name = mall_graph_detailed.nodes[store_node_id]['name']
                print(f"- {store_name} (ID: {store_node_id})")
        else:
            print(f"Product ID {first_product_id} not found in the graph.")

    store_id_to_query = "S1"
    if mall_graph_detailed.has_node(store_id_to_query):
        store_name = mall_graph_detailed.nodes[store_id_to_query]['name']
        print(f"\nProducts sold in '{store_name}' (ID: {store_id_to_query}):")
        # For incoming edges to a store node (item -> store):
        for product_node_id in mall_graph_detailed.predecessors(store_id_to_query):
            product_details = mall_graph_detailed.nodes[product_node_id]
            print(f"- {product_details['name']} (Brand: {product_details['brand']}, Color: {product_details['color']}, ID: {product_node_id})")
    else:
        print(f"Store ID {store_id_to_query} not found in the graph.")