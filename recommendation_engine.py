# recommender_engine.py
import math

class RecommenderEngine:
    def __init__(self, knowledge_graph_service, user_profile_manager):
        self.kg_service = knowledge_graph_service
        self.profile_manager = user_profile_manager
        print("RecommenderEngine initialized.")

    def _calculate_distance(self, x1, y1, x2, y2):
        if None in [x1, y1, x2, y2]:
            return float('inf') # Cannot calculate distance if coordinates are missing
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def generate_recommendations(self, parsed_llm_data, kg_query_results, active_user_profile, robot_pose):
        """
        Generates ranked recommendations.
        parsed_llm_data: Output from LLM (intent, item_types, attributes, etc.)
        kg_query_results: Results from KnowledgeGraphService (e.g., {"stores": [...] } or {"products_in_stores": {...}})
        active_user_profile: The profile dictionary for the user we are shopping for.
        robot_pose: Tuple (x, y, theta_rad) of the robot's current position.
        """
        print(f"RECOMMENDER: Generating recommendations. Intent: {parsed_llm_data.get('intent')}")
        recommendations = {}
        robot_x, robot_y, _ = robot_pose if robot_pose else (None, None, None)

        intent = parsed_llm_data.get("intent")

        if intent == "find_store":
            # kg_query_results should be like: {"stores": [store_details_dict, ...]}
            raw_stores = kg_query_results.get("stores", [])
            scored_stores = []
            for store in raw_stores:
                score = 0
                # 1. Vicinity bonus (higher score for closer stores)
                distance = self._calculate_distance(robot_x, robot_y, store.get("map_x"), store.get("map_y"))
                if distance < float('inf'):
                    score += max(0, 100 - distance * 5) # Arbitrary scoring based on distance

                # 2. User profile bonus (based on past reviews for this specific store)
                shop_review = active_user_profile.get("shop_reviews", {}).get(store["name"], {})
                if shop_review.get("rating"):
                    score += shop_review["rating"] * 10 # e.g., 5-star rating adds 50 points
                if shop_review.get("last_visit_fulfilled") is True:
                    score += 20
                elif shop_review.get("last_visit_fulfilled") is False:
                    score -= 10
                
                # 3. (Future) Bonus if store type matches preferred categories (more complex logic needed)

                scored_stores.append({**store, "score": score, "distance": distance})
            
            recommendations["stores"] = sorted(scored_stores, key=lambda x: x["score"], reverse=True)
            # print(f"RECOMMENDER: Scored stores: {recommendations['stores']}")


        elif intent == "find_product":
            # kg_query_results should be like: {"products_in_stores": {store_name: {store_details: ..., products_found: [...]}}}
            products_by_store = kg_query_results.get("products_in_stores", {})
            ranked_store_product_options = []

            for store_name, data in products_by_store.items():
                store_details = data["store_details"]
                products_found = data["products_found"]
                
                store_score = 0
                # 1. Vicinity
                distance = self._calculate_distance(robot_x, robot_y, store_details.get("map_x"), store_details.get("map_y"))
                if distance < float('inf'):
                    store_score += max(0, 100 - distance * 5)

                # 2. User profile (shop review)
                shop_review = active_user_profile.get("shop_reviews", {}).get(store_name, {})
                if shop_review.get("rating"):
                    store_score += shop_review["rating"] * 10
                if shop_review.get("last_visit_fulfilled") is True:
                    store_score += 20
                
                # 3. Product relevance/preference (more complex)
                # - Do products match favorite brands of the user?
                # - Do product categories match preferred categories?
                # - Do specific items have positive/negative feedback?
                # This part can get quite detailed. For now, a simple brand match:
                num_fav_brand_matches = 0
                for product in products_found:
                    if product["brand"] in active_user_profile.get("favorite_brands", []):
                        num_fav_brand_matches += 1
                store_score += num_fav_brand_matches * 5 # Bonus for each favorite brand found

                ranked_store_product_options.append({
                    "store_details": store_details,
                    "products_found": products_found, # Could also score individual products later
                    "score": store_score,
                    "distance": distance
                })
            
            recommendations["products_in_stores_ranked"] = sorted(ranked_store_product_options, key=lambda x: x["score"], reverse=True)
            # print(f"RECOMMENDER: Ranked product options: {recommendations['products_in_stores_ranked']}")

        return recommendations

if __name__ == '__main__':
    print("--- Testing RecommenderEngine ---")
    # Mock dependencies for testing
    class MockKGService:
        def get_all_store_names(self): return ["Store A", "Store B", "Store C"]
        def get_shop_details(self, name): # Not directly used by recommender in this test
            if name == "Store A": return {"name": "Store A", "map_x": 0, "map_y": 0, "type": "Apparel"}
            return None

    class MockUserProfileManager:
        def get_profile(self, user_name):
            if user_name == "mother":
                return {
                    "favorite_brands": ["BrandX"],
                    "shop_reviews": {
                        "Store A": {"rating": 5, "last_visit_fulfilled": True},
                        "Store B": {"rating": 3}
                    },
                    # ... other profile fields
                }
            return {"shop_reviews": {}} # Default empty profile

    kg_mock = MockKGService()
    profile_mock = MockUserProfileManager()
    recommender = RecommenderEngine(kg_mock, profile_mock)

    robot_pose_test = (0, 0, 0) # Robot at origin
    mother_profile = profile_mock.get_profile("mother")

    # Test Case 1: Find Store
    parsed_llm_find_store = {"intent": "find_store"}
    kg_results_stores = {
        "stores": [
            {"name": "Store A", "map_x": 1, "map_y": 1, "type": "Apparel"}, # Close, good review
            {"name": "Store B", "map_x": 10, "map_y": 10, "type": "Shoes"}, # Far, okay review
            {"name": "Store C", "map_x": 2, "map_y": 0, "type": "Electronics"} # Close, no review
        ]
    }
    recs_stores = recommender.generate_recommendations(parsed_llm_find_store, kg_results_stores, mother_profile, robot_pose_test)
    print("\nStore Recommendations:")
    for store_rec in recs_stores.get("stores", []):
        print(f"  - {store_rec['name']} (Score: {store_rec['score']:.0f}, Distance: {store_rec['distance']:.1f})")

    # Test Case 2: Find Product
    parsed_llm_find_product = {"intent": "find_product", "item_types": ["Dress"]}
    kg_results_products = {
        "products_in_stores": {
            "Store A": { # Close, good review, brand match
                "store_details": {"name": "Store A", "map_x": 1, "map_y": 1, "type": "Apparel"},
                "products_found": [{"name": "Red Dress", "brand": "BrandX"}, {"name": "Blue Dress", "brand": "BrandY"}]
            },
            "Store B": { # Far, okay review, no brand match
                "store_details": {"name": "Store B", "map_x": 10, "map_y": 10, "type": "Shoes"},
                "products_found": [{"name": "Green Dress", "brand": "BrandZ"}]
            }
        }
    }
    recs_products = recommender.generate_recommendations(parsed_llm_find_product, kg_results_products, mother_profile, robot_pose_test)
    print("\nProduct Location Recommendations (Stores ranked):")
    for store_option in recs_products.get("products_in_stores_ranked", []):
        print(f"  - Store: {store_option['store_details']['name']} (Overall Score: {store_option['score']:.0f}, Distance: {store_option['distance']:.1f}) has {len(store_option['products_found'])} matching products.")