# recommender_engine.py
import math

class RecommenderEngine:
    def __init__(self, knowledge_graph_service, user_profile_manager):
        self.kg_service = knowledge_graph_service
        self.profile_manager = user_profile_manager
        print("RecommenderEngine initialized.")

    def _calculate_distance(self, x1, y1, x2, y2):
        if None in [x1, y1, x2, y2]: # Check if any coordinate is None
            return float('inf') # Cannot calculate distance if coordinates are missing
        try:
            # Ensure coordinates are numbers before calculation
            x1_f, y1_f, x2_f, y2_f = float(x1), float(y1), float(x2), float(y2)
            return math.sqrt((x2_f - x1_f)**2 + (y2_f - y1_f)**2)
        except (TypeError, ValueError):
            print(f"RECOMMENDER: Warning - Invalid coordinate types for distance calc: {x1},{y1} to {x2},{y2}")
            return float('inf')


    def generate_recommendations(self, parsed_llm_data, kg_query_results, active_user_profile, robot_pose): # robot_pose is (x,y,theta_rad)
        """
        Generates ranked recommendations.
        robot_pose: Tuple (x, y, theta_rad) of the robot's current position. Can be (None, None, None).
        """
        print(f"RECOMMENDER: Generating recommendations. Intent: {parsed_llm_data.get('intent')}")
        recommendations = {}
        
        robot_x, robot_y, _ = robot_pose if robot_pose and robot_pose[0] is not None and robot_pose[1] is not None else (None, None, None)
        
        print(f"RECOMMENDER: Using robot pose for scoring: x={robot_x}, y={robot_y}")


        intent = parsed_llm_data.get("intent")

        if intent == "find_store":
            raw_stores = kg_query_results.get("stores", [])
            scored_stores = []
            for store in raw_stores:
                score = 0
                # 1. Vicinity bonus
                distance = self._calculate_distance(robot_x, robot_y, store.get("map_x"), store.get("map_y"))
                if distance < float('inf'): 
                    score += max(0, 100 - distance * 5) # Arbitrary scoring based on distance
                    print(f"RECOMMENDER: Store {store['name']}, distance {distance:.2f}, vicinity score part {max(0, 100 - distance * 5)}")

                # 2. User profile bonus (based on past reviews for this specific store)
                shop_review = active_user_profile.get("shop_reviews", {}).get(store["name"], {})
                profile_score_part = 0
                if shop_review.get("rating"):
                    profile_score_part += shop_review["rating"] * 10 
                if shop_review.get("last_visit_fulfilled") is True:
                    profile_score_part += 20
                elif shop_review.get("last_visit_fulfilled") is False:
                    profile_score_part -= 10
                score += profile_score_part
                print(f"RECOMMENDER: Store {store['name']}, profile score part {profile_score_part}")
                
                scored_stores.append({**store, "score": score, "distance": distance})
            
            recommendations["stores"] = sorted(scored_stores, key=lambda x: (x["score"], -x["distance"] if x["distance"] is not None else float('inf')), reverse=True)
            print(f"RECOMMENDER: Scored stores: {recommendations['stores']}")


        elif intent == "find_product":
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
                    print(f"RECOMMENDER: Product search, Store {store_name}, distance {distance:.2f}, vicinity score part {max(0, 100 - distance * 5)}")


                # 2. User profile (shop review)
                shop_review = active_user_profile.get("shop_reviews", {}).get(store_name, {})
                profile_score_part = 0
                if shop_review.get("rating"):
                    profile_score_part += shop_review["rating"] * 10
                if shop_review.get("last_visit_fulfilled") is True:
                    profile_score_part += 20
                store_score += profile_score_part
                print(f"RECOMMENDER: Product search, Store {store_name}, profile score part {profile_score_part}")

                
                # 3. Product relevance/preference
                brand_match_score_part = 0
                num_fav_brand_matches = 0
                for product in products_found:
                    if product["brand"] in active_user_profile.get("favorite_brands", []):
                        num_fav_brand_matches += 1
                brand_match_score_part = num_fav_brand_matches * 5 # Bonus for each favorite brand found
                store_score += brand_match_score_part
                print(f"RECOMMENDER: Product search, Store {store_name}, brand match score part {brand_match_score_part}")


                ranked_store_product_options.append({
                    "store_details": store_details,
                    "products_found": products_found, 
                    "score": store_score,
                    "distance": distance
                })
            
            recommendations["products_in_stores_ranked"] = sorted(ranked_store_product_options, key=lambda x: (x["score"], -x["distance"] if x["distance"] is not None else float('inf')), reverse=True)
            print(f"RECOMMENDER: Ranked product options: {recommendations['products_in_stores_ranked']}")

        return recommendations

if __name__ == '__main__':
    print("--- Testing RecommenderEngine (WITH Distance Scoring) ---")
    # Mock dependencies for testing
    class MockKGService:
        def get_all_store_names(self): return ["Store A", "Store B", "Store C"]
        def get_shop_details(self, name): 
            if name == "Store A": return {"name": "Store A", "map_x": 1.0, "map_y": 1.0, "type": "Apparel"} # Close
            if name == "Store B": return {"name": "Store B", "map_x": 10.0, "map_y": 10.0, "type": "Shoes"} # Far
            if name == "Store C": return {"name": "Store C", "map_x": 2.0, "map_y": 0.0, "type": "Electronics"} # Medium
            return None

    class MockUserProfileManager:
        def get_profile(self, user_name):
            if user_name == "mother":
                return {
                    "favorite_brands": ["BrandX"],
                    "shop_reviews": {
                        "Store A": {"rating": 5, "last_visit_fulfilled": True}, # Good review
                        "Store B": {"rating": 3} # Okay review
                    },
                }
            return {"shop_reviews": {}} 

    kg_mock = MockKGService()
    profile_mock = MockUserProfileManager()
    recommender = RecommenderEngine(kg_mock, profile_mock)

    robot_pose_test = (0.0, 0.0, 0.0) # Robot at origin (x, y, theta_rad)
    mother_profile = profile_mock.get_profile("mother")

    # Test Case 1: Find Store
    print("\n--- Test Case 1: Find Store ---")
    parsed_llm_find_store = {"intent": "find_store"}
    kg_results_stores = {
        "stores": [
            {"id": "SA", "name": "Store A", "map_x": 1.0, "map_y": 1.0, "type": "Apparel", "location_text": "Loc A"}, 
            {"id": "SB", "name": "Store B", "map_x": 10.0, "map_y": 10.0, "type": "Shoes", "location_text": "Loc B"}, 
            {"id": "SC", "name": "Store C", "map_x": 2.0, "map_y": 0.0, "type": "Electronics", "location_text": "Loc C"} 
        ]
    }
    recs_stores = recommender.generate_recommendations(parsed_llm_find_store, kg_results_stores, mother_profile, robot_pose_test)
    print("\nStore Recommendations:")
    for store_rec in recs_stores.get("stores", []):
        dist_str = f", Distance: {store_rec['distance']:.1f}m" if store_rec.get('distance', float('inf')) < float('inf') else ""
        print(f"  - {store_rec['name']} (Score: {store_rec['score']:.0f}{dist_str})")

    # Test Case 2: Find Product
    print("\n--- Test Case 2: Find Product ---")
    parsed_llm_find_product = {"intent": "find_product", "item_types": ["Dress"]}
    kg_results_products = {
        "products_in_stores": {
            "Store A": { 
                "store_details": {"id": "SA", "name": "Store A", "map_x": 1.0, "map_y": 1.0, "type": "Apparel", "location_text": "Loc A"},
                "products_found": [{"name": "Red Dress", "brand": "BrandX"}, {"name": "Blue Dress", "brand": "BrandY"}]
            },
            "Store B": { 
                "store_details": {"id": "SB", "name": "Store B", "map_x": 10.0, "map_y": 10.0, "type": "Shoes", "location_text": "Loc B"},
                "products_found": [{"name": "Green Dress", "brand": "BrandZ"}]
            }
        }
    }
    recs_products = recommender.generate_recommendations(parsed_llm_find_product, kg_results_products, mother_profile, robot_pose_test)
    print("\nProduct Location Recommendations (Stores ranked):")
    for store_option in recs_products.get("products_in_stores_ranked", []):
        dist_str = f", Distance: {store_option['distance']:.1f}m" if store_option.get('distance', float('inf')) < float('inf') else ""
        print(f"  - Store: {store_option['store_details']['name']} (Overall Score: {store_option['score']:.0f}{dist_str}) has {len(store_option['products_found'])} matching products.")