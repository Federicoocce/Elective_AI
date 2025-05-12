#!/usr/bin/env python

import rospy
import yaml
import os
import tf
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

# Import your modules
from graph_data_generator import create_mall_graph
from mall_query_engine import parse_request_to_structured_query, execute_structured_query, process_natural_language_request

class MallNavigator:
    def __init__(self):
        rospy.init_node('mall_navigator')
        
        # Initialize parameters
        self.map_path = rospy.get_param('~map_path', os.path.join(os.environ.get('MARRTINO_APPS_HOME', ''), 'navigation/maps'))
        self.semantic_map_file = rospy.get_param('~semantic_map_file', 'DISB1.yaml')
        
        # Load semantic map
        self.semantic_map = self._load_semantic_map()
        
        # Create knowledge graph
        self.mall_graph = create_mall_graph()
        rospy.loginfo("Mall knowledge graph created")
        
        # Initialize navigation client
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # TF listener for robot position
        self.tf_listener = tf.TransformListener()
        
        # Initialize publishers and subscribers
        self.status_pub = rospy.Publisher('/mall_navigation/status', String, queue_size=10)
        self.marker_pub = rospy.Publisher('/mall_navigation/markers', MarkerArray, queue_size=10)
        
        # Subscribe to mother's requests
        self.request_sub = rospy.Subscriber('/mall_navigation/mother_request', String, self.handle_mother_request)
        
        # Navigation state variables
        self.current_plan = []
        self.current_store_index = 0
        self.navigation_active = False
        
        # Navigation progress checker
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_navigation_progress)
        
        # Publish store markers
        self._publish_store_markers()
        
        rospy.loginfo("Mall Navigator initialized and ready")
    
    def _load_semantic_map(self):
        """Load semantic map from YAML file"""
        map_file = os.path.join(self.map_path, self.semantic_map_file)
        try:
            with open(map_file, 'r') as f:
                map_data = yaml.safe_load(f)
                rospy.loginfo(f"Loaded semantic map from {map_file}")
                return map_data
        except Exception as e:
            rospy.logerr(f"Failed to load semantic map: {e}")
            return {}
    
    def handle_mother_request(self, msg):
        """Handle the mother's request from voice or text input"""
        request_text = msg.data.strip()
        rospy.loginfo(f"Received mother's request: '{request_text}'")
        
        # Check if it's a direct store request
        if self._is_direct_store_request(request_text):
            self._handle_direct_store_request(request_text)
        else:
            # It's a product search request
            self._handle_product_search_request(request_text)
    
    def _is_direct_store_request(self, request_text):
        """Check if the request is for a direct store navigation"""
        request_lower = request_text.lower()
        direct_patterns = [
            "take me to", "go to", "find", "where is", "show me", "navigate to"
        ]
        
        for pattern in direct_patterns:
            if pattern in request_lower:
                # Check if it matches a store name
                store_part = request_lower.split(pattern, 1)[1].strip()
                for store_id, store_data in self.semantic_map.items():
                    if isinstance(store_data, dict) and store_data.get('type') == 'store':
                        store_name = store_data.get('kg_name', '').lower()
                        if store_name and store_name in store_part:
                            return True
        
        return False
    
    def _handle_direct_store_request(self, request_text):
        """Handle a request to go directly to a specific store"""
        request_lower = request_text.lower()
        
        # Extract store name
        store_name = None
        for pattern in ["take me to", "go to", "find", "where is", "show me", "navigate to"]:
            if pattern in request_lower:
                store_part = request_lower.split(pattern, 1)[1].strip()
                for store_id, store_data in self.semantic_map.items():
                    if isinstance(store_data, dict) and store_data.get('type') == 'store':
                        kg_name = store_data.get('kg_name', '').lower()
                        if kg_name and kg_name in store_part:
                            store_name = kg_name
                            target_store = store_data
                            break
                if store_name:
                    break
        
        if not store_name:
            self.status_pub.publish("I couldn't identify which store you want to visit.")
            return
        
        # Create a navigation plan with just this store
        self.current_plan = [target_store]
        self.current_store_index = 0
        self.navigation_active = True
        
        # Start navigation
        self._navigate_to_current_store()
        self.status_pub.publish(f"Taking you to {store_name}")
    
    def _handle_product_search_request(self, request_text):
        """Handle a request to find products"""
        # Use the mall_query_engine to parse the request
        structured_query = parse_request_to_structured_query(request_text)
        
        if not structured_query:
            self.status_pub.publish("I couldn't understand what products you're looking for.")
            return
        
        # Execute the query to find matching stores
        results = execute_structured_query(self.mall_graph, structured_query)
        
        if not results:
            # No results found
            item_classes = ", ".join(structured_query.get("item_base_classes", []))
            attributes = ""
            if structured_query.get("attributes"):
                attrs = [f"{k}: {v}" for k, v in structured_query.get("attributes", {}).items()]
                attributes = f" with {', '.join(attrs)}"
            
            self.status_pub.publish(f"Sorry, I couldn't find any stores selling {item_classes}{attributes}.")
            return
        
        # Map store names to store data from semantic map
        matching_stores = []
        for store_name in results.keys():
            for store_id, store_data in self.semantic_map.items():
                if isinstance(store_data, dict) and store_data.get('type') == 'store' and store_data.get('kg_name') == store_name:
                    matching_stores.append(store_data)
                    break
        
        if not matching_stores:
            self.status_pub.publish("Found matching products but couldn't locate the stores in the map.")
            return
        
        # Plan route to stores
        current_pose = self._get_current_pose()
        self.current_plan = self._plan_route(current_pose, matching_stores)
        self.current_store_index = 0
        self.navigation_active = True
        
        # Start navigation
        self._navigate_to_current_store()
        
        # Publish status message
        store_names = [s.get('kg_name', 'Unknown') for s in self.current_plan]
        items_desc = ", ".join(structured_query.get("item_base_classes", []))
        self.status_pub.publish(f"Found {len(self.current_plan)} stores selling {items_desc}. Taking you to {store_names[0]}.")
    
    def _get_current_pose(self):
        """Get current robot position from TF"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return [trans[0], trans[1]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get current position: {e}")
            return [2, 2]  # Default position
    
    def _plan_route(self, current_position, stores):
        """Plan an efficient route through stores based on distance"""
        # Calculate distances from current position
        stores_with_distance = []
        for store in stores:
            gotopose = store.get('gotopose')
            if not gotopose:
                continue
                
            dx = gotopose[0] - current_position[0]
            dy = gotopose[1] - current_position[1]
            distance = (dx**2 + dy**2)**0.5
            stores_with_distance.append((store, distance))
        
        # Sort by distance
        return [s for s, _ in sorted(stores_with_distance, key=lambda x: x[1])]
    
    def _navigate_to_current_store(self):
        """Navigate to the current store in the plan"""
        if not self.navigation_active or not self.current_plan:
            return
            
        if self.current_store_index >= len(self.current_plan):
            self._complete_navigation()
            return
            
        current_store = self.current_plan[self.current_store_index]
        gotopose = current_store.get('gotopose')
        
        if not gotopose:
            rospy.logwarn(f"No gotopose found for store {current_store.get('kg_name', 'Unknown')}")
            self.current_store_index += 1
            self._navigate_to_current_store()
            return
        
        # Create and send navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = gotopose[0]
        goal.target_pose.pose.position.y = gotopose[1]
        
        # Set orientation (convert from degrees to quaternion)
        yaw_rad = gotopose[2] * 3.14159/180.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        
        # Send the goal
        store_name = current_store.get('kg_name', f"Store {self.current_store_index+1}")
        rospy.loginfo(f"Navigating to {store_name}")
        self.move_client.send_goal(goal)
    
    def check_navigation_progress(self, event):
        """Periodically check navigation progress"""
        if not self.navigation_active:
            return
            
        state = self.move_client.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            # Reached the current store
            current_store = self.current_plan[self.current_store_index]
            store_name = current_store.get('kg_name', f"Store {self.current_store_index+1}")
            
            # Announce arrival
            self.status_pub.publish(f"Arrived at {store_name}")
            
            # Announce available products if this was a product search
            if self.current_store_index == 0:  # First store, announce what we found
                store_id = current_store.get('kg_id')
                self._announce_store_products(store_id)
            
            # Move to next store after a delay
            rospy.sleep(2.0)
            self.current_store_index += 1
            
            if self.current_store_index < len(self.current_plan):
                next_store = self.current_plan[self.current_store_index]
                next_name = next_store.get('kg_name', f"Store {self.current_store_index+1}")
                self.status_pub.publish(f"Now taking you to {next_name}")
                self._navigate_to_current_store()
            else:
                self._complete_navigation()
        
        elif state == GoalStatus.ABORTED or state == GoalStatus.REJECTED or state == GoalStatus.PREEMPTED:
            # Navigation failed
            current_store = self.current_plan[self.current_store_index]
            store_name = current_store.get('kg_name', f"Store {self.current_store_index+1}")
            
            rospy.logwarn(f"Failed to reach {store_name}. Skipping to next store.")
            self.status_pub.publish(f"Could not reach {store_name}. Moving on to the next store.")
            
            # Skip to next store
            self.current_store_index += 1
            if self.current_store_index < len(self.current_plan):
                self._navigate_to_current_store()
            else:
                self._complete_navigation()
    
    def _announce_store_products(self, store_id):
        """Announce products available at the current store"""
        # Find products in the knowledge graph
        products = []
        
        for node_id, data in self.mall_graph.nodes(data=True):
            if data.get('label_node') == 'Product':
                # Check if there's an edge to this store
                for _, neighbor in self.mall_graph.out_edges(node_id):
                    neighbor_data = self.mall_graph.nodes[neighbor]
                    if (neighbor_data.get('label_node') == 'Store' and 
                        neighbor == store_id):
                        products.append(data)
                        break
        
        if products:
            product_names = [p.get('product_name', 'Unknown') for p in products[:5]]
            if len(products) > 5:
                product_names.append(f"and {len(products) - 5} more items")
            
            product_list = ", ".join(product_names)
            self.status_pub.publish(f"This store sells: {product_list}")
    
    def _complete_navigation(self):
        """Handle completion of the navigation plan"""
        self.navigation_active = False
        self.current_plan = []
        self.current_store_index = 0
        
        self.status_pub.publish("Shopping tour completed. Let me know if you need anything else.")
    
    def _publish_store_markers(self):
        """Publish visualization markers for stores"""
        marker_array = MarkerArray()
        marker_id = 0
        
        for store_id, store_data in self.semantic_map.items():
            if not isinstance(store_data, dict) or store_data.get('type') != 'store':
                continue
            
            # Skip if no pose data
            pose = store_data.get('pose')
            if not pose or len(pose) < 4:
                continue
            
            # Create store volume marker
            volume_marker = Marker()
            volume_marker.header.frame_id = 'map'
            volume_marker.header.stamp = rospy.Time.now()
            volume_marker.ns = 'store_volumes'
            volume_marker.id = marker_id
            marker_id += 1
            volume_marker.type = Marker.CUBE
            volume_marker.action = Marker.ADD
            
            # Set position
            volume_marker.pose.position.x = pose[0]
            volume_marker.pose.position.y = pose[1]
            volume_marker.pose.position.z = pose[2] if len(pose) > 2 else 1.0
            
            # Set orientation
            yaw_rad = pose[3] * 3.14159/180.0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
            volume_marker.pose.orientation.x = quaternion[0]
            volume_marker.pose.orientation.y = quaternion[1]
            volume_marker.pose.orientation.z = quaternion[2]
            volume_marker.pose.orientation.w = quaternion[3]
            
            # Set scale
            size = store_data.get('size', [1.0, 1.0, 1.0])
            volume_marker.scale.x = size[0]
            volume_marker.scale.y = size[1]
            volume_marker.scale.z = size[2] if len(size) > 2 else 2.0
            
            # Set color
            color_name = store_data.get('color', 'Blue')
            r, g, b = 0.0, 0.0, 1.0  # Default blue
            
            if color_name == 'Red': r, g, b = 1.0, 0.0, 0.0
            elif color_name == 'Green': r, g, b = 0.0, 1.0, 0.0
            elif color_name == 'Blue': r, g, b = 0.0, 0.0, 1.0
            elif color_name == 'Yellow': r, g, b = 1.0, 1.0, 0.0
            elif color_name == 'Magenta': r, g, b = 1.0, 0.0, 1.0
            elif color_name == 'Cyan': r, g, b = 0.0, 1.0, 1.0
            elif color_name == 'Orange': r, g, b = 1.0, 0.5, 0.0
            elif color_name == 'Purple': r, g, b = 0.5, 0.0, 0.5
            elif color_name == 'Pink': r, g, b = 1.0, 0.4, 0.7
            elif color_name == 'Brown': r, g, b = 0.6, 0.4, 0.2
            elif color_name == 'Grey' or color_name == 'Gray': r, g, b = 0.5, 0.5, 0.5
            elif color_name == 'DarkGreen': r, g, b = 0.0, 0.5, 0.0
            elif color_name == 'LightBlue': r, g, b = 0.4, 0.7, 1.0
            elif color_name == 'Beige': r, g, b = 0.96, 0.96, 0.86
            
            volume_marker.color.r = r
            volume_marker.color.g = g
            volume_marker.color.b = b
            volume_marker.color.a = 0.6  # Semi-transparent
            
            # Create text marker for store name
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = 'store_names'
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = pose[0]
            text_marker.pose.position.y = pose[1]
            text_marker.pose.position.z = 2.5  # Above the store
            
            text_marker.text = store_data.get('kg_name', store_id)
            text_marker.scale.z = 0.4  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(volume_marker)
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        navigator = MallNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass