#!/usr/bin/env python

import sys
import os
import math

# Assuming robot_cmd_ros.py is in the same directory
# No sys.path modification needed if they are co-located.

try:
    from robot_cmd_ros import *
except ImportError:
    print "Failed to import robot_cmd_ros."
    print "Ensure 'robot_cmd_ros.py' is in the SAME directory as 'navigation_example.py'."
    sys.exit(1)

def navigate_to_goal(x, y, theta_degrees):
    """
    Navigates the robot to the specified goal in the map frame.
    x, y: target coordinates in meters.
    theta_degrees: target orientation in degrees.
    """
    if not marrtinoOK():
        print "MARRtino system not ready for navigation."
        return False

    # Python 2 compatible print statement
    print "Attempting to navigate to: x={}, y={}, theta={} degrees".format(x, y, theta_degrees)

    # The goto function is blocking. It will return True on success, False on failure.
    success = goto(x, y, theta_degrees)

    if success:
        print "Navigation successful!"
        current_pose = getRobotPose('map') # 'map' frame for localized pose
        if current_pose:
            # Python 2 compatible print statement with formatting
            print "Current pose after navigation: x={:.2f}, y={:.2f}, theta={:.2f} degrees".format(current_pose[0], current_pose[1], math.degrees(current_pose[2]))
    else:
        print "Navigation failed or was interrupted."
    return success

if __name__ == "__main__":
    try:
        # Initialize MARRtino interface
        # The nodename should be unique if multiple scripts are run simultaneously
        begin(nodename='my_navigation_controller_py2', init_node=True) # Slightly different nodename for clarity

        if not marrtinoOK():
            print "MARRtino system not ready after begin(). Exiting."
            # It might take a moment for all ROS services to be available
            wait(2) # wait is a function in robot_cmd_ros
            if not marrtinoOK():
                 sys.exit(1)

        print "MARRtino system initialized and ready for commands."

        # --- Define your target goal(s) here ---
        # These coordinates are in the 'map' frame and depend on YOUR MAP!

        # Example Goal 1 (adjust these values for your map!)
        target_x1 = 1.0  # meters
        target_y1 = 0.5  # meters
        target_theta_deg1 = 90.0 # degrees

        # Example Goal 2
        target_x2 = -1.0
        target_y2 = -0.8
        target_theta_deg2 = 0.0

        # Perform navigation to the first goal
        print "\n--- Navigating to Goal 1 ---"
        if navigate_to_goal(target_x1, target_y1, target_theta_deg1):
            print "Reached Goal 1. Waiting for a few seconds..."
            wait(3) # wait is a function in robot_cmd_ros

            # Perform navigation to the second goal
            print "\n--- Navigating to Goal 2 ---"
            navigate_to_goal(target_x2, target_y2, target_theta_deg2)
            print "Reached Goal 2 or failed."
        else:
            print "Could not reach Goal 1."

        print "\nNavigation sequence finished."

    except KeyboardInterrupt:
        print "\nProgram interrupted by user (Ctrl+C)."
    except Exception as e:
        # Python 2 compatible print statement for exception
        print "An unexpected error occurred: {}".format(e)
    finally:
        # Clean up MARRtino interface
        print "Shutting down MARRtino interface."
        stop() # Ensure the robot stops
        end()
        print "Program finished."