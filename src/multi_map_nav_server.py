#!/usr/bin/env python3
import sys
import argparse
import os
import rospy
import actionlib
from multi_map_nav.msg import MultiMapNavAction, MultiMapNavGoal, MultiMapNavResult, MultiMapNavFeedback

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
from database_handler import DatabaseHandler
from navigation_handler import NavigationHandler, create_pose
from wormhole_data import WormholeData

class MultiMapNavServer:
    
    def __init__(self):
        rospy.loginfo("=== MULTI-MAP NAV SERVER STARTUP ===")
        
        self.database_path = "/home/pete/noetic/anscer_ws/src/multi_map_nav/wormholes.db"
        self.map_base_path = "/home/pete/noetic/anscer_ws/src/AR100/anscer_navigation/maps"
        
        # Initialize components
        self.db_handler = DatabaseHandler(self.database_path)
        if not self.db_handler.test_connection():
            rospy.logerr("Database test failed")
            return
        
        self.db_handler.log_all_wormholes()
        
        try:
            self.nav_handler = NavigationHandler(self.map_base_path)
        except RuntimeError:
            rospy.logerr("Navigation handler initialization failed")
            return
        
        # Initialize action server
        self.action_server = actionlib.SimpleActionServer(
            'multi_map_navigation',
            MultiMapNavAction,
            execute_cb=self.execute_goal,
            auto_start=False
        )
        self.action_server.start()
        
        rospy.loginfo("Multi-Map Navigation Server started successfully")
    
    def execute_goal(self, goal):
        rospy.loginfo("=== NEW GOAL RECEIVED ===")
        rospy.loginfo("Target map: %s", goal.target_map)
        rospy.loginfo("Target pose: (%.2f, %.2f, %.2f)",
                     goal.target_pose.position.x,
                     goal.target_pose.position.y,
                     goal.target_pose.position.z)
        
        # Validate goal
        if not self.is_valid_goal(goal):
            rospy.logerr("Invalid goal parameters")
            self.set_aborted("Invalid goal parameters")
            return
        
        current_map = self.nav_handler.current_map
        rospy.loginfo("Current map: %s, Target map: %s", current_map, goal.target_map)
        
        try:
            if goal.target_map == current_map:
                # Same map navigation
                self.execute_same_map_navigation(goal)
            else:
                # Cross-map navigation
                self.execute_cross_map_navigation(goal, current_map)
                
        except Exception as e:
            rospy.logerr("Goal execution failed: %s", str(e))
            self.set_aborted(f"Navigation failed: {e}")
        
        rospy.loginfo("=== GOAL EXECUTION COMPLETE ===")
    
    def execute_same_map_navigation(self, goal):
        rospy.loginfo("Same map navigation requested")
        self.publish_feedback("Navigating within current map")
        
        success = self.nav_handler.navigate_to_pose(goal.target_pose)
        
        if success:
            rospy.loginfo("Same-map navigation completed successfully")
            self.set_succeeded("Navigation completed successfully")
        else:
            rospy.logerr("Same-map navigation failed")
            self.set_aborted("Navigation failed")
    
    def execute_cross_map_navigation(self, goal, current_map):
        rospy.loginfo("Cross-map navigation requested: %s -> %s", current_map, goal.target_map)
        
        # Find wormhole connection
        self.publish_feedback("Finding wormhole connection")
        wormhole = self.db_handler.get_wormhole(current_map, goal.target_map)
        
        if not wormhole.is_valid():
            rospy.logerr("No wormhole connection found between %s and %s",
                        current_map, goal.target_map)
            self.set_aborted("No wormhole connection found")
            return
        
        rospy.loginfo("Using wormhole for cross-map navigation:")
        wormhole.log_info()
        
        # Navigate to wormhole in source map
        self.publish_feedback("Moving to wormhole")
        wormhole_pose = create_pose(wormhole.source_x, wormhole.source_y)
        
        rospy.loginfo("Navigating to wormhole at (%.2f, %.2f)", wormhole.source_x, wormhole.source_y)
        if not self.nav_handler.navigate_to_pose(wormhole_pose):
            rospy.logerr("Failed to reach wormhole")
            self.set_aborted("Failed to reach wormhole")
            return
        
        rospy.loginfo("Reached wormhole successfully")
        
        # Switch maps
        self.publish_feedback("Switching maps")
        rospy.loginfo("Switching from %s to %s", current_map, goal.target_map)
        
        if not self.nav_handler.switch_map(goal.target_map):
            rospy.logerr("Failed to switch maps")
            self.set_aborted("Failed to switch maps")
            return
        
        rospy.loginfo("Map switch completed successfully")
        
        # Navigate to final destination
        self.publish_feedback("Navigating to final destination")
        rospy.loginfo("Navigating to final destination: (%.2f, %.2f)",
                     goal.target_pose.position.x, goal.target_pose.position.y)
        
        if self.nav_handler.navigate_to_pose(goal.target_pose):
            rospy.loginfo("Multi-map navigation completed successfully")
            self.set_succeeded("Multi-map navigation completed")
        else:
            rospy.logerr("Failed to reach final destination")
            self.set_aborted("Failed to reach final destination")
    
    def is_valid_goal(self, goal):
        valid_maps = {'map1', 'map2'}
        
        if not goal.target_map or goal.target_map not in valid_maps:
            rospy.logerr("Invalid target map: '%s' (must be one of %s)",
                        goal.target_map, valid_maps)
            return False
        
        return True
    
    def publish_feedback(self, status):
        feedback = MultiMapNavFeedback()
        feedback.current_status = status
        self.action_server.publish_feedback(feedback)
        rospy.loginfo("[FEEDBACK] %s", status)
    
    def set_succeeded(self, message):
        result = MultiMapNavResult()
        result.success = True
        result.message = message
        self.action_server.set_succeeded(result)
    
    def set_aborted(self, message):
        result = MultiMapNavResult()
        result.success = False
        result.message = message
        self.action_server.set_aborted(result)
    
    def shutdown(self):
        rospy.loginfo("Shutting down Multi-Map Navigation Server")
        if hasattr(self, 'nav_handler'):
            self.nav_handler.cancel_navigation()


def main():
    help_str = """
Multi-Map Navigation Server for ROS

This server enables navigation across multiple maps using wormhole connections.

Modules used:
    - DatabaseHandler: SQLite wormhole queries
    - NavigationHandler: move_base integration  
    - WormholeData: Connection data structure

Usage:
    rosrun multi_map_nav multi_map_nav_server

Action server: /multi_map_navigation
    """
    
    parser = argparse.ArgumentParser(
        description=help_str,
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    args = parser.parse_args()
    rospy.init_node('multi_map_nav_server_py')
    
    try:
        server = MultiMapNavServer()
        rospy.on_shutdown(server.shutdown)
        rospy.loginfo("Python server ready, waiting for goals...")
        rospy.spin()
    except Exception as e:
        rospy.logerr("Server failed: %s", str(e))
        return 1
    
    return 0


if __name__ == '__main__':
    main()
