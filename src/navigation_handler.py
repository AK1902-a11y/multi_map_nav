#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import subprocess
import time
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler



class NavigationHandler:
    
    def __init__(self, map_base_path):
        self.map_base_path = map_base_path
        self.current_map = "map1"
        
        # Initialize move_base client
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available after 10 seconds")
            raise RuntimeError("move_base not available")
        
        rospy.loginfo("move_base client ready")
    
    def navigate_to_pose(self, target_pose, timeout=60.0):
        rospy.loginfo(
            "Starting navigation to position (%.2f, %.2f)",
            target_pose.position.x, target_pose.position.y
        )
    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_pose
    
        # Send goal
        self.move_base_client.send_goal(goal)
    
        # Wait for result
        success = self.move_base_client.wait_for_result(rospy.Duration(timeout))
    
        if success:
            state = self.move_base_client.get_state()
            success = (state == GoalStatus.SUCCEEDED)
            rospy.loginfo("Navigation result: %s", str(state))
        else:
            rospy.logwarn("Navigation timeout after %.1f seconds", timeout)
            self.move_base_client.cancel_goal()
    
        return success
    
    def switch_map(self, new_map_name):
        rospy.loginfo("=== MAP SWITCHING ===")
        rospy.loginfo("Switching from '%s' to '%s'", self.current_map, new_map_name)
        
        try:
            # Stop current navigation processes
            rospy.loginfo("Stopping current navigation stack...")
            subprocess.run(['rosnode', 'kill', '/move_base'], capture_output=True, check=False)
            subprocess.run(['rosnode', 'kill', '/map_server'], capture_output=True, check=False)
            time.sleep(2.0)
            
            # Start new map_server
            map_file = f"{self.map_base_path}/{new_map_name}.yaml"
            rospy.loginfo("Loading new map: %s", map_file)
            
            subprocess.Popen([
                'rosrun', 'map_server', 'map_server', map_file, '__name:=map_server'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(3.0)
            
            # Restart navigation stack
            subprocess.Popen([
                'roslaunch', 'anscer_navigation', 'anscer_navigation.launch',
                f'map_name:={new_map_name}'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(2.0)
            
            # Wait for move_base to be available again
            rospy.loginfo("Waiting for move_base to restart...")
            if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
                rospy.logerr("move_base did not restart properly")
                return False
            
            self.current_map = new_map_name
            rospy.loginfo("Map switch completed successfully. Current map: %s", self.current_map)
            rospy.loginfo("=== MAP SWITCHING COMPLETE ===")
            
            return True
            
        except Exception as e:
            rospy.logerr("Map switching failed: %s", str(e))
            return False
    
    def cancel_navigation(self):
        self.move_base_client.cancel_goal()
        rospy.loginfo("Navigation goal cancelled")


def create_pose(x, y, yaw=0.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    
    # Convert yaw to quaternion
    quat = quaternion_from_euler(0, 0, yaw)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    
    return pose
