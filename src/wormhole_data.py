#!/usr/bin/env python3
import rospy

class WormholeData:
    
    def __init__(self):
        """Initialize empty wormhole data."""
        self.name = ""
        self.source_map = ""
        self.target_map = ""
        self.source_x = 0.0
        self.source_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
    
    def is_valid(self):
        return bool(self.name.strip())
    
    def log_info(self):
        rospy.loginfo(
            "Wormhole '%s': %s(%.2f,%.2f) -> %s(%.2f,%.2f)",
            self.name, self.source_map, self.source_x, self.source_y,
            self.target_map, self.target_x, self.target_y
        )
    
    def set_data(self, name, source_map, target_map, source_x, source_y, target_x, target_y):
        self.name = name
        self.source_map = source_map
        self.target_map = target_map
        self.source_x = source_x
        self.source_y = source_y
        self.target_x = target_x
        self.target_y = target_y
