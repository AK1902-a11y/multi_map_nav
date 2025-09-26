#!/usr/bin/env python3
"""Database handler for wormhole data storage and retrieval."""

import sqlite3
import rospy
from wormhole_data import WormholeData


class DatabaseHandler:
    
    def __init__(self, database_path):
        self.database_path = database_path
        self.test_connection()
    
    def _get_connection(self):
        try:
            connection = sqlite3.connect(self.database_path)
            return connection
        except sqlite3.Error as e:
            rospy.logerr("Database connection failed: %s", str(e))
            return None
    
    def close(self):
        rospy.loginfo("Database handler closed")
    
    def test_connection(self):
        connection = self._get_connection()
        if not connection:
            return False
            
        try:
            cursor = connection.cursor()
            cursor.execute("SELECT COUNT(*) FROM wormholes")
            count = cursor.fetchone()[0]
            rospy.loginfo("Database test successful - found %d wormhole entries", count)
            connection.close()
            return True
        except sqlite3.Error as e:
            rospy.logerr("Database test failed: %s", str(e))
            connection.close()
            return False
    
    def get_wormhole(self, source_map, target_map):
        rospy.loginfo("=== WORMHOLE SEARCH ===")
        rospy.loginfo("Searching: %s -> %s", source_map, target_map)
        
        wormhole = WormholeData()
        connection = self._get_connection()
        
        if not connection:
            rospy.logerr("Failed to get database connection")
            return wormhole
        
        try:
            cursor = connection.cursor()
            sql = "SELECT name, source_x, source_y, target_x, target_y FROM wormholes WHERE source_map=? AND target_map=?"
            rospy.loginfo("SQL Query: %s with params (%s, %s)", sql, source_map, target_map)
            
            cursor.execute(sql, (source_map, target_map))
            row = cursor.fetchone()
            
            if row:
                wormhole.set_data(
                    row[0],        # name
                    source_map,    # source_map
                    target_map,    # target_map
                    row[1],        # source_x
                    row[2],        # source_y
                    row[3],        # target_x
                    row[4]         # target_y
                )
                rospy.loginfo("FOUND WORMHOLE:")
                wormhole.log_info()
            else:
                rospy.logwarn("No wormhole found matching the query")
                
        except sqlite3.Error as e:
            rospy.logerr("Database query failed: %s", str(e))
        finally:
            connection.close()
        
        rospy.loginfo("=== WORMHOLE SEARCH COMPLETE ===")
        return wormhole
    
    def log_all_wormholes(self):
        rospy.loginfo("=== DATABASE CONTENTS ===")
        
        connection = self._get_connection()
        if not connection:
            rospy.logerr("Failed to get database connection")
            return
        
        try:
            cursor = connection.cursor()
            cursor.execute("SELECT id, name, source_map, target_map, source_x, source_y, target_x, target_y FROM wormholes")
            
            for row in cursor.fetchall():
                rospy.loginfo(
                    "ID:%d '%s': %s(%.2f,%.2f) -> %s(%.2f,%.2f)",
                    row[0], row[1], row[2], row[4], row[5], row[3], row[6], row[7]
                )
                
        except sqlite3.Error as e:
            rospy.logerr("Failed to retrieve database contents: %s", str(e))
        finally:
            connection.close()
        
        rospy.loginfo("=== END DATABASE CONTENTS ===")
