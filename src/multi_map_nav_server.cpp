
#include <sqlite3.h>

#include <memory>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "multi_map_nav/MultiMapNavAction.h"

namespace multi_map_nav {

struct WormholeData {
  std::string name;
  std::string source_map;
  std::string target_map;
  double source_x;
  double source_y; 
  double target_x;
  double target_y;
  
  WormholeData() = default;
  
  bool IsValid() const {
    return !name.empty();
  }
  
  void Print() const {
    ROS_INFO("Wormhole '%s': %s(%.2f,%.2f) -> %s(%.2f,%.2f)", 
             name.c_str(), source_map.c_str(), source_x, source_y,
             target_map.c_str(), target_x, target_y);
  }
};

class MultiMapNavServer {
 public:
  explicit MultiMapNavServer(ros::NodeHandle& nh);
  ~MultiMapNavServer();

 private:
  using ActionServer = actionlib::SimpleActionServer<MultiMapNavAction>;
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  
  ros::NodeHandle nh_;
  ActionServer action_server_;
  MoveBaseClient move_base_client_;
  
  sqlite3* db_;
  std::string current_map_;
  
  const std::string kActionName_;
  const std::string kDatabasePath_;
  const std::string kMapBasePath_;
  
  // Core functionality with enhanced error handling
  void ExecuteGoal(const MultiMapNavGoalConstPtr& goal);
  bool NavigateToPosition(const geometry_msgs::Pose& target_pose);
  bool SwitchMap(const std::string& new_map_name);
  WormholeData GetWormhole(const std::string& source_map, const std::string& target_map);
  bool InitializeDatabase();
  geometry_msgs::Pose CreatePose(double x, double y, double yaw = 0.0);
  
  // Enhanced utility functions
  bool IsValidGoal(const MultiMapNavGoalConstPtr& goal) const;
  MultiMapNavResult CreateResult(bool success, const std::string& message) const;
  void PublishFeedback(const std::string& status);
  void LogDatabaseContents();
  bool TestDatabaseConnection();
};

MultiMapNavServer::MultiMapNavServer(ros::NodeHandle& nh)
    : nh_(nh),
      action_server_(nh_, "multi_map_navigation", 
                    boost::bind(&MultiMapNavServer::ExecuteGoal, this, _1), 
                    false),
      move_base_client_("move_base", true),
      db_(nullptr),
      current_map_("map1"),
      kActionName_("multi_map_navigation"),
      kDatabasePath_("/home/pete/noetic/anscer_ws/src/multi_map_nav/wormholes.db"),
      kMapBasePath_("/home/pete/noetic/anscer_ws/src/AR100/anscer_navigation/maps/") {
  
  ROS_INFO("=== MULTI-MAP NAV SERVER STARTUP ===");
  ROS_INFO("Database path: %s", kDatabasePath_.c_str());
  ROS_INFO("Map base path: %s", kMapBasePath_.c_str());
  ROS_INFO("Starting map: %s", current_map_.c_str());
  
  if (!InitializeDatabase()) {
    ROS_ERROR("Failed to initialize database connection - ABORTING");
    return;
  }
  
  // Test database with sample query
  if (!TestDatabaseConnection()) {
    ROS_ERROR("Database test failed - ABORTING");
    return;
  }
  
  // Log database contents for debugging
  LogDatabaseContents();
  
  ROS_INFO("Waiting for move_base action server...");
  if (!move_base_client_.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("move_base action server not available after 10 seconds");
    return;
  }
  
  action_server_.start();
  ROS_INFO("Multi-Map Navigation Server started successfully");
  ROS_INFO("=== STARTUP COMPLETE ===");
}

MultiMapNavServer::~MultiMapNavServer() {
  if (db_) {
    sqlite3_close(db_);
    ROS_INFO("Database connection closed");
  }
}

bool MultiMapNavServer::InitializeDatabase() {
  ROS_INFO("Opening database: %s", kDatabasePath_.c_str());
  
  int rc = sqlite3_open(kDatabasePath_.c_str(), &db_);
  if (rc != SQLITE_OK) {
    ROS_ERROR("Cannot open database: %s (error code: %d)", sqlite3_errmsg(db_), rc);
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
    return false;
  }
  
  ROS_INFO("Database opened successfully");
  return true;
}

bool MultiMapNavServer::TestDatabaseConnection() {
  if (!db_) {
    ROS_ERROR("Database connection is NULL");
    return false;
  }
  
  // Simple test query
  const char* test_sql = "SELECT COUNT(*) FROM wormholes;";
  sqlite3_stmt* stmt;
  
  int rc = sqlite3_prepare_v2(db_, test_sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    ROS_ERROR("Test query prepare failed: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    int count = sqlite3_column_int(stmt, 0);
    ROS_INFO("Database test successful - found %d wormhole entries", count);
    sqlite3_finalize(stmt);
    return true;
  }
  
  sqlite3_finalize(stmt);
  ROS_ERROR("Database test failed");
  return false;
}

void MultiMapNavServer::LogDatabaseContents() {
  ROS_INFO("=== DATABASE CONTENTS ===");
  
  const char* sql = "SELECT id, name, source_map, target_map, source_x, source_y, target_x, target_y FROM wormholes;";
  sqlite3_stmt* stmt;
  
  int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    ROS_ERROR("Failed to prepare database listing query: %s", sqlite3_errmsg(db_));
    return;
  }
  
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    int id = sqlite3_column_int(stmt, 0);
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    const char* src_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    const char* tgt_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
    double src_x = sqlite3_column_double(stmt, 4);
    double src_y = sqlite3_column_double(stmt, 5);
    double tgt_x = sqlite3_column_double(stmt, 6);
    double tgt_y = sqlite3_column_double(stmt, 7);
    
    ROS_INFO("ID:%d '%s': %s(%.2f,%.2f) -> %s(%.2f,%.2f)", 
             id, name, src_map, src_x, src_y, tgt_map, tgt_x, tgt_y);
  }
  
  sqlite3_finalize(stmt);
  ROS_INFO("=== END DATABASE CONTENTS ===");
}

void MultiMapNavServer::ExecuteGoal(const MultiMapNavGoalConstPtr& goal) {
  ROS_INFO("=== NEW GOAL RECEIVED ===");
  ROS_INFO("Target map: %s", goal->target_map.c_str());
  ROS_INFO("Target pose: (%.2f, %.2f, %.2f)", 
           goal->target_pose.position.x, 
           goal->target_pose.position.y,
           goal->target_pose.position.z);
  
  if (!IsValidGoal(goal)) {
    ROS_ERROR("Invalid goal parameters");
    action_server_.setAborted(CreateResult(false, "Invalid goal parameters"));
    return;
  }
  
  ROS_INFO("Current map: %s, Target map: %s", current_map_.c_str(), goal->target_map.c_str());
  
  // Same map navigation
  if (goal->target_map == current_map_) {
    ROS_INFO("Same map navigation requested");
    PublishFeedback("Navigating within current map");
    
    bool success = NavigateToPosition(goal->target_pose);
    
    if (success) {
      ROS_INFO("Same-map navigation completed successfully");
      action_server_.setSucceeded(CreateResult(true, "Navigation completed successfully"));
    } else {
      ROS_ERROR("Same-map navigation failed");
      action_server_.setAborted(CreateResult(false, "Navigation failed"));
    }
    return;
  }
  
  // Different map navigation - use wormhole
  ROS_INFO("Cross-map navigation requested: %s -> %s", current_map_.c_str(), goal->target_map.c_str());
  PublishFeedback("Finding wormhole connection");
  
  WormholeData wormhole = GetWormhole(current_map_, goal->target_map);
  
  ROS_INFO("Wormhole search completed");
  
  if (!wormhole.IsValid()) {
    ROS_ERROR("No wormhole connection found between %s and %s", 
              current_map_.c_str(), goal->target_map.c_str());
    action_server_.setAborted(CreateResult(false, "No wormhole connection found"));
    return;
  }
  
  ROS_INFO("Using wormhole for cross-map navigation:");
  wormhole.Print();
  
  // Navigate to wormhole in source map
  PublishFeedback("Moving to wormhole");
  geometry_msgs::Pose wormhole_pose = CreatePose(wormhole.source_x, wormhole.source_y);
  
  ROS_INFO("Navigating to wormhole at (%.2f, %.2f)", wormhole.source_x, wormhole.source_y);
  if (!NavigateToPosition(wormhole_pose)) {
    ROS_ERROR("Failed to reach wormhole");
    action_server_.setAborted(CreateResult(false, "Failed to reach wormhole"));
    return;
  }
  
  ROS_INFO("Reached wormhole successfully");
  
  // Switch maps
  PublishFeedback("Switching maps");
  ROS_INFO("Switching from %s to %s", current_map_.c_str(), goal->target_map.c_str());
  
  if (!SwitchMap(goal->target_map)) {
    ROS_ERROR("Failed to switch maps");
    action_server_.setAborted(CreateResult(false, "Failed to switch maps"));
    return;
  }
  
  ROS_INFO("Map switch completed successfully");
  
  // Navigate to final goal in target map
  PublishFeedback("Navigating to final destination");
  ROS_INFO("Navigating to final destination: (%.2f, %.2f)", 
           goal->target_pose.position.x, goal->target_pose.position.y);
  
  if (NavigateToPosition(goal->target_pose)) {
    ROS_INFO("Multi-map navigation completed successfully");
    action_server_.setSucceeded(CreateResult(true, "Multi-map navigation completed"));
  } else {
    ROS_ERROR("Failed to reach final destination");
    action_server_.setAborted(CreateResult(false, "Failed to reach final destination"));
  }
  
  ROS_INFO("=== GOAL EXECUTION COMPLETE ===");
}

bool MultiMapNavServer::NavigateToPosition(const geometry_msgs::Pose& target_pose) {
  ROS_INFO("Starting navigation to position (%.2f, %.2f)", 
           target_pose.position.x, target_pose.position.y);
  
  move_base_msgs::MoveBaseGoal move_goal;
  move_goal.target_pose.header.frame_id = "map";
  move_goal.target_pose.header.stamp = ros::Time::now();
  move_goal.target_pose.pose = target_pose;
  
  move_base_client_.sendGoal(move_goal);
  
  // Wait with periodic feedback
  ros::Rate feedback_rate(2.0); // 2 Hz feedback
  ros::Time start_time = ros::Time::now();
  
  while (!move_base_client_.waitForResult(ros::Duration(0.5))) {
    // Check for preemption
    if (action_server_.isPreemptRequested()) {
      ROS_WARN("Action preempted during navigation");
      move_base_client_.cancelGoal();
      action_server_.setPreempted(CreateResult(false, "Action preempted"));
      return false;
    }
    
    ros::Duration elapsed = ros::Time::now() - start_time;
    std::string feedback_msg = "Navigating... (" + std::to_string((int)elapsed.toSec()) + "s)";
    PublishFeedback(feedback_msg);
    
    feedback_rate.sleep();
  }
  
  actionlib::SimpleClientGoalState state = move_base_client_.getState();
  ROS_INFO("Navigation result: %s", state.toString().c_str());
  
  return state == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool MultiMapNavServer::SwitchMap(const std::string& new_map_name) {
  ROS_INFO("=== MAP SWITCHING ===");
  ROS_INFO("Switching from '%s' to '%s'", current_map_.c_str(), new_map_name.c_str());
  
  ROS_INFO("Stopping current navigation stack...");
  system("rosnode kill /move_base 2>/dev/null");
  system("rosnode kill /map_server 2>/dev/null");
  ros::Duration(2.0).sleep();
 
  std::string map_file = kMapBasePath_ + new_map_name + ".yaml";
  ROS_INFO("Loading new map: %s", map_file.c_str());
  
  std::string cmd = "rosrun map_server map_server " + map_file + " __name:=map_server &";
  ROS_INFO("Executing: %s", cmd.c_str());
  
  int result = system(cmd.c_str());
  if (result != 0) {
    ROS_ERROR("Failed to start map_server (exit code: %d)", result);
    return false;
  }
  
  ros::Duration(3.0).sleep();
  
  std::string move_base_cmd = "roslaunch anscer_navigation anscer_navigation.launch map_name:=" + new_map_name + " &";
  ROS_INFO("Restarting navigation: %s", move_base_cmd.c_str());
  system(move_base_cmd.c_str());
  
  ros::Duration(2.0).sleep();
  
  ROS_INFO("Waiting for move_base to restart...");
  if (!move_base_client_.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("move_base did not restart properly");
    return false;
  }
  
  current_map_ = new_map_name;
  ROS_INFO("Map switch completed successfully. Current map: %s", current_map_.c_str());
  ROS_INFO("=== MAP SWITCHING COMPLETE ===");
  
  return true;
}

WormholeData MultiMapNavServer::GetWormhole(const std::string& source_map, 
                                           const std::string& target_map) {
  WormholeData wormhole;
  
  ROS_INFO("=== WORMHOLE SEARCH ===");
  ROS_INFO("Searching: %s -> %s", source_map.c_str(), target_map.c_str());
  ROS_INFO("Database connection: %s", db_ ? "VALID" : "NULL");
  
  if (!db_) {
    ROS_ERROR("Database connection is NULL!");
    return wormhole;
  }
  
  std::string sql = "SELECT name, source_x, source_y, target_x, target_y FROM wormholes WHERE source_map='" 
                   + source_map + "' AND target_map='" + target_map + "';";
  
  ROS_INFO("SQL Query: %s", sql.c_str());
  
  sqlite3_stmt* stmt;
  int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
  
  ROS_INFO("sqlite3_prepare_v2 result: %d (%s)", rc, rc == SQLITE_OK ? "OK" : "ERROR");
  
  if (rc != SQLITE_OK) {
    ROS_ERROR("SQL prepare failed: %s", sqlite3_errmsg(db_));
    return wormhole;
  }
  
  int step_result = sqlite3_step(stmt);
  ROS_INFO("sqlite3_step result: %d (SQLITE_ROW=%d, SQLITE_DONE=%d)", 
           step_result, SQLITE_ROW, SQLITE_DONE);
  
  if (step_result == SQLITE_ROW) {
    wormhole.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    wormhole.source_map = source_map;
    wormhole.target_map = target_map;
    wormhole.source_x = sqlite3_column_double(stmt, 1);
    wormhole.source_y = sqlite3_column_double(stmt, 2);
    wormhole.target_x = sqlite3_column_double(stmt, 3);
    wormhole.target_y = sqlite3_column_double(stmt, 4);
    
    ROS_INFO("FOUND WORMHOLE:");
    wormhole.Print();
  } else if (step_result == SQLITE_DONE) {
    ROS_WARN("No wormhole found matching the query");
  } else {
    ROS_ERROR("sqlite3_step failed with error: %d", step_result);
  }
  
  sqlite3_finalize(stmt);
  ROS_INFO("=== WORMHOLE SEARCH COMPLETE ===");
  
  return wormhole;
}

geometry_msgs::Pose MultiMapNavServer::CreatePose(double x, double y, double yaw) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  pose.orientation = tf2::toMsg(quat);
  
  return pose;
}

bool MultiMapNavServer::IsValidGoal(const MultiMapNavGoalConstPtr& goal) const {
  bool valid = !goal->target_map.empty() && 
               (goal->target_map == "map1" || goal->target_map == "map2");
  
  if (!valid) {
    ROS_ERROR("Invalid goal: map='%s' (must be 'map1' or 'map2')", goal->target_map.c_str());
  }
  
  return valid;
}

MultiMapNavResult MultiMapNavServer::CreateResult(bool success, const std::string& message) const {
  MultiMapNavResult result;
  result.success = success;
  result.message = message;
  return result;
}

void MultiMapNavServer::PublishFeedback(const std::string& status) {
  MultiMapNavFeedback feedback;
  feedback.current_status = status;
  action_server_.publishFeedback(feedback);
  ROS_INFO("[FEEDBACK] %s", status.c_str());
}

}  // namespace multi_map_nav

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_map_nav_server");
  ros::NodeHandle nh;
  
  try {
    multi_map_nav::MultiMapNavServer server(nh);
    ROS_INFO("Server ready, waiting for goals...");
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Server failed: %s", e.what());
    return 1;
  }
  
  return 0;
}
