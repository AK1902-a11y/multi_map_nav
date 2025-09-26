# Multi-Map Navigation System 

## Overview
A comprehensive ROS-based navigation system that enables seamless robot movement across multiple maps using wormhole connections. Designed for **Anscer Robotics** platform running **ROS Noetic**, this system allows autonomous navigation within single maps and intelligent transitions between different mapped areas through predefined wormhole portals.

## System Specifications
- **Robot Platform**: Anscer Robotics Bot
- **ROS Version**: ROS Noetic (Ubuntu 20.04)
- **Simulation Environment**: Gazebo
- **Database**: SQLite3
- **Navigation Stack**: move_base, AMCL, map_server
- **Programming Language**: Python 3
- **Action Interface**: Custom MultiMapNav action

## Architecture Overview

### Core Components
The system consists of four main Python modules working together:

- **Action Server**: Handles multi-map navigation goals and orchestrates the entire process
- **Database Handler**: Manages SQLite operations for wormhole data queries
- **Navigation Handler**: Interfaces with move_base and handles dynamic map switching
- **Wormhole Data Structure**: Provides data validation and storage for wormhole connections

The wormhole detailes saved in wormhole.db file.

The rooms are mapped seperately and used.

### Module Details

#### `multi_map_nav_server.py` - Main Action Server
- Receives navigation goals with target map and pose
- Validates goal parameters and determines navigation strategy
- Orchestrates same-map vs cross-map navigation workflows
- Publishes feedback and results to action clients

#### `database_handler.py` - SQLite Operations  
- Thread-safe database connections for wormhole queries
- Efficient connection management using context managers
- Wormhole data retrieval and validation
- Database content logging for debugging

#### `navigation_handler.py` - Navigation & Map Switching
- move_base action client integration
- Dynamic map switching (kill/restart nodes)
- Pose creation utilities with quaternion handling
- Navigation goal cancellation and timeout management

#### `wormhole_data.py` - Data Structures
- Wormhole connection data validation
- Position and map name storage
- ROS logging integration for debugging
- Simple data manipulation methods

## Navigation Algorithm

### Multi-Map Navigation Process
The system follows a structured approach for handling navigation goals:

1. **Goal Analysis**: Determine if navigation is within the same map or requires map transition
2. **Same Map Navigation**: Direct move_base goal execution for intra-map movement
3. **Cross-Map Navigation**: 
   - Query database for wormhole connection between source and target maps
   - Navigate to wormhole position in source map
   - Execute map switching (kill current nodes, launch new map_server)
   - Navigate to final destination in target map
4. **Result Reporting**: Return success/failure status with descriptive messages

### Map Switching Mechanism
Dynamic map transitions are handled through:
- Graceful shutdown of current navigation nodes (move_base, map_server)
- Launch of new map_server with target map configuration
- Restart of navigation stack with proper initialization
- Verification of successful map transition before continuing

## Database Schema
- CREATE TABLE wormholes (
- id INTEGER PRIMARY KEY,
- name TEXT ,
- source_map TEXT ,
- target_map TEXT ,
- source_x REAL ,
- source_y REAL ,
- target_x REAL ,
- target_y REAL 
- );

## Installation & Setup

### Prerequisites
- #### Ensure ROS Noetic is installed along with necessary packages for navigation

- #### Ensure sqlite3 is installed

 ```bash
 sudo apt install sqlite3
 ```

- #### Ensure Anscer Robotics AR100 bot is built (https://wiki.ros.org/AnscerRobotics/AR100)

## Execution Instructions

- Clone the package inside src directory in ros workspace along side AR100 package.

- Make sure Ar100 package is built in ros workspace and both the map files, map1 and map2, included in this directory are placed inside `<your-ros-workspace>/AR100/anscer_navigation/maps/<map-files>`

- Build this package inside ros workspace and make sure wormhole.db file is present inside package along with CMake file and package xml file.


#### Build ros workspace:

```bash
cd ~/noetic/anscer_ws #your ros workspace 
catkin_make
source devel/setup.bash
```
#### Execute the following commands to run the naviagtion system:

- run the roscore 
``` bash
cd cd ~/noetic/anscer_ws #your ros workspace 

roscore #start roscore 
```
- run the anscer gazebo environment 
```bash
roslaunch start_anscer start_anscer.launch
```

- run the navigation stack and localilze the robot using 2D pose estimate.
``` bash
roslaunch anscer_navigation anscer_navigation.launch map_name:=<map_name>
```

- run the server for wormhole navigation
```bash
rosrun multi_map_nav multi_map_nav_server.py
```
- To read the help text about run the command with flag --help
```bash
rosrun multi_map_nav multi_map_nav_server.py --help
```

- Give the target goal through cli command
``` bash
rostopic pub -1 /multi_map_navigation/goal multi_map_nav/MultiMapNavActionGoal \
'{goal: {
    target_map: "map2", 
    target_pose: {
        position: {x: 13.94, y: -4.48, z: 0.0}, 
        orientation: {x: 0.0, y: 0.0, z: -0.711, w: 0.703}
    }
}}'
```


