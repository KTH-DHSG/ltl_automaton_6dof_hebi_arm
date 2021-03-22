# ltl_automaton_6dof_hebi_arm
Provides integration of the 6-DoF Hebi manipulator with LTL automaton package

## Installation

### Dependencies
- [ltl_automaton_core](https://github.com/KTH-SML/ltl_automaton_core). Core ROS package for LTL.

- [HEBI-ROS](http://wiki.ros.org/hebiros). HEBI Robotics ROS package.
  - For ROS Kinetic, install using `sudo apt install ros-kinetic-hebiros`
  - For ROS Melodic, Noetic or more recent, build the following packages from source:
    - [https://github.com/HebiRobotics/HEBI-ROS-DEPRECATED](https://github.com/HebiRobotics/HEBI-ROS-DEPRECATED)

- [PyYAML](https://pyyaml.org/). Should be integrated with ROS but it's better to check if version is up-to-date.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install pyyaml`
	- For Python3 (ROS Morenia):
	`pip3 install pyyaml`
  
### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/KTH-SML/ltl_automaton_6dof_hebi_arm
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage
The package provides the agent-level code needed for interacting with the 6-DoF Hebi manipulator.

To launch the planner and LTL Hebi arm node, simply run the following command.

```
roslaunch ltl_automaton_6dof_hebi_arm ltl_6dof_hebi.launch
```

This will run the planner using the task specification in `config/6dof_hebi_ltl_formula.yaml` and transition system in `config/6dof_hebi_ts.yaml`.

### Transition system and actions
The robot transition system needs to be of the following type: `[6d_jointspace_region", "hebi_load]`. The robot can carry the following actions:
- `goto_<region>` (from *6d_jointspace_region*): Go to the defined regions. The action needs to be part of the transition system textfile with the following attributes
  
  ```Python
  attr:
  	jointposition: [0.1, 1.1, 2.1, 2.8, -1.5, 0.4] # Joint values for the 6 joints, following hebiros group name order
  ```
  
- `pick` (from *hebi_load*): Perform pick up action. The manipulator will wait for one of the agent defined in `pick_agent_list` to be on the region defined in `pick_region` and have in its TS state one of the state from `pick_agent_load_state_guard` before moving. Confirmation of finished action is sent on `pick_ack` topic.

- `drop` (from *hebi_load*): Perform drop action. The manipulator will wait for one of the agent defined in `drop_agent_list` to be on the region defined in `drop_region` and have in its TS state `unloaded` before moving. Confirmation of finished action is sent on `drop_ack` topic.


## Config files
- **6dof_hebi_ltl_formula.yaml** Example of LTL formula with both hard and soft task.

- **6dof_hebi_ts.yaml** Example of LTL transition system definition.

## Launch files
- **ltl_6dof_hebi.launch** Example of the LTL planner implementation with the 6-DoF Hebi manipulator as agent. Run the planner node and 6-DoF Hebi manipulator LTL node with an example TS (Transition System) and example LTL formula.
    - `agent_name` Agent name. Default: `kth_hebi_arm`.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `false`.
    - `pick_agent_list` List of agent to pick up from. Use to check if any agent is in the pick up region before pick up action. Default: `[nexus]`.
    - `pick_agent_load_state_guard` List of accepted state of pick up agent. Use to check if agent to pick up from is in the good state before pick up action. Default: `[loaded]`.
    - `pick_region` Pick up region. The agent to pick up from should be in the given region (2d_pose_region). Default: "s2"
    - `drop_agent_list` List of agent to deliver to. Use to check if any agent is in the drop region before drop action. Default: `[turtlebot]`
    - `drop_region` Drop region. The agent to deliver to should be in the given region (2d_pose_region). Default: "s2"

## Nodes
### ltl_automaton_hebi_node.py
LTL 6-DoF Hebi manipulator node, execute the action sent by the LTL planner and returns the aggregated TS state from the state monitors.

#### Actions
*Action published topics*
- `hebiros/arm_group/trajectory` ([hebiros/Trajectory Action](http://docs.ros.org/en/kinetic/api/hebiros/html/action/Trajectory.html))
    
    Send pose command to reach region when executing the action `goto_<region>`.
    
#### Subscribed Topics
- `next_move_cmd` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Next move from the output word (action sequence) to be carried out by the agent in order to satisfy the plan.
    
- `current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Agent region from the transition system state model `6d_jointspace_region`.

- `current_load_state` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Agent load state from the transition system state model `hebi_load`.
    
- `hebiros/arm_group/feedback/joint_state` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

   Manipulator joint values.

- `pick_inplace_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

   External acknowledgement that everything is in place for pick-up action. The manipulator will wait for this topic to be true before executing pick-up action.

- `drop_inplace_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

   External acknowledgement that everything is in place for drop action. The manipulator will wait for this topic to be true before executing drop action.
   
- `end_effector_state` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

   End-effector state.

#### Published Topics
- `ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic. The agent TS state is composed of a list of states from the different state models composing the action model. The 6-DoF Hebi manipulator node aggretates the `6d_jointspace_region` state from a region_6d_jointspace_monitor with a `hebi_load` state from the hebi_load_monitor.
    
- `end_effector_command` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
    
    Command sent to the end effector.
    
#### Parameters
- `agent_name` (string, default: "agent")

    Agent name.
    
- `transition_system_textfile` (string)

    Action model transition system definition.
    
### hebi_load_monitor.py
Monitor the load state of the manipulator and publishes it to the LTL 6-DoF Hebi manipulator node. The load state is based on the manipulator region and end effector state.

#### Subscribed Topics
- `hebiros/arm_group/feedback/joint_state` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

   Manipulator joint values.
   
- `end_effector_state` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

   End-effector state.

#### Published Topics
- `current_load_state` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Agent load state from the transition system state model `hebi_load`.

- `pick_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

   Send pick-up acknowledgement to be used by external agents.

- `drop_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

   Send drop acknowledgement to be used by external agents.
    
#### Parameters
- `transition_system_textfile` (string)

    Action model transition system definition.

### hebi_pick_drop_inplace_monitor.py
Monitor the state of the external agents to pick up from and deliver to, and send acknowledgment that they are in place and in the proper state for their respective action. For pick-up agents, a parameter defines the expected states. For drop agents, the state should be `unloaded`.

#### Subscribed Topics
- `/<pick_agent>/current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
  
  A topic for every agent in the pick-up agent list. Listen to the current region to know if one agent in the list is on the pick-up region.
  
- `/<drop_agent>/current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
  
  A topic for every agent in the drop agent list. Listen to the current region to know if one agent in the list is on the drop region.

- `/<pick_agent>/ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

  A topic for every agent in the pick-up agent list. Used to check if the agent to pick-up from has the correct state.

- `/<drop_agent>/ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

  A topic for every agent in the drop agent list. Used to check if the agent to deliver to has the correct state.
  
#### Published Topics   
- `pick_inplace_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

   Acknowledgement that everything is in place for pick-up action.

- `drop_inplace_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

   Acknowledgement that everything is in place for drop action.
   
#### Parameters
- `~pick_agent_list` (string[])
   
   List of agents to pick-up from. Used to generate the topic subscriptions.

- `~drop_agent_list` (string[])
  
   List of agents to deliver to. Used to generate the topic subscriptions.

- `~pick_agent_load_state_guard` (string[])

   List of state for the pick agents to be in to be considered ready for pick-up action.

- `~pick_region` (string)

  Region the pick agents should be in to be considered ready for pick-up action.

- `~drop_region` (string)

  Region the drop agents should be in to be considered ready for drop action.
