#!/usr/bin/env python
import rospy
import sys
import yaml
import std_msgs
from copy import deepcopy
#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState
# Import transition system loader
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
# Import modules for commanding the nexus
import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import hebiros.msg


#=================================================================
#  Interfaces between LTL planner node and lower level controls
#                       -----------------
# The node is reponsible for outputting current agent state based
# on TS and agent output.
# The node converts TS action to interpretable commands using
# action attributes defined in the TS config file
#=================================================================
class LTLController(object):
    def __init__(self):

        self.init_params()

        self.create_monitors()

        self.set_pub_sub()

        self.main_loop()

    #---------------------------------------------------
    # Get params from ROS param server and config files
    #---------------------------------------------------
    def init_params(self):
        self.agent_name = rospy.get_param('agent_name', "6dof_hebi")

        # Get TS from param
        self.transition_system = import_ts_from_file(rospy.get_param('transition_system_textfile'))

        # Init state message with TS
        self.ltl_state_msg = TransitionSystemStateStamped()
        self.ltl_state_msg.ts_state.state_dimension_names = self.transition_system["state_dim"]

        # Initialize running time and index of command received and executed
        self.t0 = rospy.Time.now()
        self.t = self.t0
        self.plan_index = 0

        self.next_action = None
        # Initialize the end_effector state and command
        self.end_effector_state = None
        self.end_effector_on = Bool(False)

        # Initialize inplace feedback
        self.pick_inplace_feedback = False
        self.drop_inplace_feedback = False

        # Initialize pick-ready, drop-ready positions
        self.pick_ready_position = []
        self.drop_ready_position = []

        self.currentJointState = JointState()
        # self.Goal= hebiros.msg.TrajectoryActionGoal()
        self.GoalCmd= hebiros.msg.TrajectoryGoal()
        self.currentGoalCmd= hebiros.msg.TrajectoryGoal()
        self.goalActionActive= False
        self.hebiActionState= None

    #-------------------------------------------------------------------------
    # Create monitoring object for every state dimension in transition system
    #-------------------------------------------------------------------------
    def create_monitors(self):
        number_of_dimensions = len(self.transition_system["state_dim"])

        # Init LTL state variables
        self.curr_ltl_state = [None for element in range(number_of_dimensions)]
        self.prev_ltl_state = deepcopy(self.curr_ltl_state)

        # Setup subscribers
        for i in range(number_of_dimensions):
            print "checking dimension states"
            dimension = self.transition_system["state_dim"][i]
            print dimension
            if (dimension == "6d_jointspace_region"):
                # Setup subscriber to 6D joint space region monitor
                # may need to retreive only the joint angle part info
                self.hebi_region_sub = rospy.Subscriber("current_region", String, self.region_state_callback, i, queue_size=100)
            elif (dimension == "hebi_load"):
                # Setup subscriber to hebi load state
                self.hebi_load_sub = rospy.Subscriber("current_load_state", String, self.load_state_callback, i, queue_size=100)
            else:
                raise ValueError("state type [%s] is not supported by LTL 6dof Hebi Arm" % (dimension))

    #----------------------------------
    # Setup subscribers and publishers
    #----------------------------------
    def set_pub_sub(self):

        self.trajActionClient = actionlib.SimpleActionClient('hebiros/arm_group/trajectory', hebiros.msg.TrajectoryAction)
        rospy.loginfo("LTL automaton Hebi Arm node: waiting for HEBI trajectory action server...")
        self.trajActionClient.wait_for_server() # wait for action server to start

        # Setup LTL state publisher
        self.ltl_state_pub = rospy.Publisher("ts_state", TransitionSystemStateStamped, latch=True, queue_size=10)

        # Setup subscriber to ltl_automaton_core next_move_cmd
        self.next_move_sub = rospy.Subscriber("next_move_cmd", std_msgs.msg.String, self.next_move_callback, queue_size=1)

        # Subscribe to arm joint_state topic 
        # topic path for experiment 
        self.pose_sub = rospy.Subscriber("hebiros/arm_group/feedback/joint_state", JointState, self.joint_state_callback)

        # Setup cargo-for-pickup in-place acknowkedge topic subscriber
        self.pick_inplace_sub = rospy.Subscriber("pick_inplace_ack", std_msgs.msg.Bool, self.pick_inplace_ack_callback)
        # Setup cargo-for-drop in-place acknowkedge topic subscriber
        self.drop_inplace_sub = rospy.Subscriber("drop_inplace_ack", std_msgs.msg.Bool, self.drop_inplace_ack_callback)

        # Setup end_effector state subscriber and command publisher
        self.end_effector_sub= rospy.Subscriber("end_effector_state", std_msgs.msg.String, self.end_effector_state_callback)        
        self.end_effector_pub= rospy.Publisher("end_effector_command", std_msgs.msg.Bool, latch= True,queue_size=10)        

        rospy.loginfo("LTL automaton 6dof Hebi Arm node: initialized!")

    # ------------------------------------
    #     Process current joint states
    # ------------------------------------
    def joint_state_callback(self,msg):

        self.currentJointState= msg

    #----------------------------------------
    # Handle message from load state monitor
    #----------------------------------------
    def load_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data

    #------------------------------------------
    # Handle message from region state monitor
    #------------------------------------------
    def region_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data

    #------------------------------------------
    # Handle end-effector state from kth_hebi_arm package
    #------------------------------------------
    def end_effector_state_callback(self, msg):
        self.end_effector_state = msg.data

    #---------------------------------------
    # Handle next move command from planner
    #---------------------------------------
    def next_move_callback(self, msg):
        '''Recieve next_move_cmd from ltl_automaton_core planner and convert into robot action to implement'''

        # Update running time and augment plan index
        self.t = rospy.Time.now()-self.t0
        self.plan_index += 1

        # Extract command message string
        cmd_str =  msg.data

        # Check if next_move_cmd is 'None', which is output by ltl_automaton_core if the current state is not in the TS
        if cmd_str == "None":

            # To do: Handle when ltl_automaton_core encounteres state outside of TS (i.e. next_move_cmd = 'None')
            rospy.logwarn('None next_move_cmd sent to LTL 6dof Hebi Arm')
        else:

            # Check if next_move_cmd is in list of actions from transition_system
            for act in self.transition_system['actions']:
                if str(act) == cmd_str:

                    # Extract action types, attributes, etc. in dictionary
                    action_dict = self.transition_system['actions'][str(act)]

                    break

            # Raise error if next_move_cmd does not exist in transition system
            if not(action_dict):
                raise ValueError("next_move_cmd not found in LTL 6dof Hebi transition system")

        self.next_action = action_dict


    def hebi_action(self, act_dict):
        '''Read components of act_dict associated with current command and output control to 6dof Hebi'''  
        # 0 for action implemented successfully
        # 1 for the cargo/robot not inplace for pick/drop
        # 2 for the end-effector not working properly
        # 3 for indefinite results yet
        # 4 for other scenarios

        if act_dict['type'] == 'move':
            # Extract pose to move to:
            jointposition = act_dict['attr']['jointposition']

            # compose initial and target waypoints
            print("self.currentJointState.position" + str(self.currentJointState.position))
            WayPoint1 = self.compose_waypoint(self.currentJointState.position)
            WayPoint2 = self.compose_waypoint(jointposition)

            self.GoalCmd.times = [0.0,3.0]
            self.GoalCmd.waypoints = [WayPoint1,WayPoint2]
            self.currentGoalCmd = deepcopy(self.GoalCmd)

            # Sends the goal to the action server. 
            self.goalActionActive = True
            self.trajActionClient.send_goal(self.GoalCmd,done_cb = self.goalActionActive_done_cb)
            rospy.loginfo("Goal" + str(jointposition) + " is send to Trajectory Server!")
            self.hebiActionState = 0
            return self.hebiActionState 

        if act_dict['type'] == 'hebi_pick':
            # obtain the pick-ready and pick positions
            self.pick_ready_position = act_dict['attr']['jointpositionlist'][0]
            pick_position = act_dict['attr']['jointpositionlist'][1]

            # wait for the to-be-picked-from robot to be in place
            if not self.pick_inplace_feedback:
                if not self.goalActionActive:
                    print("testing waiting location 0")
                    self.hebi_wait(self.pick_ready_position)
                    rospy.loginfo("The cargo is not in place for picking-up. Hebi arm is waiting...")
                self.hebiActionState = 1
                return self.hebiActionState
            else:
                # start from the current position
                WayPoint1 = self.compose_waypoint(self.currentJointState.position)
                # go to the target position 
                WayPoint2 = self.compose_waypoint(pick_position)

                self.GoalCmd.times = [0.0,3.0]
                self.GoalCmd.waypoints = [WayPoint1,WayPoint2]

                # Sends the goal to the action server. 
                # In pickCargo callback, both the end-effector command and back to pick-ready command are implemented

                # this is a loop in main_loop() and can only exit when 0 is returned
                # if trajectoryActionClient is inactive, check
                    # if the joint position is in the pick-ready region and the end-effector_functioning is not True
                    # then go to pick position with pickCargo as done_cb. 
                    # And if end-effector is False, return 2
                # else: waiting at pick-ready position
                # if the trajectoryActionClient is active
                    # check if the go-back to pick-ready command sent and the end-effector functioning, return 0
                    # else: trajectoryAction still implementing, return 3
                if not self.goalActionActive:
                    if ((self.dist_6d_err(self.currentJointState.position, self.pick_ready_position ) < 0.1) and (self.end_effector_state != "activated")):  
                        self.currentGoalCmd = deepcopy(self.GoalCmd)
                        self.goalActionActive = True
                        self.trajActionClient.send_goal(self.GoalCmd,done_cb = self.pickCargo)
                        rospy.loginfo("Goal" + str(pick_position) + " is send to Trajectory Server!")

                        if self.end_effector_state == "deactivated":
                            rospy.loginfo(" The end-effector did not activate yet. Trying. ")
                            self.hebiActionState = 2
                            return self.hebiActionState
                        self.hebiActionState = 3
                        return self.hebiActionState
                    else:
                        # something unexpected happens, go back to pick ready
                        self.hebi_wait(self.pick_ready_position)
                        self.hebiActionState = 4
                        return self.hebiActionState
                else:
                    if (self.currentGoalCmd.waypoints[1].positions == self.pick_ready_position and (self.end_effector_state == "activated")):
                        rospy.loginfo("Pick action succeeds. ")
                        self.hebiActionState = 0
                        return self.hebiActionState
                    self.hebiActionState = 3
                    return self.hebiActionState 

            

        if act_dict['type'] == 'hebi_drop':
            # obtain the drop-ready and drop positions
            self.drop_ready_position = act_dict['attr']['jointpositionlist'][0]
            drop_position = act_dict['attr']['jointpositionlist'][1]

            # wait for the to-be-loaded robot to be in place
            if not self.drop_inplace_feedback:
                # print("trajActionClient.get_state() is " + str(self.trajActionClient.get_state() ) )
                if not self.goalActionActive:
                    print('testing waiting location 0')
                    self.hebi_wait(self.drop_ready_position)
                    rospy.loginfo("Hebi is waiting at drop-ready position " + str(self.currentJointState.position))
                self.hebiActionState = 1
                return self.hebiActionState
            else:
                # start from the current position
                WayPoint1 = self.compose_waypoint(self.currentJointState.position)
                # go to the target position 
                WayPoint2 = self.compose_waypoint(drop_position)

                self.GoalCmd.times = [0.0,3.0]
                self.GoalCmd.waypoints = [WayPoint1,WayPoint2]

                # Sends the goal to the action server. 
                # In dropCargo callback, both the end-effector command and back to drop-ready command are implemented

                # this is a loop in main_loop() and can only exit when 0 is returned
                # if trajectoryActionClient is inactive, check
                    # if the joint position is in the go-to-drop-ready region and the end-effector_functioning is not True
                    # then go to drop position with dropCargo as done_cb. 
                    # And if end-effector is False, return 2
                # else: waiting at drop-ready position
                # if the trajectoryActionClient is active
                    # check if the go-back to drop-ready command sent and the end-effector functioning, return 0
                    # else: trajectoryAction still implementing, return 3 
                if not self.goalActionActive:
                    if ((self.dist_6d_err(self.currentJointState.position, self.drop_ready_position ) < 0.05) and (self.end_effector_state !="deactivated")): 
                        print("\n Hebi arm is approaching drop position now.")
                        self.currentGoalCmd = deepcopy(self.GoalCmd)
                        self.goalActionActive = True
                        self.trajActionClient.send_goal(self.GoalCmd,done_cb = self.dropCargo)
                        rospy.loginfo("Goal" + str(drop_position) + " is send to Trajectory Server!")

                        if self.end_effector_state == "activated":
                            rospy.loginfo("The end-effector is not de-activated. Trying. ")
                            self.hebiActionState = 2
                            return self.hebiActionState
                        self.hebiActionState = 3
                        return self.hebiActionState
                    else:
                        # something unexpected happens, go back to drop ready
                        self.hebi_wait(self.drop_ready_position)
                        self.hebiActionState = 4
                        return self.hebiActionState
                else:
                    # print("self.currentGoalCmd.waypoints[1]: " + str(self.currentGoalCmd.waypoints[1].positions))
                    # print("self.currentJointState.position: " + str(self.currentJointState.position))
                    if ((self.currentGoalCmd.waypoints[1].positions == self.drop_ready_position) and (self.end_effector_state == "deactivated")):
                        rospy.loginfo("Drop action succeeds. ")
                        self.hebiActionState = 0
                        return self.hebiActionState
                    self.hebiActionState = 3
                    return self.hebiActionState 

    #-------------------------------
    # Compute distance
    # between two jointPositions
    #-------------------------------
    # Position format [e1,e2,e3,...,e6]
    # Pose msg is ROS sensor_msgs/JointState/position
    # TODO need to adjust the distance metric based on experiment
    def dist_6d_err(self, position, center_position):
        # return math.sqrt((position[0] - center_position[0])**2 + (position[1] - center_position[1])**2  + (position[2] - center_position[2])**2 
        #                +   (position[3] - center_position[3])**2  + (position[4] - center_position[4])**2   + (position[5] - center_position[5])**2  )
        position_err = [a_i - b_i for a_i, b_i in zip(position, center_position)] 
        position_err_abs = [abs(err) for err in position_err]
        return max(position_err_abs[0:4]) + max(position_err_abs[4:]) 

    #-----------------------------------------
    # Handle pick-up cargo in place acknowledgement
    #------------------------------------------
    def pick_inplace_ack_callback(self, msg):
        self.pick_inplace_feedback = msg.data

    #-----------------------------------------------------------------
    # Handle pick-up cargo command and back to pick-ready position
    #------------------------------------------------------------------
    def pickCargo(self,GoalStatus,result):
        if GoalStatus == 3: # The goal was achieved successfully by the action server
            self.end_effector_on = Bool(True)

            # Publish electromagenet command
            self.end_effector_pub.publish(self.end_effector_on)

            # a few iterations in case the end-effector is not active
            for i in range(10):
                print("self.end_effector_state is " + str(self.end_effector_state))
                if self.end_effector_state != "activated":
                    self.end_effector_pub.publish(self.end_effector_on)
                else:
                    break
                


            # start from the current position
            WayPoint1 = self.compose_waypoint(self.currentJointState.position)
            # go to the target position 
            WayPoint2 = self.compose_waypoint(self.pick_ready_position)

            self.GoalCmd.times = [0.0,2.0]
            self.GoalCmd.waypoints = [WayPoint1,WayPoint2]
            self.currentGoalCmd = deepcopy(self.GoalCmd)

            # Sends the goal to the action server.
            self.goalActionActive =True
            self.trajActionClient.send_goal(self.GoalCmd,done_cb = self.goalActionActive_done_cb)
            rospy.loginfo("Now go back to the pickup-ready position.")
        else:
            rospy.loginfo("The goto-pick-position action is not completed. GoalStatus code: " + str(GoalStatus))


    #-----------------------------------------
    # Handle drop cargo in place acknowledgement
    #------------------------------------------
    def drop_inplace_ack_callback(self, msg):
        self.drop_inplace_feedback = msg.data
        # rospy.loginfo("The drop_inplace_feedback is  " + str(msg.data))

    #-----------------------------------------------------------------
    # Handle drop cargo command and back to drop-ready position
    #------------------------------------------------------------------
    def dropCargo(self,GoalStatus,result):
        if GoalStatus == 3: # The goal was achieved successfully by the action server
            self.end_effector_on = Bool(False)
            # Publish electromagenet command
            self.end_effector_pub.publish(self.end_effector_on)

            # a few iterations in case the end-effector is not de-active
            # todo there are some problems with state checking. Due to sampling maybe?
            for i in range(10):
                print("self.end_effector_state is " + str(self.end_effector_state))
                if self.end_effector_state != "deactivated":
                    self.end_effector_pub.publish(self.end_effector_on)
                else:
                    break

                
            # start from the current position
            WayPoint1 = self.compose_waypoint(self.currentJointState.position)
            # go to the target position 
            WayPoint2 = self.compose_waypoint(self.drop_ready_position)

            self.GoalCmd.times = [0.0,2.0]
            self.GoalCmd.waypoints = [WayPoint1,WayPoint2]
            self.currentGoalCmd = deepcopy(self.GoalCmd)
            
            # Sends the goal to the action server.
            self.goalActionActive = True
            self.trajActionClient.send_goal(self.GoalCmd,done_cb = self.goalActionActive_done_cb)
            rospy.loginfo("The goto-drop-ready action is send to Trajectory Server!")
        else:
            rospy.loginfo("The goto-drop-position action is not completed. GoalStatus code: " + str(GoalStatus))



    # ------------------------------------
    #     Process action done goalActionActive_done_cb
    # ------------------------------------
    def goalActionActive_done_cb(self,GoalStatus,result):

        self.goalActionActive= False

    #-----------------------------------------
    # compose hebi WayPoint
    #------------------------------------------
    def compose_waypoint(self,position):
        # Set new position goal and send
        WayPoint = hebiros.msg.WaypointMsg()
        
        # for simulation
        # WayPoint.names = ['Arm/base','Arm/shoulder','Arm/elbow','Arm/wrist1','Arm/wrist2','Arm/wrist3']
        # for experiment
        WayPoint.names = ['Arm/base/X8_9','Arm/shoulder/X8_16','Arm/elbow/X5_9','Arm/wrist1/X5_9','Arm/wrist2/X5_1','Arm/wrist3/X5_1']
        WayPoint.positions = position
        WayPoint.velocities = [0.0,0.0,0.0, 0.0,0.0,0.0]
        WayPoint.accelerations =  [0.0,0.0,0.0, 0.0,0.0,0.0]

        return WayPoint


    #-----------------------------------------
    # wait at the same position using Hebi TrajectoryAction until being Preempted
    #------------------------------------------
    def hebi_wait(self,position):
        # start from the current position
        WayPoint1 = self.compose_waypoint(self.currentJointState.position)
        # go to the target position 
        WayPoint2 = self.compose_waypoint(position)

        self.GoalCmd.times = [0.0,3.0]
        self.GoalCmd.waypoints = [WayPoint1,WayPoint2]

        # Send to the action server. 
        self.currentGoalCmd = deepcopy(self.GoalCmd)
        self.goalActionActive = True
        self.trajActionClient.send_goal(self.GoalCmd, done_cb = self.goalActionActive_done_cb)
        

    def main_loop(self):
        rate = rospy.Rate(20)
        i = None
        while not rospy.is_shutdown():
            # If current state is different from previous state
            # update message and publish it
            if not (self.curr_ltl_state == self.prev_ltl_state):
                # Update previous state
                print '\n update previous state'
                print '\n --------self.prev_ltl_state---------------'
                print self.prev_ltl_state
                print '\n --------self.curr_ltl_state---------------'
                print self.curr_ltl_state
                self.prev_ltl_state = deepcopy(self.curr_ltl_state)
                # If all states are initialized (not None), publish message
                if all([False for element in self.curr_ltl_state if element == None]):
                    # # Publish msg
                    print '\n -----------------------'
                    print self.curr_ltl_state
                    self.ltl_state_msg.header.stamp = rospy.Time.now()
                    self.ltl_state_msg.ts_state.states = self.curr_ltl_state
                    self.ltl_state_pub.publish(self.ltl_state_msg)

            # If waiting for acknowledgement to implement actions, check again
            if self.next_action:
                # If action returns true, action was carried out and is reset
                prev_i = i
                i = self.hebi_action(self.next_action) 
                # print("The state of hebi_action now is " + str(i))
                if i == 0:
                    self.next_action = {}
                if i != prev_i:
                    print("---------------------------------\n")
                    print("The state of hebi_action now changes to " + str(i))
            # print(self.trajActionClient.get_result())
            # print("self.trajActionClient.get_state" + str(self.trajActionClient.get_state()))

            # if the hebi action is sent and executed already while still no next_move_command received
            if self.hebiActionState == 0 and not self.goalActionActive:
                if self.currentGoalCmd.waypoints[1].positions:
                    self.hebi_wait(self.currentGoalCmd.waypoints[1].positions)
                else:
                    self.hebi_wait(self.currentJointState)
                rospy.loginfo("hebi arm not receiving next move command. May due to state/joint monitor")



            # rospy.loginfo("State is %s and prev state is %s" %(self.curr_ltl_state, self.prev_ltl_state))
            rate.sleep()    

#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_hebi',anonymous=False)
    try:
        ltl_hebi = LTLController()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Hebi node: %s" %(e))
        sys.exit(0)
    