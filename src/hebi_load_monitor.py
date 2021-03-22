#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
from sensor_msgs.msg import JointState
from copy import deepcopy

# from ltl_automaton_msgs.srv import ClosestState
import sys

#=====================================
#      Monitor 6dof HEBI load and
#         return load state
#=====================================
class HebiLoadStateMonitor(object):
    def __init__(self):
        self.currentJointState = JointState()
        self.currentEndEffectorState = None
        self.prev_load_state = None
        self.curr_load_state = "unloaded"

        self.transition_system = import_ts_from_file(rospy.get_param('transition_system_textfile'))
        self.pick_ready_position = self.transition_system['actions']['goto_pick_ready']['attr']['jointposition']
        self.drop_ready_position = self.transition_system['actions']['goto_drop_ready']['attr']['jointposition']

        print("pick_ready_position " + str(self.pick_ready_position))

        # Setup pose callback
        self.setup_pub_sub()
        self.main_loop()

    #----------------------------------
    # Setup subscribers and publishers
    #----------------------------------
    def setup_pub_sub(self):

        # for experiment
        self.pose_sub = rospy.Subscriber("hebiros/arm_group/feedback/joint_state", JointState, self.joint_state_callback)
        # Setup end_effector state subscriber and command publisher
        self.end_effector_sub= rospy.Subscriber("end_effector_state", String, self.end_effector_state_callback)   

        # Publisher of current load state
        self.current_load_state_pub = rospy.Publisher("current_load_state", String, latch=True, queue_size=10)

        # Publisher of delivered_assembly_ack
        self.delivered_assembly_ack_pub = rospy.Publisher("drop_ack", Bool, latch=True, queue_size=10)

        # Publisher of picked_up_assembly_ack
        self.picked_up_assembly_ack_pub = rospy.Publisher("pick_ack", Bool, latch=True, queue_size=10)

    #---------------------------------------
    # handle joint_state_callback
    #---------------------------------------
    def joint_state_callback(self, msg):
        self.currentJointState = msg

    #---------------------------------------
    # handle end_effector_state_callback
    #---------------------------------------
    def end_effector_state_callback(self, msg):
        self.currentEndEffectorState = msg.data


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

    def main_loop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # If current state is different from previous state
            # update message and publish it
            if self.currentJointState.position:
                if ((self.dist_6d_err(self.pick_ready_position, self.currentJointState.position) < 0.10) and self.currentEndEffectorState == "activated"):
                    self.curr_load_state = "loaded"
                elif ((self.dist_6d_err(self.drop_ready_position, self.currentJointState.position) < 0.10) and self.currentEndEffectorState == "deactivated"):
                    self.curr_load_state = "unloaded"

            if not (self.curr_load_state == self.prev_load_state):
                # Update previous state
                self.prev_load_state = deepcopy(self.curr_load_state)
                # # Publish msg
                print '\n -----------------------'
                print self.curr_load_state
                self.current_load_state_pub.publish(self.curr_load_state)
                if self.curr_load_state == "loaded":
                    delivered_assembly_ack = True
                    self.delivered_assembly_ack_pub.publish(delivered_assembly_ack)
                else:
                    picked_up_assembly_ack = True
                    self.picked_up_assembly_ack_pub.publish(picked_up_assembly_ack)


            # rospy.loginfo("State is %s and prev state is %s" %(self.curr_ltl_state, self.prev_ltl_state))
            rate.sleep()    



#============================
#            Main            
#============================
if __name__ == '__main__':
    rospy.init_node('hebi_load_monitor',anonymous=False)
    try:
        hebi_load_monitor = HebiLoadStateMonitor()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Hebi load Monitor: %s" %(e))
        sys.exit(0)



    
