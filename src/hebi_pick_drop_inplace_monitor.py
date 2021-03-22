#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
# from sensor_msgs.msg import JointState
from copy import deepcopy
from ltl_automaton_msgs.msg import TransitionSystemStateStamped
# from ltl_automaton_msgs.srv import ClosestState
import sys

#=====================================
#      Monitor pick and drop agents
#         return pick/drop inplace state
#=====================================
class HebiPickDropInPLaceMonitor(object):
    def __init__(self):

        # to obtain from launch file
        self.pick_agent_list = rospy.get_param('~pick_agent_list')
        self.drop_agent_list = rospy.get_param('~drop_agent_list')
        self.pick_agent_load_state_guard_list = rospy.get_param('~pick_agent_load_state_guard')

        self.pick_region = rospy.get_param('~pick_region')
        self.drop_region = rospy.get_param('~drop_region')

        self.pick_agent_region_dic = {}
        self.drop_agent_region_dic = {}
        self.pick_agent_load_dic = {}
        self.drop_agent_load_dic = {}

        for agent in self.pick_agent_list:
            self.pick_agent_region_dic[agent] = None
            self.pick_agent_load_dic[agent] = None

        for agent in self.drop_agent_list:
            self.drop_agent_region_dic[agent] = None
            self.drop_agent_load_dic[agent] = None



        self.pick_agent_inplace = False
        self.drop_agent_inplace = False

        # print(self.pick_agent_list[0])

        # Setup callback
        self.setup_pub_sub()
        self.main_loop()

    #----------------------------------
    # Setup subscribers and publishers
    #----------------------------------
    def setup_pub_sub(self):

        # for experiment
        for agent in self.pick_agent_list:
            rospy.Subscriber("/" + agent+ "/current_region", String, self.pick_agent_region_callback,agent, queue_size=10)
            rospy.Subscriber("/" + agent+ "/ts_state", TransitionSystemStateStamped, self.pick_agent_loaded_callback,agent,queue_size=10)

        for agent in self.drop_agent_list:
            rospy.Subscriber("/" + agent+ "/current_region", String, self.drop_agent_region_callback,agent, queue_size=10)
            rospy.Subscriber("/" + agent+ "/ts_state", TransitionSystemStateStamped, self.drop_agent_loaded_callback,agent,queue_size=10)

        # Publisher of current pick/drop inplace info
        self.pick_inplece_pub = rospy.Publisher("pick_inplace_ack", Bool, latch=True, queue_size=10)
        self.drop_inplece_pub = rospy.Publisher("drop_inplace_ack", Bool, latch=True, queue_size=10)

    #---------------------------------------
    # handle pick_agent_region_callback callback
    #---------------------------------------
    def pick_agent_region_callback(self, msg,agent):
        self.pick_agent_region_dic[agent] = msg.data

    #---------------------------------------
    # handle pick_agent_loaded_callback callback
    #---------------------------------------
    def pick_agent_loaded_callback(self, msg,agent):
        self.pick_agent_load_dic[agent] = False
        for load_state in self.pick_agent_load_state_guard_list:
            if load_state in msg.ts_state.states:
                self.pick_agent_load_dic[agent] = True

    #---------------------------------------
    # handle pick_agent_region_callback callback
    #---------------------------------------
    def drop_agent_region_callback(self, msg,agent):
        self.drop_agent_region_dic[agent] = msg.data

    #---------------------------------------
    # handle pick_agent_loaded_callback callback
    #---------------------------------------
    def drop_agent_loaded_callback(self, msg,agent):
        self.drop_agent_load_dic[agent] = True
        if "unloaded" in msg.ts_state.states:
            self.drop_agent_load_dic[agent] = False


    def main_loop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # Check for the pick region is occupied or not and the load state
            # update message and publish it
            self.pick_agent_inplace = False
            self.drop_agent_inplace = False

            for pick_agent in self.pick_agent_list:
                if self.pick_agent_region_dic[pick_agent] == self.pick_region and self.pick_agent_load_dic[pick_agent] == True:
                    self.pick_agent_inplace = True


            for drop_agent in self.drop_agent_list:
                if self.drop_agent_region_dic[drop_agent] == self.drop_region and self.drop_agent_load_dic[drop_agent] == False:
                    self.drop_agent_inplace = True

            self.pick_inplece_pub.publish(self.pick_agent_inplace)
            self.drop_inplece_pub.publish(self.drop_agent_inplace)

            # rospy.loginfo("State is %s and prev state is %s" %(self.curr_ltl_state, self.prev_ltl_state))
            rate.sleep()    



#============================
#            Main            
#============================
if __name__ == '__main__':
    rospy.init_node('hebi_pick_drop_inplace_monitor',anonymous=False)
    try:
        hebi_load_monitor = HebiPickDropInPLaceMonitor()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Hebi load Monitor: %s" %(e))
        sys.exit(0)



    