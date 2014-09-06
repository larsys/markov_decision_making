#!/usr/bin/env python

import roslib; roslib.load_manifest ( 'item_finding' )
import rospy
import actionlib
import os
import sys
import random

from topological_tools.msg import PoseLabel
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from item_finding.msg import HandleObjectAction
from std_srvs.srv import Empty



class Simulator ():
    def __init__ ( self ):
        # ROS actionlib server
        self.handle_object_server = actionlib.SimpleActionServer ( '/agent/100/handle_object', HandleObjectAction, self.execute, False )
        self.handle_object_server.start ()
        
        # ROS Subscribers
        self.sub_robot_location  = rospy.Subscriber ( "/agent/100/pose_label", PoseLabel, self.callback_location )
        
        # ROS Publishers
        #self.pub_person_confidence = rospy.Publisher ( '/agent/100/person_confidence', Int32 )
        self.pub_object_confidence = rospy.Publisher ( '/agent/100/object_confidence', Int32 )
        self.pub_object_possession = rospy.Publisher ( '/agent/100/object_possession', Bool )
        
        # ROS Services
        self.republish_client = rospy.ServiceProxy ( '/agent/100/publish_new_action', Empty )
        #rospy.wait_for_service( 'publish_new_action', 5 )
        
        # Variables
        #self.person_location = 0
        self.object_location = 0
        self.robot_location = 0
        
        #self.person_confidence = 0
        self.object_confidence = 0
        self.object_possession = False
        
        #self.generate_variable ( self.person_location )
        self.generate_variable ()



    def generate_variable ( self ):
        r = random.random ()
        
        if r < 0.6:
            self.object_location = "Bedroom"
        elif r < 0.8:
            self.object_location = "DiningArea"
        else:
            self.object_location = "TVArea"
            
        print '\033[92m Object location is ' + str ( self.object_location )



    def generate_confidence_levels ( self, var, confidence_var, pub ):
        r = random.random ()
        
        if var == self.robot_location:
            if r < 0.75:
                confidence_var = 2
            else:
                confidence_var = 1
        else:
            # Object/Person not found
            confidence_var = 0
    
        print '\033[92m Publishing confidence var = ' + str ( confidence_var ) + '\033[0m'
    
        pub.publish ( Int32 ( confidence_var ) )
    
    

    def execute ( self, goal ):
        print '\033[92m Received grasp command \033[0m'
        
        if goal.grab_or_release == 0:
            # Release
            if self.object_possession == True:
                self.object_possession = False
        else:
            # Grab
            if self.robot_location == self.object_location:
                if self.object_possession == False:
                    self.object_possession = True
            else:
                request = Empty ()
                rospy.wait_for_service( '/agent/100/publish_new_action', 5 )
                self.republish_client ()
        
        print '\033[92m Publishing object possession = ' + str ( self.object_possession ) + '\033[0m'
        
        self.pub_object_possession.publish ( Bool ( self.object_possession ) )
        
        self.handle_object_server.set_succeeded ()
        
        print '\033[92m Execution done. Set succeeded. \033[0m'



    def callback_location ( self, data ):
        location = data.label
        
        if location == 255:
            self.robot_location = "Bedroom"
        elif location == 46335:
            self.robot_location = "Bathroom"
        elif location == 2031360:
            self.robot_location = "InsideHallway"
        elif location == 60159:
            self.robot_location = "OutsideHallway"
        elif location == 16757760:
            self.robot_location = "DiningArea"
        elif location == 13434986:
            self.robot_location = "TVArea"
        elif location == 10289081:
            self.robot_location = "KitchenArea"
            
        self.generate_confidence_levels ( self.object_location, self.object_confidence, self.pub_object_confidence )
        #self.generate_confidence_levels ( self.person_location, self.person_confidence, self.pub_person_confidence )
        
        print '\033[92m Robot location is ' + str ( self.robot_location ) + '\033[0m'





def main ():
    # Create the ROS node
    rospy.init_node ( 'item_finding_simulator', anonymous = True )
    
    Simulator ()
    
    rospy.spin ()
    




if __name__ == "__main__":
    main ()
