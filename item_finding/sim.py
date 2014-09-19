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
from geometry_msgs.msg import PoseWithCovarianceStamped
from item_finding.msg import HandleObjectAction
from std_srvs.srv import Empty



class Simulator ():
    def __init__ ( self ):
        # ROS actionlib servers
        self.grasp_object_server = actionlib.SimpleActionServer ( '/agent/100/handle_object_grasp', HandleObjectAction, self.grasp, False )
        self.grasp_object_server.start ()
        
        self.release_object_server = actionlib.SimpleActionServer ( '/agent/100/handle_object_release', HandleObjectAction, self.release, False )
        self.release_object_server.start ()
        
        # ROS Subscribers
        self.sub_robot_location  = rospy.Subscriber ( "/agent/100/pose_label", PoseLabel, self.callback_location )
        self.sub_reset  = rospy.Subscriber ( "/agent/100/reset_all", Int32, self.callback_reset )
        
        # ROS Publishers
        self.pub_person_found = rospy.Publisher ( '/agent/100/person_found', Bool )
        self.pub_object_confidence = rospy.Publisher ( '/agent/100/object_confidence', Int32 )
        self.pub_object_possession = rospy.Publisher ( '/agent/100/object_possession', Bool )
        
        # ROS Services
        self.republish_client = rospy.ServiceProxy ( '/agent/100/publish_new_action', Empty )
        
        # Variables
        self.person_location = "DiningArea"
        #self.person_location = "Bedroom"
        self.object_location = 0
        self.robot_location = "TVArea"
        
        self.person_found = False
        self.object_confidence = 0
        self.object_possession = False
        self.object_previously_possessed = False
        
        self.pub_object_possession.publish ( Bool ( False ) )
        self.pub_person_found.publish ( Bool ( False ) )
        
        self.generate_variable ()
        self.generate_confidence_levels ()
        
        


    def callback_reset ( self, data ):
        
        self.person_location = "DiningArea"
        #self.person_location = "Bedroom"
        self.object_location = 0
        self.robot_location = "TVArea"
        
        self.person_found = False
        self.object_confidence = 0
        self.object_possession = False
        self.object_previously_possessed = False
        
        self.pub_object_possession.publish ( Bool ( False ) )
        self.pub_person_found.publish ( Bool ( False ) )
        
        self.generate_variable ()
        self.generate_confidence_levels ()



    def generate_variable ( self ):
        r = random.random ()
        
        if r < 0.6:
            self.object_location = "Bedroom"
        else:
            self.object_location = "TVArea"
        
        #self.object_location = "TVArea"
        #self.object_location = "Bedroom"
        
        print '\033[92m Object location is ' + str ( self.object_location )



    def generate_confidence_levels ( self ):
        r = random.random ()
        
        if self.object_location == self.robot_location:
            if r < 0.5:
                self.object_confidence = 2
            else:
                self.object_confidence = 1
                
            #self.object_confidence = 1
            #self.object_confidence = 2
        else:
            # Object/Person not found
            self.object_confidence = 0
    
        print '\033[92m Publishing confidence var = ' + str ( self.object_confidence ) + '\033[0m'
    
        self.pub_object_confidence.publish ( Int32 ( self.object_confidence ) )
    
    
    
    def is_person_found_checker ( self ):
        if self.robot_location == self.person_location:
            self.is_person_found = True
        else:
            self.is_person_found = False
        
        self.pub_person_found.publish ( Bool ( self.is_person_found ) )
    
    

    def grasp ( self, goal ):
        print '\033[92m Received grasp command \033[0m'
        
        # Grab
        if self.robot_location == self.object_location:
            print '\033[92m Robot location is equal to object location \033[0m'
            
            if self.object_possession == False:
                print '\033[92m Object possession is false \033[0m'
                self.object_possession = True
                self.object_previously_possessed = True
                
                print '\033[92m Publishing object possession = ' + str ( self.object_possession ) + '\033[0m'
    
                self.pub_object_possession.publish ( Bool ( self.object_possession ) )
            else:
                print '\033[92m Republishing from sim -> Grasp \033[0m'
                request = Empty ()
                rospy.wait_for_service( '/agent/100/publish_new_action', 5 )
                self.republish_client ()
                
            print '\033[92m Object possession is ' + str ( self.object_possession ) + '\033[0m'
        else:
            print '\033[92m Republishing from sim -> Grasp \033[0m'
            request = Empty ()
            rospy.wait_for_service( '/agent/100/publish_new_action', 5 )
            self.republish_client ()
            
        self.grasp_object_server.set_succeeded ()
        
        print '\033[92m Finished grasping \033[0m'



    def release ( self, goal ):
        print '\033[92m Received release command \033[0m'
        
        # Release
        if self.robot_location == self.person_location:
            print '\033[92m Robot location is equal to person location \033[0m'
            
            if self.object_possession == True:
                print '\033[92m Object possession is true \033[0m'
                self.object_possession = False
                
                print '\033[92m Publishing object possession = ' + str ( self.object_possession ) + '\033[0m'
    
                self.pub_object_possession.publish ( Bool ( self.object_possession ) )
            else:
                print '\033[92m Republishing from sim -> Grasp \033[0m'
                request = Empty ()
                rospy.wait_for_service( '/agent/100/publish_new_action', 5 )
                self.republish_client ()
                
            print '\033[92m Object possession is ' + str ( self.object_possession ) + '\033[0m'
        else:
            print '\033[92m Republishing from sim -> Release \033[0m'
            request = Empty ()
            rospy.wait_for_service( '/agent/100/publish_new_action', 5 )
            self.republish_client ()
        
        self.release_object_server.set_succeeded ()
            
        print '\033[92m Finished releasing \033[0m'



    def callback_location ( self, data ):
        location = data.label
        
        if location == 255:
            self.robot_location = "Bedroom"
        elif location == 46335:
            self.robot_location = "Bathroom"
        elif location == 2031360:
            self.robot_location = "InsideHallway"
        elif location == 60159:
            self.robot_location = "KitchenArea"
        elif location == 16757760:
            self.robot_location = "DiningArea"
        elif location == 13434986:
            self.robot_location = "TVArea"
            
        self.generate_confidence_levels ()
        self.is_person_found_checker ()
        
        print '\033[92m Robot location is ' + str ( self.robot_location ) + '\033[0m'





def main ():
    # Create the ROS node
    rospy.init_node ( 'item_finding_simulator', anonymous = True )
    
    Simulator ()
    
    rospy.spin ()
    




if __name__ == "__main__":
    main ()
