#! /usr/bin/env python

import sys
import math
import random

import roslib; roslib.load_manifest('demos')
import rospy
import smach
import smach_ros
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from scout_msgs.msg import *
from scout_msgs.srv import *
from cobot_grasp.msg import *


# Bedroom Coords
PT1_X = -9.8519
PT1_Y = -7.2850
PT1_T = 0



def pose2alib ( x, y, t ):
    print "MOVING TO:" + " X - " + str(x) + " Y - " + str(y) + " T - " + str(t)
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



class GraspObject ( smach.State ):
    def __init__ ( self ):
        smach.State.__init__ ( self, outcomes = ['succeeded', 'aborted'] )
        self.client = actionlib.SimpleActionClient('grasp_marker', GraspMarkerAction)
        print "Created state"

    def execute ( self, data ):
        self.client.wait_for_server ()

        goal = GraspMarkerGoal ( x_offset = 0, z_offset = -0.05 )

        self.client.send_goal ( goal )

        self.client.wait_for_result ()

        result = self.client.get_result ().success

        if result == True:
            return 'succeeded'
        else:
            return 'aborted'



class React ( smach.State ):
    def __init__ ( self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )

    def execute ( self, data ):
        print "--------- DONE!--------"

        return 'succeeded'



def task ():
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )
    
    with sm:
      sm.add ( 'GRASP_OBJECT',
               GraspObject (),
               transitions = { 'succeeded': 'REACT', 'aborted': 'GRASP_OBJECT' } )

      sm.add ( 'REACT',
               React (),
               transitions = { 'succeeded': 'succeeded' } )
      
      sm.add ( 'FAILURE',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( PT1_X, PT1_Y, PT1_T ) ) )
    
    return sm



def main ( argv ):
    # Init ROS
    rospy.init_node( "behaviors_tests" )
    
    # Define the task
    sm = task()
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer ( 'behaviors_tests', sm, '/SM_ROOT' )
    sis.start()

    # Execute state machine
    rospy.loginfo ( "Executing behavior state machine" )
    sm.execute()

    sis.stop()



if __name__ == '__main__':
    main ( sys.argv )
