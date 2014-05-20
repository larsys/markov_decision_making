#! /usr/bin/env python

import sys
import math
import random

import roslib; roslib.load_manifest('mbot_simulation')
import rospy
import smach
import smach_ros

from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
#from scout_msgs.msg import *
#from scout_msgs.srv import *


# Bedroom Coords
PT1_X = -9.8519
PT1_Y = -7.2850
PT1_T = 0

# Living Room Coords
PT2_X = 4.1143
PT2_Y = -7.5501
PT2_T = 0

# Living Room Coords
PT3_X = 3.8461
PT3_Y = -20.7156
PT3_T = 0

# Living Room Coords
PT4_X = -9.5119
PT4_Y = -20.6804
PT4_T = 0

## Bathroom Coords
#BATHROOM_X = 1
#BATHROOM_Y = 3
#BATHROOM_T = 0

## Inside Hallway Coords
#IH_X = 2
#IH_Y = 1.5
#IH_T = 0



def pose2alib ( x, y, t ):
    print "MOVING TO:" + " X - " + str(x) + " Y - " + str(y) + " T - " + str(t)
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



def task ():
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )
    
    with sm:
      sm.add ( 'MOVE_2_PT1',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( PT1_X, PT1_Y, PT1_T ) ),
               transitions = { 'succeeded': 'MOVE_2_PT2', 'aborted': 'MOVE_2_PT2' } )

      sm.add ( 'MOVE_2_PT2',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( PT2_X, PT2_Y, PT2_T ) ),
               transitions = { 'succeeded': 'MOVE_2_PT3', 'aborted': 'MOVE_2_PT3' } )
      
      sm.add ( 'MOVE_2_PT3',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( PT3_X, PT3_Y, PT3_T ) ),
               transitions = { 'succeeded': 'MOVE_2_PT4', 'aborted': 'MOVE_2_PT4' } )

      sm.add ( 'MOVE_2_PT4',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( PT4_X, PT4_Y, PT4_T ) ) )
        
      #sm.add ( 'MOVE_2_BATHROOM',
               #smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( BATHROOM_X, BATHROOM_Y, BATHROOM_T ) ),
               #transitions = { 'succeeded': 'MOVE_2_INSIDE_HALLWAY', 'aborted': 'FAILURE' } )
      
      #sm.add ( 'MOVE_2_INSIDE_HALLWAY',
               #smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( IH_X, IH_Y, IH_T ) ),
               #transitions = { 'succeeded': 'MOVE_2_LIVING_ROOM_2', 'aborted': 'FAILURE' } )
      
      #sm.add ( 'MOVE_2_LIVING_ROOM_2',
               #smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( LR_X, LR_Y, LR_T ) ) )
      
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
