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



def speeding_callback ( userdata, msg ):
    speeding = 'nothing'
    speeding = msg.data

    if speeding == 'SPEEDING_DETECTED':
        return False



def termination_callback ( outcome_map ):
    print "TERM"
    return True



def outcome_callback ( outcome_map ):
    print "OUTCOME"
    if outcome_map['ESCORT'] == 'succeeded':
        return 'succeeded'

    if outcome_map['SPEEDING_DETECTOR'] == 'invalid':
        return 'speeding'



def pose2alib ( x, y, t ):
    print "MOVING TO:" + " X - " + str(x) + " Y - " + str(y) + " T - " + str(t)
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



class Escort ( smach.State ):
    def __init__ ( self ):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )
        self.client = actionlib.SimpleActionClient('heading_control', HeadingControlAction)
        print "Created state"

    def execute ( self, data ):
        self.client.wait_for_server ()

        goal = HeadingControlGoal ( topic = "postman_yaw", heading = 0, is_body_frame = True )

        self.client.send_goal ( goal )

        answer = False

        while answer != True:
            print "ASD"
            if self.client.wait_for_result ( timeout = rospy.Duration ( 1 ) ) == True:
                answer = True

            if self.preempt_requested ():
                print "PREEMPT"
                self.service_preempt ()
                return 'preempted'

        result = self.client.get_result()



class React ( smach.State ):
    def __init__ ( self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )

    def execute ( self, data ):
        print "--------- DONE!--------"

        return 'succeeded'



class ReactSpeeding ( smach.State ):
    def __init__ ( self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )

    def execute ( self, data ):
        print "--------- SPEEDING DETECTED!--------"

        return 'succeeded'



def task ():
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )

    cm = smach.Concurrence ( outcomes = ['succeeded', 'speeding'],
                             default_outcome = 'succeeded',
                             child_termination_cb = termination_callback,
                             outcome_cb = outcome_callback )

    with cm:
        cm.add ( 'ESCORT',
                 Escort () )

        cm.add ( 'SPEEDING_DETECTOR',
                 smach_ros.MonitorState ( '/speeding', String, speeding_callback ) )
                 #transitions = { 'valid': 'SPEEDING_DETECTOR',
                 #                'invalid': 'REACT_SPEEDING',
                 #                'preempted': 'REACT' } )
    
    with sm:
      sm.add ( 'ESCORT_INSIDE',
               cm,
               transitions = { 'succeeded': 'REACT',
                               'speeding': 'REACT_SPEEDING' } )

      sm.add ( 'REACT',
               React (),
               transitions = { 'succeeded': 'succeeded' } )

      sm.add ( 'REACT_SPEEDING',
               ReactSpeeding (),
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
