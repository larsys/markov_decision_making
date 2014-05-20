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



def pose2alib ( x, y, t ):
    print "MOVING TO:" + " X - " + str(x) + " Y - " + str(y) + " T - " + str(t)
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



class StateSpeak(smach.State):
    def __init__(self, message = None):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.message = message
        self.pub = rospy.Publisher("speech", String, latch = True)                                                                                   
    def execute(self, data):
        msg = data.message if self.message is None else self.message
        self.pub.publish(String(msg))
                                                                              
        return 'succeeded'



def task ():
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )
    
    with sm:
      sm.add ( 'SPEAK_1',
               StateSpeak ("asd"),
               transitions = { 'succeeded': 'SPEAK_2' } )

      sm.add ( 'SPEAK_2',
               StateSpeak ("Test! Test! Test! Test! Test!"),
               transitions = { 'succeeded': 'succeeded' } )
      
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
