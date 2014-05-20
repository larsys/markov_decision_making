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
from scout_msgs.msg import *
from scout_msgs.srv import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *



def pose2alib ( x, y, t ):
    print "MOVING TO:" + " X - " + str(x) + " Y - " + str(y) + " T - " + str(t)
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



class AskVisitorIntentions ( smach.State ):
    def __init__(self):
        smach.State.__init__ ( self, outcomes = ['registered_mail', 'regular_mail', 'no_answer', 'preempted'] )
        self.dialogue_client = actionlib.SimpleActionClient('do_dialogue', DialogueControlAction)
 
    def execute ( self, data ):
        self.dialogue_client.wait_for_server()

        goal = DialogueControlGoal(result=1)

        self.dialogue_client.send_goal(goal)

        answer = False

        while answer != True:
            if self.dialogue_client.wait_for_result ( timeout = rospy.Duration ( 1 ) ) == True:
                answer = True

            if self.preempt_requested ():
                self.service_preempt ()
                return 'preempted'

        result = self.dialogue_client.get_result()

        if result == 'REGISTERED':
            return 'registered_mail'
        elif result == 'REGULAR':
            return 'regular_mail'
        elif result == 'NOANSWER':
            return 'no_answer'



class GuiReplacer ( smach.State ):
    def __init__(self, sentence, options, outcomes):
        smach.State.__init__ ( self, outcomes )
        self.client = actionlib.SimpleActionClient('gui_server', GuiActAction)

        self.sentence = sentence
        self.options = options
        self.outcomes = outcomes
        
    def execute ( self, data ):
        self.client.wait_for_server ()

        goal = GuiActGoal ( self.sentence, self.options  )

        self.client.send_goal ( goal )

        answer = False

        while answer != True:
            if self.client.wait_for_result ( timeout = rospy.Duration ( 1 ) ) == True:
                answer = True

            if self.preempt_requested ():
                self.service_preempt ()
                return 'preempted'

        result = self.client.get_result ().choice

        return self.outcomes[result]

        if int ( result ) == 0:
            return 'registered_mail'
        elif int ( result ) == 1:
            return 'regular_mail'
        


class ReactRegistered ( smach.State ):
    def __init__ ( self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )

    def execute ( self, data ):
        print "--------- REGISTERED!--------"

        return 'succeeded'


class ReactRegular ( smach.State ):
    def __init__ ( self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )

    def execute ( self, data ):
        print "--------- REGULAR!--------"

        return 'succeeded'


class ReactNo ( smach.State ):
    def __init__ ( self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )

    def execute ( self, data ):
        print "--------- NO ANSWER!--------"

        return 'succeeded'



def termination_callback ( outcome_map ):
    return True



def outcome_callback ( outcome_map ):
    if outcome_map['ASK_VISITOR_INTENTIONS'] == 'registered_mail' or outcome_map['GUI_REPLACER'] == 'registered_mail':
        return 'registered_mail'

    if outcome_map['ASK_VISITOR_INTENTIONS'] == 'regular_mail' or outcome_map['GUI_REPLACER'] == 'regular_mail':
        return 'regular_mail'

    if outcome_map['ASK_VISITOR_INTENTIONS'] == 'no_answer':
        return 'no_answer'



def task ():
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )

    cm = smach.Concurrence ( outcomes = ['registered_mail', 'regular_mail', 'no_answer'],
                             default_outcome = 'no_answer',
                             child_termination_cb = termination_callback,
                             outcome_cb = outcome_callback )
                             
    
    with cm:
      cm.add ( 'ASK_VISITOR_INTENTIONS',
               AskVisitorIntentions () )

      cm.add ( 'GUI_REPLACER',
               GuiReplacer ( sentence = 'What kind of mail is it?', options = ['Registered Mail', 'Regular Mail'], outcomes = ['registered_mail', 'regular_mail', 'no_answer', 'preempted'] ) )


    with sm:
      sm.add ( 'DIALOG',
               cm,
               transitions = { 'registered_mail': 'REACT_REGISTERED',
                               'regular_mail': 'REACT_REGULAR',
                               'no_answer': 'REACT_NO' } )

      sm.add ( 'REACT_REGISTERED',
               ReactRegistered (),
               transitions = { 'succeeded': 'succeeded' } )
      

      sm.add ( 'REACT_REGULAR',
               ReactRegular (),
               transitions = { 'succeeded': 'succeeded' } )

      sm.add ( 'REACT_NO',
               ReactNo (),
               transitions = { 'succeeded': 'succeeded' } )

      sm.add ( 'FAILURE',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( 1, 1, 1 ) ) )
    
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
