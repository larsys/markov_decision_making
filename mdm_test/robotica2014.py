#! /usr/bin/env python

import sys
import math
import random

import roslib; roslib.load_manifest('mdm_behaviors')
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



# Position near the door so that the robot can open it
DOOR_X = -9.8519
DOOR_Y = -7.2850
DOOR_T = 0

# Ideling position inside the house
DROP_X = 4.1143
DROP_Y = -7.5501
DROP_T = 0

# Final position for the demo
FINAL_X = 3.8461
FINAL_Y = -20.7156
FINAL_T = 0



def pose2alib ( x, y, t ):
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



def doorbell_callback ( userdata, msg ):
    doorbell = 'nothing'
    doorbell = msg.data

    if doorbell == 'RING' or doorbell == 'DOOR':
        return False



def speeding_callback ( userdata, msg ):
    speeding = 'nothing'
    speeding = msg.data

    if speeding == 'SPEEDING_DETECTED':
        return False



def dialog_callback ( data ):
    global dialog_state
    dialog_state = data.data



def termination_callback ( outcome_map ):
    return True



def get_the_door_outcome_callback ( outcome_map ):
    if outcome_map['ASK_VISITOR_INTENTIONS'] == 'registered_mail' or outcome_map['GUI_REPLACER_GET_THE_DOOR'] == 'registered_mail':
        return 'registered_mail'

    if outcome_map['ASK_VISITOR_INTENTIONS'] == 'regular_mail' or outcome_map['GUI_REPLACER_GET_THE_DOOR'] == 'regular_mail':
        return 'regular_mail'

    if outcome_map['ASK_VISITOR_INTENTIONS'] == 'no_answer':
        return 'no_answer'



def ask_for_escort_outside_outcome_callback ( outcome_map ):
    return 'escort'



def escort_outcome_callback ( outcome_map ):
    if outcome_map['ESCORT'] == 'succeeded':
        return 'succeeded'

    if outcome_map['SPEEDING_DETECTOR'] == 'invalid':
        return 'speeding'



class AskVisitorIntentions ( smach.State ):
    def __init__ (self):
        smach.State.__init__ ( self, outcomes = ['registered_mail', 'regular_mail', 'no_answer'] )
        self.dialogue_client = actionlib.SimpleActionClient ( 'do_dialogue', DialogueControlAction )
        
    def execute (self, data):
        self.dialogue_client.wait_for_server ()

        goal = DialogueControlGoal ( result = 1 )

        self.dialogue_client.send_goal ( goal )

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
    def __init__ ( self, sentence, options, outcomes ):
        smach.State.__init__ ( self, outcomes )
        self.client = actionlib.SimpleActionClient ( 'gui_server', GuiActAction )

        self.sentence = sentence
        self.options = options
        self.outcomes = outcomes

    def execute ( self, data ):
        self.client.wait_for_server ()

        goal = GuiActGoal ( self.sentence, self.options )

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



class StateSpeak ( smach.State ):
    def __init__ ( self, message = None ):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )
        self.message = message
        self.pub = rospy.Publisher ( "speech", String, latch = True )

    def execute ( self, data ):
        msg = data.message if self.message is None else self.message
        self.pub.publish ( String ( msg ) )

        return 'succeeded'



class GraspObject ( smach.State ):
    def __init__ ( self ):
        smach.State.__init__ ( self, outcomes = ['succeeded', 'failed'] )
        self.client = actionlib.SimpleActionClient ( 'grasp_marker', GraspMarkerAction )

    def execute ( self, data ):
        self.client.wait_for_server ()

        goal = GraspMarkerGoal ( x_offset = 0, z_offset = -0.05 )

        self.client.send_goal ( goal )

        self.client.wait_for_result ()

        result = self.client.get_result ().success

        if result == True:
            return 'succeeded'
        else:
            return 'failed'



class EscortInside ( smach.State ):
    def __init__ (self):
        smach.State.__init__ ( self, outcomes = ['succeeded'] )
        self.client = actionlib.SimpleActionClient ( 'heading_control', HeadingControlAction )

    def execute (self, data):
        self.client.wait_for_server ()

        goal = HeadingControlGoal ( topic = "postman_yaw", heading = 0, is_body_frame = True )

        self.client.send_goal ( goal )

        answer = False

        while answer != True:
            if self.client.wait_for_result ( timeout = rospy.Duration ( 1 ) ) == True:
                answer = True

            if self.preempt_requested ():
                self.service_preempt ()
                return 'preempted'

        result = self.client.get_result()



class AskForInstructions ( smach.State ):
    def __init__ (self):
        smach.State.__init__ ( self, outcomes = ['escort'] )
        self.dialogue_client = actionlib.SimpleActionClient ( 'do_dialogue', DialogueControlAction )
        
    def execute (self, data):
        self.dialogue_client.wait_for_server ()

        # TODO: change the result value for this goal to launch the other dialogue
        goal = DialogueControlGoal ( result = 1 )

        self.dialogue_client.send_goal ( goal )

        answer = False

        while answer != True:
            if self.dialogue_client.wait_for_result ( timeout = rospy.Duration ( 1 ) ) == True:
                answer = True

            if self.preempt_requested ():
                self.service_preempt ()
                return 'preempted'

        result = self.dialogue_client.get_result()

        if result == 'ESCORT':
            return 'escort'



class EscortOuside ( smach.State ):
    def __init__ (self):
        smach.State.__init__ ( self, outcomes = ['succeeded', 'bad_behavior_detected', 'failed'] )
        self.client = actionlib.SimpleActionClient ( 'heading_control', HeadingControlAction )
        
    def execute (self, data):
        self.client.wait_for_server ()

        goal = HeadingControlGoal ( topic = "postman_yaw", heading = 0, is_body_frame = True )

        self.client.send_goal ( goal )

        answer = False

        while answer != True:
            if self.client.wait_for_result ( timeout = rospy.Duration ( 1 ) ) == True:
                answer = True

            if self.preempt_requested ():
                self.service_preempt ()
                return 'preempted'

        result = self.client.get_result()



class DropObject ( smach.State ):
    def __init__ (self):
        smach.State.__init__ ( self, outcomes = ['succeeded', 'failed'] )
        
    def execute (self, data):
        return 'succeeded'



def task ():
    # Concurrent FSMs for the dialog and GUI components
    cm_get_the_door = smach.Concurrence ( outcomes = ['registered_mail', 'regular_mail', 'no_answer'],
                                          default_outcome = 'no_answer',
                                          child_termination_cb = termination_callback,
                                          outcome_cb = get_the_door_outcome_callback )

    cm_ask_for_escort_outside = smach.Concurrence ( outcomes = ['escort'],
                                                    default_outcome = 'escort',
                                                    child_termination_cb = termination_callback,
                                                    outcome_cb = ask_for_escort_outside_outcome_callback )


    # Concurrent FSMs for the escort states and speeding detectors
    cm_escort_inside = smach.Concurrence ( outcomes = ['succeeded', 'speeding'],
                                           default_outcome = 'succeeded',
                                           child_termination_cb = termination_callback,
                                           outcome_cb = escort_outcome_callback )

    cm_escort_outside = smach.Concurrence ( outcomes = ['succeeded', 'speeding'],
                                            default_outcome = 'succeeded',
                                            child_termination_cb = termination_callback,
                                            outcome_cb = escort_outcome_callback )


    with cm_get_the_door:
        cm_get_the_door.add ( 'ASK_VISITOR_INTENTIONS',
                              AskVisitorIntentions () )

        cm_get_the_door.add ( 'GUI_REPLACER_GET_THE_DOOR',
                              GuiReplacer ( sentence = 'What kind of mail is it?',
                                            options = ['Registered Mail', 'Regular Mail'],
                                            outcomes = ['registered_mail', 'regular_mail', 'no_answer', 'preempted']) )
    

    with cm_ask_for_escort_outside:
        cm_ask_for_escort_outside.add ( 'ASK_FOR_INSTRUCTIONS',
                                        AskForInstructions () )

        cm_ask_for_escort_outside.add ( 'GUI_REPLACER_ASK_FOR_ESCORT_OUTSIDE',
                                        GuiReplacer ( sentence = 'When you are ready, please ask me to escort you outside.',
                                                      options = ['Please escort me outside.'],
                                                      outcomes = ['escort', 'preempted'] ) )

    with cm_escort_inside:
        cm_escort_inside.add ( 'ESCORT_INSIDE',
                 EscortInside () )

        cm_escort_inside.add ( 'INSIDE_SPEEDING_DETECTOR',
                 smach_ros.MonitorState ( '/speeding', String, speeding_callback ) )

    with cm_escort_outside:
        cm_escort_outside.add ( 'ESCORT_OUTSIDE',
                 EscortOuside () )

        cm_escort_outside.add ( 'OUTSIDE_SPEEDING_DETECTOR',
                 smach_ros.MonitorState ( '/speeding', String, speeding_callback ) )


    # The top-level FSM
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )

    with sm:
      sm.add ( 'STANDBY_FOR_DOORBELL',
               smach_ros.MonitorState ( '/bell_recog', String, doorbell_callback ),
               transitions = { 'valid': 'STANDBY_FOR_DOORBELL',
                               'invalid': 'MOVE_2_DOOR',
                               'preempted': 'STANDBY_FOR_DOORBELL' } )

      sm.add ( 'MOVE_2_DOOR',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( DOOR_X, DOOR_Y, DOOR_T ) ),
               transitions = { 'succeeded': 'GET_THE_DOOR',
                               'aborted': 'aborted' } )
        
      sm.add ( 'GET_THE_DOOR',
               cm_get_the_door,
               transitions = { 'registered_mail': 'ESCORT_INSIDE',
                               'regular_mail': 'GRASP_OBJECT',
                               'no_answer': 'RAISE_ALARM' } )
      
      sm.add ( 'GRASP_OBJECT',
               GraspObject (),
               transitions = { 'succeeded': 'SEND_AWAY',
                               'failed': 'GRASP_OBJECT' } )
      
      sm.add ( 'SEND_AWAY',
               StateSpeak ( 'Thank you for the delivery.' ),
               transitions = { 'succeeded': 'MOVE_2_DROPPING_POSITION' } )
               
      sm.add ( 'MOVE_2_DROPPING_POSITION',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( DROP_X, DROP_Y, DROP_T ) ),
               transitions = { 'succeeded': 'DROP_OBJECT',
                               'aborted': 'aborted' } )
      
      sm.add ( 'DROP_OBJECT',
               DropObject (),
               transitions = { 'succeeded': 'STOP',
                               'failed': 'DROP_OBJECT' } )

      sm.add ( 'ESCORT_INSIDE',
               cm_escort_inside,
               transitions = { 'succeeded': 'ASK_FOR_ESCORT_OUTSIDE',
                               'speeding': 'RAISE_ALARM' } )
      
      sm.add ( 'ASK_FOR_ESCORT_OUTSIDE',
               cm_ask_for_escort_outside,
               transitions = { 'escort': 'ESCORT_OUTSIDE' } )
      
      sm.add ( 'ESCORT_OUTSIDE',
               cm_escort_outside,
               transitions = { 'succeeded': 'STOP',
                               'speeding': 'RAISE_ALARM'} )
      
      sm.add ( 'RAISE_ALARM',
               StateSpeak ( 'Warning! The postman is behaving badly!' ),
               transitions = { 'succeeded': 'STOP' } )
      
      sm.add ( 'STOP',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( FINAL_X, FINAL_Y, FINAL_T ) ),
               transitions = { 'succeeded': 'STANDBY_FOR_DOORBELL',
                               'aborted': 'aborted' } )
    
    return sm



def main ( argv ):
    # Init ROS
    rospy.init_node( "robotica2014_task" )
        
    # Define the task
    sm = task()
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer ( 'robotica2014_task', sm, '/SM_ROOT' )
    sis.start()

    # Execute state machine
    rospy.loginfo ( "Executing Robotica 2014 state machine" )
    sm.execute()

    sis.stop()



if __name__ == '__main__':
    main ( sys.argv )
