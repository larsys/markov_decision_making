#!/usr/bin/env python

import os
import rospy
import sys

from Tkinter import *
import tkFont
import pylab
import matplotlib.pyplot as plt

from std_msgs.msg       import Float32
from mdm_library.msg    import WorldSymbol
from mdm_library.msg    import ActionSymbol
from mdm_library.msg    import Policy



class Application ( Frame ):
    def __init__ ( self, state_topic, action_topic, reward_topic, policy_topic, master = None ):
        Frame.__init__( self, master )
        
        # ROS Subscribers
        self.sub_state  = rospy.Subscriber  ( state_topic,     WorldSymbol,    self.callback_state  )
        self.sub_action = rospy.Subscriber  ( action_topic,    ActionSymbol,   self.callback_action )
        self.sub_reward = rospy.Subscriber  ( reward_topic,    Float32,        self.callback_reward )
        self.sub_policy = rospy.Subscriber  ( policy_topic,    Float32,        self.callback_policy )
        
        # Text Variables
        self.state  =   StringVar ()
        self.action =   StringVar ()
        self.reward =   StringVar ()
        self.ep     =   StringVar ()
        self.policy =   StringVar ()
        
        # Plot Variables
        self.acc_reward = [0, 1, 2, 3, 4, 5, 6, 7, 8, 15, 20, 40]
        self.num_rewards = pylab.arange ( 0, len ( self.acc_reward ), 1 )
        
        # Font
        self.custom_font = tkFont.Font( family = "Helvetica", size = 11 )
        
        # GUI Widgets
        self.button_quit         = Button ( self, text = "Quit",         command = self.quit,        font = self.custom_font )
        self.button_show_plot    = Button ( self, text = "Show Plot",    command = self.showPlot,    font = self.custom_font )
        self.button_refresh_plot = Button ( self, text = "Refresh Plot", command = self.refreshPlot, font = self.custom_font )
        
        self.label_state    = Label ( self, text = "Current State:",            font = self.custom_font )
        self.label_action   = Label ( self, text = "Last Action:",              font = self.custom_font )
        self.label_reward   = Label ( self, text = "Last Reward:",              font = self.custom_font )
        self.label_ep       = Label ( self, text = "Current Decision Episode:", font = self.custom_font )
        self.label_policy   = Label ( self, text = "Current Policy:",           font = self.custom_font )
        
        self.text_state     = Label ( self, textvariable = self.state,  font = self.custom_font )
        self.text_action    = Label ( self, textvariable = self.action, font = self.custom_font )
        self.text_reward    = Label ( self, textvariable = self.reward, font = self.custom_font )
        self.text_ep        = Label ( self, textvariable = self.ep,     font = self.custom_font )
        self.text_policy    = Label ( self, textvariable = self.policy, font = self.custom_font )
        
        # Pack and setup the widgets
        self.pack ( fill = "both", expand = True, padx = 20, pady = 20 )
        self.setupWidgets ()
        
        # Configure the rows and columns to expand and contract with the window
        for x in xrange ( 7 ):
            Grid.rowconfigure ( self, x, weight = 1 )
            
        for y in xrange ( 2 ):
            Grid.columnconfigure ( self, y, weight = 1 )



    #
    # Setup the widgets in a grid
    #
    def setupWidgets ( self ):
        # Buttons
        self.button_quit         .grid ( row = 7, column = 1, sticky = E )
        self.button_show_plot    .grid ( row = 5, column = 1, sticky = E )
        self.button_refresh_plot .grid ( row = 6, column = 1, sticky = E )
        
        # Labels
        self.label_state         .grid ( row = 0, column = 0, sticky = W )
        self.label_action        .grid ( row = 1, column = 0, sticky = W )
        self.label_reward        .grid ( row = 2, column = 0, sticky = W )
        self.label_ep            .grid ( row = 3, column = 0, sticky = W )
        self.label_policy        .grid ( row = 4, column = 0, sticky = W )
        
        # Texts
        self.text_state          .grid ( row = 0, column = 1 )
        self.text_action         .grid ( row = 1, column = 1 )
        self.text_reward         .grid ( row = 2, column = 1 )
        self.text_ep             .grid ( row = 3, column = 1 )
        self.text_policy         .grid ( row = 4, column = 1 )
        
        
    
    #
    # Callback for the Show Plot button
    #
    def showPlot ( self ):
        self.figure = plt.figure ()
        self.ax = self.figure.add_subplot(111)
        self.ax.scatter ( self.num_rewards, self.acc_reward )
        self.ax.set_xlabel ( 'Decision Episode' )
        self.ax.set_ylabel ( 'Accumulated Reward' )
        self.ax.set_title  ( 'Accumulated Reward' )

        plt.show ()
        
        
        
    #
    # Callback for the Refresh Plot button
    #
    def refreshPlot ( self ):
        pylab.show ()
        
        
        
    #
    # ROS Callback for the state
    #
    def callback_state ( self, data ):
        self.state.set ( str ( data.world_symbol ) )



    #
    # ROS Callback for the action and decision episode
    #
    def callback_action ( self, data ):
        self.ep.set ( str ( data.decision_episode ) )
        self.action.set ( str ( data.action_symbol ) )
        
        
        
    #
    # ROS Callback for the reward
    #
    def callback_reward ( self, data ):
        self.reward.set ( str ( data.data ) )
        
        self.acc_reward.append ( data.data )
        self.num_rewards = pylab.arange ( 0, len ( self.acc_reward ), 1 )
        
        self.ax.scatter ( self.num_rewards, self.acc_reward )
        
        
        
    #
    # ROS Callback for the policy
    #
    def callback_policy ( self, data ):
        pol = [ 0 * data.number_of_states ]
        i = 0
        
        for element in data.policy:
            pol[i] = str ( element )
            i += 1
        
        self.policy.set ( ' '.join ( pol ) )


    
#
# The main function initializes the GUI and the ROS node.
#
def main ():
    # Create the ROS node
    rospy.init_node ( 'VisGUI', anonymous = True )
    
    # Create the Tkinter node
    root = Tk ()
    
    # Set the window's title
    root.title ( "MDM Learning Visualizer" )
    
    # Set the window's width and height and center it
    w = 350
    h = 220
    
    ws = root.winfo_screenwidth ()
    hs = root.winfo_screenheight ()

    x = ws / 2 - w / 2
    y = hs / 2 - h / 2

    root.geometry ( '%dx%d+%d+%d' % ( w, h, x, y ) )
    
    # Get the topic names from params
    try:
        state_topic  = rospy.get_param ( "/mdm_visualizer/state_topic" )
        action_topic = rospy.get_param ( "/mdm_visualizer/action_topic" )
        reward_topic = rospy.get_param ( "/mdm_visualizer/reward_topic" )
        policy_topic = rospy.get_param ( "/mdm_visualizer/policy_topic" )
    except:
        print "\n\nPlease set the parameters state_topic, action_topic, reward_topic and policy_topic. Each of them should"
        print "contain the topics where the respective information is being published.\n\n"
        sys.exit ( 0 )
    
    # Run the application
    app = Application ( state_topic, action_topic, reward_topic, policy_topic, master = root )
    app.mainloop ()
    root.destroy ()
    




if __name__ == "__main__":
    main ()
