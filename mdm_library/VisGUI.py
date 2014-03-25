#!/usr/bin/env python

import os
import rospy
import sys

from Tkinter import *
import tkFont
import pylab

from std_msgs.msg import Float32
from mdm_library.msg import WorldSymbol
from mdm_library.msg import ActionSymbol
from mdm_library.msg import Policy



class Application ( Frame ):
    def __init__ ( self, master = None ):
        Frame.__init__( self, master )
        
        # ROS Subscribers
        self.sub_state = rospy.Subscriber   ( "/state",     WorldSymbol,    self.callback_state )
        self.sub_action = rospy.Subscriber  ( "/action",    ActionSymbol,   self.callback_action )
        self.sub_reward = rospy.Subscriber  ( "/reward",    Float32,        self.callback_reward )
        self.sub_policy = rospy.Subscriber  ( "/policy",    Float32,        self.callback_policy )
        
        # Text Variables
        self.state = StringVar ()
        self.action = StringVar ()
        self.reward = StringVar ()
        self.ep = StringVar ()
        self.policy = StringVar ()
        
        # Font
        self.custom_font = tkFont.Font( family = "Helvetica", size = 11 )
        
        # GUI Widgets
        self.button_quit      = Button ( self, text = "Quit",       command = self.quit,        font = self.custom_font, anchor = E )
        self.button_show_plot = Button ( self, text = "Show Plot",  command = self.showPlot,    font = self.custom_font, anchor = E )
        
        self.label_state    = Label ( self, text = "Current State:",            anchor = NW,    font = self.custom_font )
        self.label_action   = Label ( self, text = "Last Action:",              anchor = NW,    font = self.custom_font )
        self.label_reward   = Label ( self, text = "Last Reward:",              anchor = NW,    font = self.custom_font )
        self.label_ep       = Label ( self, text = "Current Decision Episode:", anchor = NW,    font = self.custom_font )
        self.label_policy   = Label ( self, text = "Current Policy:",           anchor = NW,    font = self.custom_font )
        
        self.text_state     = Label ( self, textvariable = self.state,  font = self.custom_font )
        self.text_action    = Label ( self, textvariable = self.action, font = self.custom_font )
        self.text_reward    = Label ( self, textvariable = self.reward, font = self.custom_font )
        self.text_ep        = Label ( self, textvariable = self.ep,     font = self.custom_font )
        self.text_policy    = Label ( self, textvariable = self.policy, font = self.custom_font )
        
        # Pack and setup the widgets
        self.pack ( fill = "both", expand = True, padx = 20, pady = 20 )
        self.setupWidgets ()


    def setupWidgets ( self ):
        # Buttons
        self.button_quit.grid       ( row = 6, column = 1 )
        self.button_show_plot.grid  ( row = 5, column = 1 )
        
        # Labels
        self.label_state.grid   ( row = 0, column = 0 )
        self.label_action.grid  ( row = 1, column = 0 )
        self.label_reward.grid  ( row = 2, column = 0 )
        self.label_ep.grid      ( row = 3, column = 0 )
        self.label_policy.grid  ( row = 4, column = 0 )
        
        # Texts
        self.text_state.grid    ( row = 0, column = 1 )
        self.text_action.grid   ( row = 1, column = 1 )
        self.text_reward.grid   ( row = 2, column = 1 )
        self.text_ep.grid       ( row = 3, column = 1 )
        self.text_policy.grid   ( row = 4, column = 1 )
        
        
    
    #
    # Callback for the Show Plot button
    #
    def showPlot ( self ):
        x=pylab.arange(0,2,0.01)
        y=2*pylab.sin(2*pylab.pi*(x-1/4))

        pylab.plot(x,y)
        pylab.xlabel('x-axis')
        pylab.ylabel('y-axis')
        pylab.title(r'$y=2\sin (2\pi(x-1/4))$')

        pylab.show()
        
        
        
    #
    # Callback for the state
    #
    def callback_state ( self, data ):
        self.state.set ( str ( data.world_symbol ) )



    #
    # Callback for the action and decision episode
    #
    def callback_action ( self, data ):
        self.ep.set ( str ( data.decision_episode ) )
        self.action.set ( str ( data.action_symbol ) )
        
        
        
    #
    # Callback for the reward
    #
    def callback_reward ( self, data ):
        self.reward.set ( str ( data.data ) )
        
        
        
    #
    # Callback for the policy
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
    
    # Set the application icon
    icon = PhotoImage ( file = os.path.join ( os.path.dirname ( os.path.realpath ( __file__ ) ), 'socrob.gif' ) )
    root.tk.call ( 'wm', 'iconphoto', root._w, icon )
    
    # Set the window's width and height and center it
    w = 280
    h = 200
    
    ws = root.winfo_screenwidth ()
    hs = root.winfo_screenheight ()

    x = ws / 2 - w / 2
    y = hs / 2 - h / 2

    root.geometry ( '%dx%d+%d+%d' % ( w, h, x, y ) )
    
    # Run the application
    app = Application ( master = root )
    app.mainloop ()
    root.destroy ()
    




if __name__ == "__main__":
    main ()
