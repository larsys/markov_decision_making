#!/usr/bin/env python

import os
import rospy
import sys

from PyQt4 import QtGui, QtCore

from std_msgs.msg import Float32
from mdm_library.msg import WorldSymbol
from mdm_library.msg import ActionSymbol
#from mdm_library import PolicySymbol


global gui


class MainWindow (QtGui.QWidget):
    #
    # Constructor for the MainWindow class.
    #
    def __init__ (self):
        super ( MainWindow, self ).__init__ ()
        
        # ROS subscribers
        self.sub_state = rospy.Subscriber ( "/state", WorldSymbol, callback_state )
        self.sub_action = rospy.Subscriber ( "/action", ActionSymbol, callback_action )
        self.sub_reward = rospy.Subscriber ( "/reward", Float32, callback_reward )
        #self.sub_policy = rospy.Subscriber ( "/policy", String, callback )
        
        # Class variables for the GUI
        self.label_state = QtGui.QLabel ( 'Current State' )
        self.label_action = QtGui.QLabel ( 'Last Action' )
        self.label_reward = QtGui.QLabel ( 'Last Reward' )
        self.label_policy = QtGui.QLabel ( 'Current Policy' )
        self.button_quit = QtGui.QPushButton ( 'Quit', self )
        self.text_state = QtGui.QPlainTextEdit ()
        self.text_action = QtGui.QPlainTextEdit ()
        self.text_reward = QtGui.QPlainTextEdit ()
        self.text_policy = QtGui.QPlainTextEdit ()
        self.grid = QtGui.QGridLayout ()
        
        self.initUI ()
    
    #
    # Builds the GUI.
    #
    def initUI (self):
        # Set the tooltip properties
        QtGui.QToolTip.setFont ( QtGui.QFont ( 'SansSerif', 10 ) )
        
        
        # Tooltips for the QLabels
        self.label_state.setToolTip ( 'Shows the current state of the agent' )
        self.label_action.setToolTip ( 'Shows the last action performed by the agent' )
        self.label_reward.setToolTip ( 'Shows the last reward received by the agent' )
        self.label_policy.setToolTip ( 'Shows the current version of the learning policy' )


        # Set the functionality and the tooltip for the QPushButtons
        self.button_quit.clicked.connect ( QtCore.QCoreApplication.instance ().quit )
        self.button_quit.resize ( self.button_quit.sizeHint () )
        self.button_quit.setToolTip ( 'Quit the vizualiser' )


        # Set the QPlainTextEdits to read only
        self.text_state.setReadOnly ( True )
        self.text_action.setReadOnly ( True )
        self.text_reward.setReadOnly ( True )
        self.text_policy.setReadOnly ( True )


        # Layout definitions
        self.grid.setSpacing (10)

        # Add widgets to layout
        self.grid.addWidget ( self.label_state, 1, 0 )
        self.grid.addWidget ( self.text_state, 1, 1 )

        self.grid.addWidget ( self.label_action, 2, 0 )
        self.grid.addWidget ( self.text_action, 2, 1 )

        self.grid.addWidget ( self.label_reward, 3, 0 )
        self.grid.addWidget ( self.text_reward, 3, 1 )
        
        self.grid.addWidget ( self.label_policy, 4, 0 )
        self.grid.addWidget ( self.text_policy, 4, 1 )
        
        self.grid.addWidget ( self.button_quit, 5, 1 )
        
        self.setLayout ( self.grid )
        
        
        # Window properties
        self.resize ( 500, 312 )
        self.center ()
        self.setWindowTitle ( 'MDM Learning Visualizer' )
        self.setWindowIcon ( QtGui.QIcon ( os.path.join ( os.path.dirname( os.path.realpath ( __file__ ) ), 'socrob.png' ) ) )
        self.show ()
    
    #
    # Centers the window on the screen.
    #
    def center (self):
        
        qr = self.frameGeometry ()
        cp = QtGui.QDesktopWidget ().availableGeometry ().center ()
        qr.moveCenter ( cp )
        self.move ( qr.topLeft () )



#
# Callback for the state
#
def callback_state ( data ):
    s = QtCore.QString ( data.world_symbol )
    gui.text_state.setPlainText ( s )



#
# Callback for the action
#
def callback_action ( data ):
    s = QtCore.QString ( data.action_symbol )
    gui.text_action.setPlainText ( s )
    
    
    
#
# Callback for the reward
#
def callback_reward ( data ):
    s = QtCore.QString ( str ( data.data ) )
    gui.text_reward.setPlainText ( s )
    
    
    
#
# Callback for the policy
#
def callback_policy ( data ):
    s = QtCore.QString ( data.data )
    gui.text_policy.setPlainText ( s )
    
    
    
#
# The main function initializes the GUI and the ROS node.
#
def main ():
    
    app = QtGui.QApplication ( sys.argv )
    
    global gui
    gui = MainWindow ()
    
    rospy.init_node ( 'VisGUI', anonymous = True )
    
    app.exec_()
    #sys.exit ( app.exec_ () )



if __name__ == "__main__":
    main ()
