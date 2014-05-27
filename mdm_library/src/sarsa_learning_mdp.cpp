/**\file sarsa_learning_mdp.cpp
 *
 * Author:
 * Pedro Resende <pt.resende@gmail.com>
 *
 * Markov Decision Making is a ROS library for robot decision-making based on MDPs.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of Markov Decision Making.
 *
 * Markov Decision Making is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Markov Decision Making is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <mdm_library/sarsa_learning_mdp.h>
#include <mdm_library/Policy.h>


using namespace std;
using namespace mdm_library;


#ifdef HAVE_MADP



SarsaLearningMDP::
SarsaLearningMDP ( ALPHA_TYPE alpha_type,
                   EPSILON_TYPE epsilon_type,
                   CONTROLLER_TYPE controller_type,
                   const string& problem_file_path,
                   const string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, policy_file_path, problem_file_path, initial_status )
{
    // Create the controller
    if ( controller_type == EVENT )
        controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerEventMDP ( policy_file_path,
                                                                                    epsilon_type,
                                                                                    initial_status );
    else
    {
        if ( controller_type == TIMED )
            controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerTimedMDP ( policy_file_path,
                                                                                        epsilon_type,
                                                                                        initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    state_sub_ = nh_.subscribe ( "state", 1, &SarsaLearningMDP::stateSymbolCallback, this );
    
    initializeQValues ();
}



#endif



SarsaLearningMDP::
SarsaLearningMDP ( ALPHA_TYPE alpha_type,
                   EPSILON_TYPE epsilon_type,
                   CONTROLLER_TYPE controller_type,
                   uint32_t num_states,
                   uint32_t num_actions,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, num_states, num_actions,
                        policy_file_path, initial_status )
{
    // Create the controller
    if ( controller_type == EVENT )
        controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerEventMDP ( policy_file_path,
                                                                                    epsilon_type,
                                                                                    initial_status );
    else
    {
        if ( controller_type == TIMED )
            controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerTimedMDP ( policy_file_path,
                                                                                        epsilon_type,
                                                                                        initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    state_sub_ = nh_.subscribe ( "state", 1, &SarsaLearningMDP::stateSymbolCallback, this );
    policy_pub_ = nh_.advertise<Policy> ( "policy", 0, true );
    
    initializeQValues ();
}



void
SarsaLearningMDP::
initializeQValues ()
{
    size_t num_states = controller_.get() -> getNumberOfStates ();
    size_t num_actions = controller_ .get() -> getNumberOfActions ();
    
    Matrix q_values_ ( num_states_, num_actions_ );
    
    // Initialize the Q values as 0
    for ( unsigned i = 0; i < q_values_.size1(); ++i )
        for ( unsigned j = 0; j < q_values_.size2(); ++j )
            q_values_ ( i, j ) = 0;
}



void
SarsaLearningMDP::
updateQValues ()
{
    if ( alpha_type_ != ALPHA_CONSTANT )
        alpha_ = updateAlpha ( alpha_type_, curr_decision_ep_ );
    
    q_values_ ( state_, action_ ) = q_values_ ( state_, action_ ) + alpha_ * ( reward_ + gamma_ *
                                    q_values_ ( next_state_, next_action_ ) - q_values_ ( state_, action_ ) );
}



void
SarsaLearningMDP::
updatePolicy ()
{
    boost::shared_ptr<MDPPolicy> policy;
    
    policy = ( *controller_ ).getPolicy ();
    
    policy -> updatePolicy ( q_values_ );
}



void
SarsaLearningMDP::
stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg )
{
    curr_decision_ep_ = ( *controller_ ).getDecisionEpisode ();
    reward_ = ( *controller_ ).getReward ();
    
    if ( curr_decision_ep_ == 0 )
    {
        state_ = msg -> world_symbol;
        action_ = ( *controller_ ).getAction ();
    }
    else
    {
        if ( curr_decision_ep_ == 1 )
        {
            next_state_ = msg -> world_symbol;
            next_action_ = ( *controller_ ).getAction ();
        }
        else
        {
            state_ = next_state_;
            next_state_ = msg -> world_symbol;
            
            action_ = next_action_;
            next_action_ = ( *controller_ ).getAction ();
        }
    }
    
    if ( curr_decision_ep_ % policy_update_frequency_ == 0 )
    {
        updatePolicy ();
        publishPolicy ();
    }
}



void
SarsaLearningMDP::
publishPolicy ()
{
    Policy pol;
    boost::shared_ptr<MDPPolicy> policy;
    IndexVector v;
    
    policy = ( *controller_ ).getPolicy ();
    v = ( * ( *policy ).getVector () );
    
    pol.number_of_states = ( *controller_ ).getNumberOfStates ();
    
    for ( int i = 0; i < v.size(); i++ )
        pol.policy[i] = v[i];
    
    policy_pub_.publish ( pol );
}
