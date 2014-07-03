/**\file q_learning_mdp.cpp
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


#include <mdm_library/q_learning_mdp.h>
#include <mdm_library/Policy.h>


using namespace std;
using namespace mdm_library;


#ifdef HAVE_MADP



QLearningMDP::
QLearningMDP ( ALPHA_TYPE alpha_type,
               EPSILON_TYPE epsilon_type,
               CONTROLLER_TYPE controller_type,
               const string& problem_file_path,
               const std::string& initial_learning_policy_file_path,
               const string& policy_file_path,
               const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, policy_file_path, problem_file_path, initial_status )
{
    try
    {
        if ( learning_policy_ptr_ != 0 )
            ROS_WARN_STREAM ( "The learning policy for this MDP had already been loaded! Overwriting." );

        ifstream fp;
        fp.open ( initial_learning_policy_file_path.c_str() );
        IndexVectorPtr policy_vec ( new IndexVector() );

        fp >> ( *policy_vec );
        
        uint32_t number_of_states = q_values_.size1 ();
        uint32_t number_of_actions = q_values_.size2 ();
        
        MDPPolicy* learning_policy_ptr_ = new MDPEpsilonGreedyPolicyVector ( policy_vec,
                                                                             number_of_states,
                                                                             number_of_actions,
                                                                             epsilon_type );
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
    
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
    
    state_sub_ = nh_.subscribe ( "state", 1, &QLearningMDP::stateSymbolCallback, this );
    
    initializeQValues ();
}



#endif


QLearningMDP::
QLearningMDP ( float gamma,
               ALPHA_TYPE alpha_type,
               EPSILON_TYPE epsilon_type,
               CONTROLLER_TYPE controller_type,
               uint32_t num_states,
               uint32_t num_actions,
               const std::string& learning_policy_file_path,
               const std::string& policy_file_path,
               const std::string& reward_file_path,
               const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, num_states, num_actions,
                        policy_file_path, initial_status ),
    state_ ( 0 ),
    action_ ( 0 )
{
    try
    {
        if ( learning_policy_ptr_ != 0 )
            ROS_WARN_STREAM ( "The learning policy for this MDP had already been loaded! Overwriting." );

        ifstream fp;
        fp.open ( learning_policy_file_path.c_str() );
        IndexVectorPtr policy_vec ( new IndexVector() );

        fp >> ( *policy_vec );
        
        uint32_t number_of_states = num_states_;
        uint32_t number_of_actions = num_actions_;
        
        learning_policy_ptr_ = boost::shared_ptr<MDPPolicy> ( new MDPEpsilonGreedyPolicyVector ( policy_vec,
                                                                                                 number_of_states,
                                                                                                 number_of_actions,
                                                                                                 epsilon_type,
                                                                                                 learning_policy_file_path ) );
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
    
    // Create the controller
    if ( controller_type == EVENT )
        controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerEventMDP ( policy_file_path,
                                                                                    epsilon_type,
                                                                                    num_states,
                                                                                    num_actions,
                                                                                    initial_status );
    else
    {
        if ( controller_type == TIMED )
            controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerTimedMDP ( policy_file_path,
                                                                                        epsilon_type,
                                                                                        num_states,
                                                                                        num_actions,
                                                                                        initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    ( *controller_ ).loadRewardMatrix ( reward_file_path );
    
    state_sub_ = nh_.subscribe ( "state", 1, &QLearningMDP::stateSymbolCallback, this );
    policy_pub_ = nh_.advertise<Policy> ( "policy", 0, true );
    
    initializeQValues ();
}



void
QLearningMDP::
initializeQValues ()
{   
    size_t num_states = controller_.get() -> getNumberOfStates ();
    size_t num_actions = controller_ .get() -> getNumberOfActions ();

    q_values_ = Matrix ( num_states_, num_actions_ );
    
    // Initialize the Q values as 0
    for ( unsigned i = 0; i < q_values_.size1(); ++i )
        for ( unsigned j = 0; j < q_values_.size2(); ++j )
            q_values_ ( i, j ) = 0;
}



void
QLearningMDP::
updateQValues ()
{
    if ( alpha_type_ != ALPHA_CONSTANT )
        alpha_ = updateAlpha ( alpha_type_, curr_decision_ep_ );
    
    q_values_ ( state_, action_ ) = q_values_ ( state_, action_ ) + alpha_ * ( reward_ + gamma_ *
                                    maxOverA() - q_values_ ( state_, action_ ) );
}



void
QLearningMDP::
updatePolicy ()
{
    learning_policy_ptr_ -> updatePolicy ( q_values_ );
}



void
QLearningMDP::
stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg )
{   
    curr_decision_ep_ = ( *controller_ ).getDecisionEpisode ();
    reward_ = ( *controller_ ).getReward ();
    action_ = ( *controller_ ).getAction ();
    
    state_ = msg -> world_symbol;
    
    cout << "NEW EPISODE!!!!!!!!!!!!!!!!!!!!!!" << endl;
    cout << "\t\tDec Ep: " << curr_decision_ep_ << endl;
    cout << "\t\tReward: " << reward_ << endl;
    cout << "\t\tState: " << state_ << endl;
    cout << "\t\tAction: " << action_ << endl;
    
    updateQValues ();
    
    if ( curr_decision_ep_ % policy_update_frequency_ == 0 )
    {
        updatePolicy ();
        publishPolicy ();
    }
}



float
QLearningMDP::
maxOverA ()
{
    // Initialize the current maximum as negative infinity
    double curr_max = std::numeric_limits<double>::infinity() * -1;
    
    // Find the action that leads to the highest Q value
    for (unsigned j = 0; j < q_values_.size2(); j++ )
    {
        if ( q_values_ (next_state_, j) > curr_max )
            curr_max = q_values_ (next_state_, j);
    }
    
    return curr_max;
}



void
QLearningMDP::
publishPolicy ()
{
    Policy pol;
    IndexVector v;
    
    v = ( * ( *learning_policy_ptr_ ).getVector () );
    
    pol.number_of_states = ( *controller_ ).getNumberOfStates ();
    
    for ( int i = 0; i < v.size(); i++ )
        pol.policy[i] = v[i];
    
    policy_pub_.publish ( pol );
}
