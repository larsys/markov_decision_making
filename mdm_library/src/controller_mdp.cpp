/**\file controller_mdp.cpp
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
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

#include <float.h>
#include <fstream>

#include <boost/numeric/ublas/io.hpp>
#include <boost/iterator/iterator_concepts.hpp>

#include <mdm_library/controller_mdp.h>
#include <mdm_library/ActionSymbol.h>
#include <std_msgs/Float32.h>



using namespace std;
using namespace mdm_library;



#ifdef HAVE_MADP



ControllerMDP::
ControllerMDP ( const string& problem_file_path,
                const string& policy_file_path,
                const CONTROLLER_STATUS initial_status ) :
    R_ptr_(),
    policy_ptr_ (),
    loader_ ( new DecPOMDPLoader ( problem_file_path ) ),
    number_of_states_ (),
    number_of_actions_ (),
    state_sub_ ( nh_.subscribe ( "state", 0, &ControllerMDP::stateCallback, this ) ),
    action_pub_ ( nh_.advertise<ActionSymbol> ( "action", 0, true ) ),
    reward_pub_ ( nh_.advertise<std_msgs::Float32> ( "reward", 0, true ) )
{
    ///Note that this constructor is implicitly grabbing the number of *joint* actions and states.
    ///This is OK for single agent controllers as long as your problem file is also defined locally.
    ///(i.e. the size of the policy is coherent with the size of the model in problem_file)
    number_of_states_ = loader_->GetDecPOMDP()->GetNrStates();
    number_of_actions_ = loader_->GetDecPOMDP()->GetNrJointActions();
    loadPolicyVector ( policy_file_path );
}



ControllerMDP::
ControllerMDP ( const string& problem_file_path,
                const string& policy_file_path,
                EPSILON_TYPE epsilon_type,
                const CONTROLLER_STATUS initial_status ) :
    R_ptr_(),
    policy_ptr_ (),
    loader_ ( new DecPOMDPLoader ( problem_file_path ) ),
    number_of_states_ (),
    number_of_actions_ (),
    state_sub_ ( nh_.subscribe ( "state", 0, &ControllerMDP::stateCallback, this ) ),
    action_pub_ ( nh_.advertise<ActionSymbol> ( "action", 0, true ) ),
    reward_pub_ ( nh_.advertise<std_msgs::Float32> ( "reward", 0, true ) )
{
    ///Note that this constructor is implicitly grabbing the number of *joint* actions and states.
    ///This is OK for single agent controllers as long as your problem file is also defined locally.
    ///(i.e. the size of the policy is coherent with the size of the model in problem_file)
    number_of_states_ = loader_->GetDecPOMDP()->GetNrStates();
    number_of_actions_ = loader_->GetDecPOMDP()->GetNrJointActions();
    loadPolicyVector ( policy_file_path, epsilon_type );
}

#endif



ControllerMDP::
ControllerMDP ( const string& policy_file_path,
                const CONTROLLER_STATUS initial_status ) :
    ControlLayerBase ( initial_status ),
    R_ptr_(),
    policy_ptr_ (),
    loader_ (),
    number_of_states_ (),
    number_of_actions_ (),
    state_sub_ ( nh_.subscribe ( "state", 0, &ControllerMDP::stateCallback, this ) ),
    action_pub_ ( nh_.advertise<ActionSymbol> ( "action", 0, true ) ),
    reward_pub_ ( nh_.advertise<std_msgs::Float32> ( "reward", 0, true ) ),
    eps_greedy_ ( false )
//     republish_service_ ( nh_.advertiseService ( "publish_new_action", &ControllerMDP::republish_callback, this ) )
{
    ///This constructor will not publish problem metadata or reward (you will have to do it manually).
    loadPolicyVector ( policy_file_path );
    
    string reward_type;
    
    if ( private_nh_.hasParam ( "reward_type" ) )
    {
        private_nh_.getParam ( "reward_type", reward_type );
        reward_type_ = reward_type;
    }
    else
    {
        reward_type_ = "matrix";
        ROS_WARN_STREAM ( "No specified reward type. Assuming a reward type of matrix." );
    }
}



ControllerMDP::
ControllerMDP ( const string& policy_file_path,
                EPSILON_TYPE epsilon_type,
                uint32_t num_states,
                uint32_t num_actions,
                const CONTROLLER_STATUS initial_status ) :
    ControlLayerBase ( initial_status ),
    R_ptr_(),
    policy_ptr_ (),
    loader_ (),
    number_of_states_ ( num_states ),
    number_of_actions_ ( num_actions ),
    state_sub_ ( nh_.subscribe ( "state", 0, &ControllerMDP::stateCallback, this ) ),
    action_pub_ ( nh_.advertise<ActionSymbol> ( "action", 0, true ) ),
    reward_pub_ ( nh_.advertise<std_msgs::Float32> ( "reward", 0, true ) ),
    eps_greedy_ ( true )
//     republish_service_ ( nh_.advertiseService ( "publish_new_action", &ControllerMDP::republish_callback, this ) )
{
    ///This constructor will not publish problem metadata or reward (you will have to do it manually).
    loadPolicyVector ( policy_file_path, epsilon_type );
    
    string reward_type;
    
    if ( private_nh_.hasParam ( "reward_type" ) )
    {
        private_nh_.getParam ( "reward_type", reward_type );
        reward_type_ = reward_type;
    }
    else
    {
        reward_type_ = "matrix";
        ROS_WARN_STREAM ( "No specified reward type. Assuming a reward type of matrix." );
    }
}



// bool
// ControllerMDP::
// republish_callback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
// {
//     act ( last_state_ );
//     
//     //decrementDecisionEpisode();
//     
//     return true;
// }



void
ControllerMDP::
loadPolicyVector ( const string& policy_vector_path )
{
    try
    {
        if ( policy_ptr_ != 0 )
            ROS_WARN_STREAM ( "The policy for this MDP had already been loaded! Overwriting." );

        ifstream fp;
        fp.open ( policy_vector_path.c_str() );
        IndexVectorPtr policy_vec ( new IndexVector() );

        fp >> ( *policy_vec );
        
        policy_ptr_ = boost::shared_ptr<MDPPolicy> ( new MDPPolicyVector ( policy_vec ) );
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
}



void
ControllerMDP::
loadPolicyVector ( const string& policy_vector_path, EPSILON_TYPE epsilon_type )
{
    try
    {
        if ( policy_ptr_ != 0 )
            ROS_WARN_STREAM ( "The policy for this MDP had already been loaded! Overwriting." );

        ifstream fp;
        fp.open ( policy_vector_path.c_str() );
        IndexVectorPtr policy_vec ( new IndexVector() );

        fp >> ( *policy_vec );
        
        policy_ptr_ = boost::shared_ptr<MDPPolicy> ( new MDPEpsilonGreedyPolicyVector ( policy_vec,
                                                                                        number_of_states_,
                                                                                        number_of_actions_,
                                                                                        epsilon_type,
                                                                                        policy_vector_path ) );
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
}



void
ControllerMDP::
loadRewardMatrix ( const string& reward_matrix_path )
{
    try
    {
        if ( R_ptr_ != 0 )
            ROS_WARN_STREAM ( "The reward model for this MDP had already been loaded! Overwriting." );

        if ( loader_ != 0 )
        {
            ROS_WARN_STREAM ( "Trying to load a reward model when it has already been parsed from the problem file." );
            ROS_WARN_STREAM ( "This reward model will be used instead, but it may be inconsistent with the policy." );
        }
        
        ifstream fp;
        fp.open ( reward_matrix_path.c_str() );
        
        if ( strcmp ( reward_type_.c_str(), "vector" ) == 0 )
        {
            IndexVectorPtr reward_vector ( new IndexVector() );

            fp >> ( *reward_vector );

            R_ptr_ = boost::shared_ptr<RewardModel> ( new RewardVector ( reward_vector ) );
        }
        else
        {
            if ( strcmp ( reward_type_.c_str(), "matrix" ) == 0 )
            {
                MatrixPtr reward_matrix ( new Matrix() );

                fp >> ( *reward_matrix );

                R_ptr_ = boost::shared_ptr<RewardModel> ( new RewardMatrix ( reward_matrix ) );
            }
            else
            {
                ROS_ERROR_STREAM ( "The specified reward type is neither \"matrix\" nor \"vector\"." );
                abort ();
            }
        }
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
}



void
ControllerMDP::
act ( const uint32_t state )
{
    if ( getStatus() == STOPPED )
    {
        return;
    }

    if ( policy_ptr_ == 0 )
    {
        ROS_WARN_STREAM ( "No policy has been specified for this MDP! Idling." );
        return;
    }

    uint32_t action;

    if ( eps_greedy_ )
        action = policy_ptr_ -> getAction ( state );
    else
    {
        try
        {
            action = ( *policy_ptr_ ) [state];
        }
        catch ( exception& e )
        {
            ROS_ERROR_STREAM ( e.what() );
            abort();
        }
    }

    publishAction ( action );
    publishReward ( state, action );

    incrementDecisionEpisode();
    
    if ( eps_greedy_ )
        policy_ptr_ -> setCurrDecisionEp( getDecisionEpisode () );
}



void
ControllerMDP::
publishAction ( uint32_t a )
{
    ActionSymbol aInfo;
    aInfo.action_symbol = a;
    aInfo.decision_episode = getDecisionEpisode();
    action_pub_.publish ( aInfo );
    
    action_ = a;
    
    // Update the policy's current decision episode value
    if ( eps_greedy_ )
        ( *policy_ptr_ ).setCurrDecisionEp ( aInfo.decision_episode );
}



void
ControllerMDP::
publishReward ( uint32_t s, uint32_t a )
{
    if ( R_ptr_ == 0 )
    {
        return;  ///no reward info available.
    }

    std_msgs::Float32 reward;
    
    if ( strcmp ( reward_type_.c_str(), "matrix" ) == 0 )
        reward.data = R_ptr_->getReward ( s, a );
    else
    {
        if ( strcmp ( reward_type_.c_str(), "vector" ) == 0 )
            reward.data = R_ptr_->getReward ( s, 0 );
        else
        {
            ROS_ERROR_STREAM ( "The specified reward type is neither \"matrix\" nor \"vector\"." );
            abort ();
        }
    }
            
        
    reward_pub_.publish ( reward );
    
    reward_ = reward.data;
}



size_t
ControllerMDP::
getNumberOfActions ()
{
    return number_of_actions_;
}



size_t
ControllerMDP::
getNumberOfStates ()
{
    return number_of_states_;
}



boost::shared_ptr<MDPPolicy>
ControllerMDP::
getPolicy ()
{
    return policy_ptr_;
}



uint32_t
ControllerMDP::
getAction ()
{
    return action_;
}



float
ControllerMDP::
getReward ()
{
    return reward_;
}



uint32_t
ControllerMDP::
getLastState ()
{
    return last_state_;
}
