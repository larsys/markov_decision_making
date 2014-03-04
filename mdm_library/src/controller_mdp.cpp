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
    reward_pub_ ( nh_.advertise<std_msgs::Float32> ( "reward", 0, true ) )
{
    ///This constructor will not publish problem metadata or reward (you will have to do it manually).
    loadPolicyVector ( policy_file_path );
}



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
        MatrixPtr reward_matrix ( new Matrix() );

        fp >> ( *reward_matrix );

        R_ptr_ = boost::shared_ptr<RewardModel> ( new RewardMatrix ( reward_matrix ) );
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
    }

    uint32_t action;

    try
    {
        action = ( *policy_ptr_ ) [state];
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }

    publishAction ( action );
    publishReward ( state, action );

    incrementDecisionEpisode();
}



void
ControllerMDP::
publishAction ( uint32_t a )
{
    ActionSymbol aInfo;
    aInfo.action_symbol = a;
    aInfo.decision_episode = getDecisionEpisode();
    action_pub_.publish ( aInfo );
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
    reward.data = R_ptr_->getReward ( s, a );
    reward_pub_.publish ( reward );
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



void
ControllerMDP::
updatePolicy ( Matrix q_values )
{
    uint32_t best_action;
    
    for ( uint32_t state = 0; state < number_of_states_; state++ )
    {
        best_action = argMaxA ( q_values, state );
        
        try
        {
            policy_ptr_ -> setAction ( state, best_action );
        }
        catch ( exception& e )
        {
            ROS_ERROR_STREAM ( e.what() );
            abort();
        }
    }
}



uint32_t
ControllerMDP::
argMaxA ( Matrix q_values, uint32_t state )
{
    // Initialize the current maximum as negative infinity
    double curr_max = std::numeric_limits<double>::infinity() * -1;
    uint32_t index;
    
    // Find the action that leads to the highest Q value
    for (unsigned j = 0; j < q_values.size2(); j++ )
    {
        if ( q_values ( state, j ) > curr_max )
        {
            curr_max = q_values ( state, j );
            index = j;
        }
    }
    
    return index;
}
