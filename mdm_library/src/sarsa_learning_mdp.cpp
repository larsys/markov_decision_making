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
#include <mdm_library/action_layer.h>


using namespace std;
using namespace mdm_library;


#ifdef HAVE_MADP



SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   float alpha,
                   EPSILON_TYPE epsilon_type,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   const string& problem_file_path,
                   const string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    OnlineLearningMDP ( gamma, alpha_type, alpha, epsilon_type, epsilon, policy_update_frequency,
                        policy_file_path, problem_file_path, initial_status )
{
}



#endif


SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   float alpha,
                   EPSILON_TYPE epsilon_type,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    OnlineLearningMDP ( gamma, alpha_type, alpha, epsilon_type, epsilon, policy_update_frequency,
                        policy_file_path, initial_status )
{
}


void
SarsaLearningMDP::
updateQValues ()
{
    // TODO actualizacao da politica
    q_values_ ( state_, action_ ) = q_values_ ( state_, action_ ) + alpha_ * ( reward_ + gamma_ *
                                    q_values_ ( next_state_, next_action_ ) - q_values_ ( state_, action_ ) );
}


void
SarsaLearningMDP::
stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg )
{
    if ( curr_decision_ep_ == 0 )
        state_ = msg -> world_symbol;
    else
    {
        if ( curr_decision_ep_ == 1 )
            next_state_ = msg -> world_symbol;
        else
        {
            state_ = next_state_;
            next_state_ = msg -> world_symbol;
        }
    }
}


void
SarsaLearningMDP::
actionSymbolCallback ( const mdm_library::ActionSymbolConstPtr& msg )
{
    curr_decision_ep_ = msg -> decision_episode;
    
    if ( curr_decision_ep_ == 0 )
        action_ = msg -> action_symbol;
    else
    {
        if ( curr_decision_ep_ == 1 )
            next_action_ = msg -> action_symbol;
        else
        {
            action_ = next_action_;
            next_action_ = msg -> action_symbol;
        }
    }
    
    // Update the Q values after each decision episode
    if ( curr_decision_ep_ >= 1 )
        updateQValues ();
    
    // TODO ver se isto e importante ver ou nao
//     if ( ActionLayer::action_sizes_.size() > 1 )
//         action_ = ActionLayer::jointToIndividualAction ( msg -> action_symbol );
//     else
//         action_ = msg -> action_symbol;
}


void
SarsaLearningMDP::
rewardSymbolCallback ( const std_msgs::Float32& msg )
{
    reward_ = msg.data;
}
