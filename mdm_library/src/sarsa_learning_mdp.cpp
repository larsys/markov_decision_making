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


using namespace std;
using namespace mdm_library;


#ifdef HAVE_MADP



SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   float alpha,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const string& problem_file_path,
                   const string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha, epsilon, policy_update_frequency,
                        controller_type, policy_file_path, problem_file_path, initial_status )
{
}



SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const string& problem_file_path,
                   const string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha_type, epsilon, policy_update_frequency,
                        controller_type, policy_file_path, problem_file_path, initial_status )
{
}


SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   float alpha,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const string& problem_file_path,
                   const string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha, epsilon_type, policy_update_frequency,
                        controller_type, policy_file_path, problem_file_path, initial_status )
{
}


SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const string& problem_file_path,
                   const string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha_type, epsilon_type, policy_update_frequency,
                        controller_type, policy_file_path, problem_file_path, initial_status )
{
}



#endif


SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   float alpha,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha, epsilon, policy_update_frequency,
                        controller_type, policy_file_path, initial_status )
{
}



SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha_type, epsilon, policy_update_frequency,
                        controller_type, policy_file_path, initial_status )
{
}



SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   float alpha,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha, epsilon_type, policy_update_frequency,
                        controller_type, policy_file_path, initial_status )
{
}



SarsaLearningMDP::
SarsaLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( gamma, alpha_type, epsilon_type, policy_update_frequency,
                        controller_type, policy_file_path, initial_status )
{
}



void
SarsaLearningMDP::
updateQValues ()
{
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
}
