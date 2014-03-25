/**\file online_learning_mdp.cpp
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


#include <mdm_library/learning_layer_base.h>
#include <mdm_library/Policy.h>


using namespace std;
using namespace mdm_library;

#ifdef HAVE_MADP

LearningLayerBase::
LearningLayerBase ( float gamma,
                    ALPHA_TYPE alpha_type,
                    EPSILON_TYPE epsilon_type,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const string& problem_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_type_ ( alpha_type ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) ),
    policy_pub_ ( nh_.advertise<Policy> ( "policy", 0, true ) )
{
    // Gather the policy update frequency value from the parameters
    uint32_t policy_update_frequency;
    if ( private_nh_.getParam ( "policy_update_frequency", policy_update_frequency ) )
    {
        policy_update_frequency_ = ( uint32_t ) policy_update_frequency;
    }
    else
    {
        policy_update_frequency_ = MDM_DEFAULT_POLICY_UPDATE_FREQ;
    }
    
    // Gather the gamma value from the parameters
    double gamma;
    if ( private_nh_.getParam ( "gamma", gamma ) )
    {
        if ( gamma < 0 || gamma > 1 )
        {
            ROS_FATAL ( "Invalid provided gamma value. The gamma value must be between 0 and 1." );
            ros::shutdown();
        }
        else
            gamma_ = ( float ) gamma;
    }
    else
    {
        gamma_ = MDM_DEFAULT_GAMMA;
    }
    
    // Gather the alpha value from the parameters if alpha type is set as constant
    double alpha;
    if ( alpha_type_ == ALPHA_CONSTANT )
    {
        if ( private_nh_.getParam ( "alpha", alpha ) )
        {
            if ( alpha < 0 || alpha > 1 )
            {
                ROS_FATAL ( "Invalid provided alpha value. The alpha value must be between 0 and 1." );
                ros::shutdown();
            }
            else
                alpha_ = ( float ) alpha;
        }
        else
        {
            alpha_ = MDM_DEFAULT_ALPHA;
        }
    }
    
    // Create the controller
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon_type, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon_type, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}

#endif

LearningLayerBase::
LearningLayerBase ( ALPHA_TYPE alpha_type,
                    EPSILON_TYPE epsilon_type,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    alpha_type_ ( alpha_type ),
    curr_decision_ep_ ( 0 ),
    private_nh_ ( "~" ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) ),
    policy_pub_ ( nh_.advertise<Policy> ( "policy", 0, true ) )
{
    // Gather the policy update frequency value from the parameters
    int policy_update_frequency;
    
    if ( private_nh_.getParam ( "policy_update_frequency", policy_update_frequency ) )
        policy_update_frequency_ = ( uint32_t ) policy_update_frequency;
    else
        policy_update_frequency_ = MDM_DEFAULT_POLICY_UPDATE_FREQ;
    
    
    // Gather the gamma value from the parameters
    double gamma;
    
    if ( private_nh_.getParam ( "gamma", gamma ) )
    {
        if ( gamma < 0 || gamma > 1 )
        {
            ROS_FATAL ( "Invalid provided gamma value. The gamma value must be between 0 and 1." );
            ros::shutdown();
        }
        else
            gamma_ = ( float ) gamma;
    }
    else
        gamma_ = MDM_DEFAULT_GAMMA;
    
    
    // Gather the alpha value from the parameters if alpha type is set as constant
    double alpha;
    
    if ( alpha_type_ == ALPHA_CONSTANT )
    {
        if ( private_nh_.getParam ( "alpha", alpha ) )
        {
            if ( alpha < 0 || alpha > 1 )
            {
                ROS_FATAL ( "Invalid provided alpha value. The alpha value must be between 0 and 1." );
                ros::shutdown();
            }
            else
                alpha_ = ( float ) alpha;
        }
        else
            alpha_ = MDM_DEFAULT_ALPHA;
    }
    
    
    // Initialize the Q values table
    initializeQValues ();
}



void
LearningLayerBase::
initializeQValues ()
{
    size_t num_states = ( *controller_ ).getNumberOfStates ();
    size_t num_actions = ( *controller_ ).getNumberOfActions ();
    
    Matrix q_values_ ( num_states, num_actions );
    
    // Initialize the Q values as 0
    for ( unsigned i = 0; i < q_values_.size1(); i++ )
        for ( unsigned j = 0; j < q_values_.size2(); j++ )
            q_values_ ( i, j ) = 0;
}
