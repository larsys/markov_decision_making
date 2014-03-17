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


using namespace std;
using namespace mdm_library;

#ifdef HAVE_MADP

LearningLayerBase::
LearningLayerBase ( float gamma,
                    float alpha,
                    float epsilon,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const string& problem_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_ ( alpha ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}



LearningLayerBase::
LearningLayerBase ( float gamma,
                    ALPHA_TYPE alpha_type,
                    float epsilon,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const string& problem_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_type_ ( alpha_type ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}



LearningLayerBase::
LearningLayerBase ( float gamma,
                    float alpha,
                    EPSILON_TYPE epsilon_type,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const string& problem_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_ ( alpha ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon_type, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon_type, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}



LearningLayerBase::
LearningLayerBase ( float gamma,
                    ALPHA_TYPE alpha_type,
                    EPSILON_TYPE epsilon_type,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const string& problem_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_type_ ( alpha_type ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon_type, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon_type, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}

#endif

LearningLayerBase::
LearningLayerBase ( float gamma,
                    float alpha,
                    float epsilon,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_ ( alpha ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}



LearningLayerBase::
LearningLayerBase ( float gamma,
                    ALPHA_TYPE alpha_type,
                    float epsilon,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_type_ ( alpha_type ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}



LearningLayerBase::
LearningLayerBase ( float gamma,
                    float alpha,
                    EPSILON_TYPE epsilon_type,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_ ( alpha ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon_type, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon_type, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    initializeQValues ();
}



LearningLayerBase::
LearningLayerBase ( float gamma,
                    ALPHA_TYPE alpha_type,
                    EPSILON_TYPE epsilon_type,
                    uint32_t policy_update_frequency,
                    CONTROLLER_TYPE controller_type,
                    const string& policy_file_path,
                    const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    gamma_ ( gamma ),
    alpha_type_ ( alpha_type ),
    policy_update_frequency_ ( policy_update_frequency ),
    curr_decision_ep_ ( 0 ),
    state_sub_ ( nh_.subscribe ( "state", 1, &LearningLayerBase::stateSymbolCallback, this ) )
{
    if ( controller_type == EVENT )
        ControllerEventMDP* controller_ = new ControllerEventMDP ( policy_file_path, epsilon_type, initial_status );
    else
    {
        if ( controller_type == TIMED )
            ControllerTimedMDP* controller_ = new ControllerTimedMDP ( policy_file_path, epsilon_type, initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type."
                        " Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
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
