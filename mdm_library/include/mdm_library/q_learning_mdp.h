/**\file q_learning_mdp.h
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

#ifndef _Q_LEARNING_MDP_H_
#define _Q_LEARNING_MDP_H_


#include <mdm_library/learning_layer_base.h>


namespace mdm_library
{
/**
 * QLearningMDP is the class for the Q-Learning method.
 */
class QLearningMDP : public LearningLayerBase
{
public:
#ifdef HAVE_MADP
    QLearningMDP ( float gamma,
                   float alpha,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const std::string& problem_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    QLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   float alpha,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const std::string& problem_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    QLearningMDP ( float gamma,
                   float alpha,
                   EPSILON_TYPE epsilon_type,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const std::string& problem_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    QLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const std::string& problem_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
#endif

    QLearningMDP ( float gamma,
                   float alpha,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    QLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   float epsilon,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    QLearningMDP ( float gamma,
                   float alpha,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    QLearningMDP ( float gamma,
                   ALPHA_TYPE alpha_type,
                   EPSILON_TYPE epsilon_type,
                   uint32_t policy_update_frequency,
                   CONTROLLER_TYPE controller_type,
                   const std::string& policy_file_path,
                   const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
private:
    /** Current state backup */
    uint32_t state_;
    
    /** Current action backup */
    uint32_t action_;
    
    /** Current reward backup */
    float reward_;
    
    /** Next state backup */
    uint32_t next_state_;
    
    /** Implementation of the pure virtual function updateQValues from LearningLayerBase */
    void updateQValues ();
    
    /** Implementation of the pure virtual function updatePolicy from LearningLayerBase */
    void updatePolicy ();
    
    /** Implementation of the pure virtual function stateSymbolCallback from LearningLayerBase */
    void stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg );
    
    /** Implementation of the pure virtual function actionSymbolCallback from LearningLayerBase */
    void actionSymbolCallback ( const mdm_library::ActionSymbolConstPtr& msg );
    
    /** Implementation of the pure virtual function rewardSymbolCallback from LearningLayerBase */
    void rewardSymbolCallback ( const std_msgs::Float32& msg );
    
    /** Function to get the maximum of Q(s, a) over a */
    float maxOverA ();
};
}

#endif
