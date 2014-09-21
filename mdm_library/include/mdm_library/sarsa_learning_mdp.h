/**\file sarsa_learning_mdp.h
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

#ifndef _SARSA_LEARNING_MDP_H_
#define _SARSA_LEARNING_MDP_H_


#include <mdm_library/learning_layer_base.h>


namespace mdm_library
{
/**
 * SarsaLearningMDP is the class for the Sarsa learning method.
 */
class SarsaLearningMDP : public LearningLayerBase
{
public:
#ifdef HAVE_MADP
    SarsaLearningMDP ( ALPHA_TYPE alpha_type,
                       EPSILON_TYPE epsilon_type,
                       CONTROLLER_TYPE controller_type,
                       uint32_t num_states,
                       uint32_t num_actions,
                       const std::string& policy_file_path,
                       const std::string& problem_file_path,
                       const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
#endif

    SarsaLearningMDP ( ALPHA_TYPE alpha_type,
                       EPSILON_TYPE epsilon_type,
                       CONTROLLER_TYPE controller_type,
                       uint32_t num_states,
                       uint32_t num_actions,
                       const std::string& policy_file_path,
                       const std::string& reward_file_path,
                       const std::string& q_values_path,
                       const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
    SarsaLearningMDP ( ALPHA_TYPE alpha_type,
                    EPSILON_TYPE epsilon_type,
                    CONTROLLER_TYPE controller_type,
                    uint32_t num_states,
                    uint32_t num_actions,
                    const std::string& policy_file_path,
                    const std::string& reward_file_path,
                    const std::string& q_values_path,
                    const std::string& eligibility_traces_path,
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
    
    /** Next action backup */
    uint32_t next_action_;
    
    /** Initialize the Q Values Table to zeroes */
    void initializeQValues ();
    
    /** Implementation of the pure virtual function updateQValues from OnlineLearningMDP */
    void updateQValues ();
    
    /** Implementation of the pure virtual function updatePolicy from LearningLayerBase */
    void updatePolicy ();
    
    /** Implementation of the pure virtual function publishPolicy from LearningLayerBase */
    void publishPolicy ();
    
    /** Implementation of the pure virtual function stateSymbolCallback from LearningLayerBase */
    void stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg );
    
    /** Implementation of the pure virtual function republish_callback from LearningLayerBase */
    bool republish_callback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response );
    
    /** Implementation of the pure virtual function to reason over new decision episodes */
    void newDecisionEpisode ( uint32_t state );
};
}


#endif
