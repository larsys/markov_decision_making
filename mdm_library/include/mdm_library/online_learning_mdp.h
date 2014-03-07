/**\file online_learning_mdp.h
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

#ifndef _ONLINE_LEARNING_MDP_H_
#define _ONLINE_LEARNING_MDP_H_


#include <mdm_library/control_layer_base.h>
#include <mdm_library/controller_event_mdp.h>
#include <mdm_library/common_defs.h>
#include <mdm_library/learning_defs.h>

#include <mdm_library/ActionSymbol.h>
#include <mdm_library/WorldSymbol.h>
#include <std_msgs/Float32.h>


namespace mdm_library
{
/**
 * OnlineLearningMDP is the base class for online learning MDPs.
 */
class OnlineLearningMDP
{
public:
#ifdef HAVE_MADP
    OnlineLearningMDP ( float gamma,
                        ALPHA_TYPE alpha_type,
                        float alpha,
                        EPSILON_TYPE epsilon_type,
                        float epsilon,
                        uint32_t policy_update_frequency,
                        const std::string& policy_file_path,
                        const std::string& problem_file_path,
                        const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
#endif

    OnlineLearningMDP ( float gamma,
                        ALPHA_TYPE alpha_type,
                        float alpha,
                        EPSILON_TYPE epsilon_type,
                        float epsilon,
                        uint32_t policy_update_frequency,
                        const std::string& policy_file_path,
                        const ControlLayerBase::CONTROLLER_STATUS initial_status = ControlLayerBase::STARTED );
    
protected:
    /** MDP Controller */
    ControllerEventMDP controller_;
    
    /** Q-table */
    Matrix q_values_;
    
    /** The gamma parameter TODO*/
    float gamma_;
    
    /** The alpha type */
    ALPHA_TYPE alpha_type_;
    
    /** The alpha parameter TODO*/
    float alpha_;
    
    /** The epsilon type */
    EPSILON_TYPE epsilon_type_;
    
    /** The epsilon parameter TODO - fazer funcao dependente de t */
    float epsilon_;
    
    /** Policy update frequency */
    uint32_t policy_update_frequency_;
    
    /** The current decision episode */
    uint32_t curr_decision_ep_;
    
    /** Number of states */
    size_t num_states_;
    
    /** Number of actions */
    size_t num_actions_;
    
    /** Pure virtual function for updating the Q values. To be implemented in each specific method */
    virtual void updateQValues () = 0;
    
    /** Pure virtual callback for actions coming from the State Layer, to be implemented in each specific method. */
    virtual void stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg ) = 0;
    
    /** Pure virtual callback for actions coming from the Control Layer, to be implemented in each specific method. */
    virtual void actionSymbolCallback ( const mdm_library::ActionSymbolConstPtr& msg ) = 0;
    
    /** Pure virtual callback for actions coming from the Control Layer, to be implemented in each specific method. */
    virtual void rewardSymbolCallback ( const std_msgs::Float32& msg ) = 0;
    
private:
    /** ROS Nodehandle for the learning layer. */
    ros::NodeHandle nh_;
    
    /** The subscriber to the "state" topic in the local (public) namespace,
     * in which the state information will be received.
     */
    ros::Subscriber state_sub_;
    
    /** The subscriber to the "action" topic in the local (public) namespace,
     * in which the action information will be received.
     */
    ros::Subscriber action_sub_;
    
    /** The subscriber to the "reward" topic in the local (public) namespace,
     * in which the reward information will be received.
     */
    ros::Subscriber reward_sub_;
    
    /**
     * Function to initialize the Q-values table.
     */
    void initializeQValues ();
};
}

#endif
