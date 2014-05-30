/**\file controller_event_mdp.h
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

#ifndef _CONTROLLER_EVENT_MDP_H_
#define _CONTROLLER_EVENT_MDP_H_

#include <string>

#include <mdm_library/controller_mdp.h>



namespace mdm_library
{
/**
 * ControllerEventMDP implements an asynchronous MDP controller, driven by changes in the
 * system state.
 * @TODO: Accepting a policy file, containing a string of separated integer values.
 */
class ControllerEventMDP : public ControllerMDP
{
public:
#ifdef HAVE_MADP
    /** Preferred constructor. When using this form, the reward function of the MDP model
     * is known to the controller, and so reward can be logged in real-time. Furthermore,
     * the metadata of the model is parsed automatically and passed to the Action Layer.
     * @param problem_file A file defining the MDP, in any MADP-compatible format.
     * @param q_value_function_file A file defining the Q-value function of this MDP, as a
     * whitespace-separated |S|x|A| matrix of floating point numbers.
     * If you have an explicit policy instead, convert it to a matrix where the only non-zero entries
     * exist in the specified (s,a) pairs.
     * @param initial_status (optional) The initial status of this controller.
     */
    ControllerEventMDP ( const std::string& policy_file_path,
                         const std::string& problem_file_path,
                         const CONTROLLER_STATUS initial_status = STARTED );
    
    ControllerEventMDP ( const std::string& policy_file_path,
                         const std::string& problem_file_path,
                         EPSILON_TYPE epsilon_type,
                         const CONTROLLER_STATUS initial_status = STARTED );
#endif
    /** Alternative constructor. This form bypasses the problem file parsing step, so it is faster, and
     * can be used if a MADP-compatible problem file isn't available. However, you can't log the reward
     * at run-time, and the action metadata will not contain any action names.
     * @param nr_actions The number of actions of this MDP.
     * @param nr_states The number of states of this MDP.
     * @param q_value_function_file A file defining the Q-value function of this MDP, as a
     * whitespace-separated |S|x|A| matrix of floating point numbers.
     * If you have an explicit policy instead, convert it to a matrix where the only non-zero entries
     * exist in the specified (s,a) pairs.
     * @param initial_status (optional) The initial status of this controller.
     */
    ControllerEventMDP ( const std::string& policy_file_path,
                         const CONTROLLER_STATUS initial_status = STARTED );
    
    ControllerEventMDP ( const std::string& policy_file_path,
                         EPSILON_TYPE epsilon_type,
                         uint32_t num_states,
                         uint32_t num_actions,
                         const CONTROLLER_STATUS initial_status = STARTED );
    
    /** The callback to incoming state information. This will trigger a new decision step. */
    void stateCallback ( const WorldSymbolConstPtr& msg );
    
private:
};
}

#endif
