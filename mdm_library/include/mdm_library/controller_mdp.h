/**\file controller_mdp.h
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

#ifndef _CONTROLLER_MDP_H_
#define _CONTROLLER_MDP_H_

#include <string>

#include <ros/ros.h>

#include <mdm_library/common_defs.h>
#include <mdm_library/control_layer_base.h>
#include <mdm_library/decpomdp_loader.h>
#include <mdm_library/mdp_policy.h>
#include <mdm_library/reward_model.h>
#include <mdm_library/WorldSymbol.h>



namespace mdm_library
{
/**
 * ControllerMDP is a base class for Control Layers based on MDPs.
 * This is an abstract class. You must implement the state callback behavior
 * depending on the particular requirements of your system.
 */
class ControllerMDP: public ControlLayerBase
{
public:
#ifdef HAVE_MADP
    /** Preferred constructor. When using this form, the reward function of the MDP model
     * is known to the controller, and so reward can be logged in real-time. Furthermore,
     * the metadata of the model is parsed automatically and passed to the Action Layer.
     * @param problem_file A file defining the MDP, in any MADP-compatible format.
     * @param q_value_function_file The path to a file defining the Q-value function of this MDP, as a
     * whitespace-separated |S|x|A| matrix of floating point numbers.
     * If you have an explicit policy instead, convert it to a matrix where the only non-zero entries
     * exist in the specified (s,a) pairs.
     * @param initial_status (optional) The initial status of this controller.
     */
    ControllerMDP ( const std::string& policy_file_path,
                    const std::string& problem_file_path,
                    const CONTROLLER_STATUS initial_status = STARTED );
#endif

    ControllerMDP ( const std::string& policy_file_path,
                    const CONTROLLER_STATUS initial_status = STARTED );

    void loadPolicyVector ( const std::string& policy_vector_path );

    void loadRewardMatrix ( const std::string& reward_matrix_path );

    /**
     * A step of the decision making process. Selects the best action at the current state,
     * and publishes it to the "action" topic on the namespace of the respective node.
     * Also publishes the reward for this state-action pair, if it is available, to the "reward" topic.
     */
    void act ( const uint32_t state );

    /**
     * Pure virtual callback to state information. Derived classes can define an execution strategy
     * (synchronous or event-driven) by implementing this method accordingly.
     */
    virtual void stateCallback ( const WorldSymbolConstPtr& msg ) = 0;

    /** Returns the number of actions of this MDP. */
    size_t getNumberOfActions ();
    /** Returns the number of states of this MDP. */
    size_t getNumberOfStates ();

protected:
    /** Publishes an action. */
    void publishAction ( uint32_t a );
    /** Publishes the reward of a state-action pair. */
    void publishReward ( uint32_t s, uint32_t a );

    boost::shared_ptr<RewardModel> R_ptr_;
    boost::shared_ptr<MDPPolicy> policy_ptr_;
    /** The parser for the MDP problem file. */
    boost::shared_ptr<DecPOMDPLoader> loader_;
    /** The number of states of this MDP. */
    size_t number_of_states_;
    /** The number of actions of this MDP. */
    size_t number_of_actions_;

    /** Subscriber to the "state" topic, where the state information will be published by a State Layer.*/
    ros::Subscriber state_sub_;
    /** Publisher to the "action" topic, where the action information will be passed on to an Action Layer.*/
    ros::Publisher action_pub_;
    /** Publisher to the "reward" topic, where reward information can be acessed for reinforcement learning or logging purposes.*/
    ros::Publisher reward_pub_;
};
}

#endif
