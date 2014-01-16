/**\file controller_pomdp.h
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

#ifndef _CONTROLLER_POMDP_H_
#define _CONTROLLER_POMDP_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <madp/QFunctionJointBeliefInterface.h>
#include <madp/JointBeliefInterface.h>
#include <madp/FSDist_COF.h>
#include <markov_decision_making/control_layer_base.h>
#include <markov_decision_making/WorldSymbol.h>
#include <markov_decision_making/BeliefStateInfo.h>
#include <markov_decision_making/FactoredDistribution.h>
#include <markov_decision_making/decpomdp_loader.h>



namespace markov_decision_making
{
/**
 * ControllerPOMDP is an abstract class that provides the basic functionality for Factored Discrete POMDPs
 * where the agent policy is implicitly described by a value function (or Q-value function).
 */
class ControllerPOMDP: public ControlLayerBase
{
public:
    /**
     * Constructor.
     * @param problem_file the POMDP problem file in any MADP-compatible format.
     * @param initial_status whether this controller should be auto-started upon construction or not.
     */
    ControllerPOMDP ( const std::string& problem_file,
                      const CONTROLLER_STATUS initial_status = STARTED );

    /**
     * Returns the number of actions in the POMDP, without exposing the DecPOMDPLoader.
     */
    size_t getNumberOfActions ();

    /**
     * Returns the number of states in the POMDP, without exposing the DecPOMDPLoader.
     */
    size_t getNumberOfStates ();

    /**
     * Returns the number of observations in the POMDP, without exposing the DecPOMDPLoader.
     */
    size_t getNumberOfObservations ();

protected:
    /**
     * Main POMDP control step. Selects an action given an observation.
     * The belief state will be updated according to the current observation and last action.
     * The action with the highest value at the updated belief will be selected and published
     * in the "action" topic in the current namespace.
     * @param observation the observation symbol for this decision step.
     */
    void act ( const uint32_t observation );

    /**
     * The callback for observations coming in through the "observation" topic in the current namespace.
     * Each particular POMDP execution strategy (synchronous/asynchronous) will have to implement this callback.
     */
    virtual void observationCallback ( const WorldSymbolConstPtr& msg ) = 0;

    /**
     * The callback for external belief state estimates.
     * Keep in mind that this information *will* override the POMDP's internal belief state.
     * This can be used in scenarios where a belief state is naturally being estimated elsewhere,
     * and the controller is only required to select an appropriate response.
     */
    virtual void extBeliefCallback ( const BeliefStateInfoConstPtr& msg );

    /**
     * The callback for initial state distribution estimates.
     * This information *will not* override the POMDP's internal belief state.
     * It will set the ISD for this problem, which will be loaded after a controller reset,
     * which will happen after the horizon runs out, or the controller is stopped/started by
     * a higher level module.
     */
    virtual void isdCallback ( const FactoredDistributionConstPtr& msg );

    /** Publishes an action symbol to the "action" topic. */
    void publishAction ( uint32_t a );

    /**
     * Publishes the -immediate- expected reward for taking action "a" in the current
     * belief state, to the "reward" topic.
     */
    void publishExpectedReward ( uint32_t a );

    /** Publishes the belief state after an update, useful for debugging purposes.*/
    void publishCurrentBelief ();

    /** Normalizes a given JointBeliefInterface so that it sums to 1.*/
    void normalizeBelief ( boost::shared_ptr<JointBeliefInterface> b );

    /** Pointer to a "loader" object which will actually parse the problem file.*/
    boost::shared_ptr<DecPOMDPLoader> loader_;
    /** Pointer to the internal belief state.*/
    boost::shared_ptr<JointBeliefInterface> belief_;
    /** Pointer to this problem's Q-Value function.*/
    boost::shared_ptr<QFunctionJointBeliefInterface> Q_;
    /** Previous action. Needed for belief updates.*/
    uint32_t prev_action_;
    /** Initial State Distribution.*/
    boost::shared_ptr<FSDist_COF> ISD_;

    /** Subscriber to the "observation" topic.*/
    ros::Subscriber observation_sub_;
    /** Subscriber to the "ext_belief_estimate" topic.*/
    ros::Subscriber ext_belief_estimate_sub_;
    /** Subscriber to the "initial_state_distribution" topic.*/
    ros::Subscriber isd_sub_;
    /** Publisher to the "current_belief" topic.*/
    ros::Publisher current_belief_pub_;
    /** Publisher to the "action" topic.*/
    ros::Publisher action_pub_;
    /** Publisher to the "reward" topic.*/
    ros::Publisher exp_reward_pub_;
};
}

#endif
