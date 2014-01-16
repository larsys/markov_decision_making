/**\file action_layer.h
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

#ifndef _ACTION_LAYER_H_
#define _ACTION_LAYER_H_

#include <string>
#include <vector>
#include <map>

#include <boost/function.hpp>

#include <ros/ros.h>
#include <ros/callback_queue_interface.h>

#include <markov_decision_making/ActionSymbol.h>
#include <markov_decision_making/FactoredSymbolMetadata.h>



namespace markov_decision_making
{
/**
 * The ActionLayer class provides the tools to abstract actions as generic functions.
 * Each action that is received by an ActionLayer triggers an agent-specific "action callback".
 * ActionLayers subscribe to the "action" and "action_metadata" topics in their node's namespace.
 * The former contains the actual actions that should be performed by the agent, as integer values,
 * while the latter specifies the number of actions of each agent and possibly their names.
 * In most cases, a Control Layer running on the same namespace will be publishing to these input topics
 * transparently.
 */
class ActionLayer
{
public:
    /** Action callbacks do not return any values or accept any arguments (they should be bound, if needed).
     * @sa action_callbacks_
     */
    typedef std::vector<boost::function<void () > > ActionCallbacksVector;
    /** Type for the set of action names.
     * @sa local_action_names_
     */
    typedef std::vector<std::string> ActionNamesVector;

    typedef std::map<std::string, size_t> ActionIDMap;

    typedef std::vector<std::size_t> ActionIDVector;

    /**
     * Type for the set containing the number of actions of each agent.
     * This is necessary to convert joint action indexes into local action indexes.
     * @sa action_sizes_
     */
    typedef std::vector<size_t> ActionSizesVector;

    /**
     * The default Action Layer constructor, which doesn't take any arguments.
     * It gets the action metadata from an associated Control Layer, through action_metadata_sub_.
     */
    ActionLayer ();
    /**
     * An optional Action Layer constructor that takes action metadata directly.
     * @param action_metadata The action metadata (number of actions and respective names)
     * for this MDM agent. See msg/ActionMetadata.msg for details.
     */
    ActionLayer ( const FactoredSymbolMetadataConstPtr& action_metadata );

    /**
     * This constructor allows the action callback to be added to a custom queue.
     * This is useful for multi-threaded nodes or those with custom spinning strategies.
     */
    ActionLayer ( ros::CallbackQueueInterface* queue );

    /** Callback for actions coming from the Control Layer. */
    void actionSymbolCallback ( const markov_decision_making::ActionSymbolConstPtr& msg );

    /** Callback for action metadata coming from the Control Layer. */
    void actionMetadataCallback ( const markov_decision_making::FactoredSymbolMetadataConstPtr& msg );

    /**
     * Add an action to this Action Layer with the specified callback.
     * Actions must be added before spinning the thread containing the declaration of this Action Layer.
     * Actions are identified by the order in which they are added to the Action Layer.
     * @param callback The function to be performed when this action is received in through action_sub_.
     */
    void addAction ( boost::function<void () > callback );

    void addAction ( boost::function<void () > callback, const std::string & action_name );

    uint32_t jointToIndividualAction ( const uint32_t joint_action );

private:
    ros::NodeHandle nh_;

    /** The subscriber to the "action" topic in the local (public) namespace,
     * in which the action information will be received.
     */
    ros::Subscriber action_sub_;
    /** The subscriber to the "action_metadata" topic in the local (public) namespace,
     * in which the action metadata will be received.
     */
    ros::Subscriber action_metadata_sub_;

    /** The set of action callbacks */
    ActionCallbacksVector action_callbacks_;
    /** The set of local action names. */
    ActionNamesVector local_action_names_;
    ActionIDMap named_action_ids_;
    ActionIDVector action_ids_;

    /** The ordered set containing the number of actions of each agent contemplated by the Control Layer. */
    ActionSizesVector action_sizes_;

    uint32_t cached_sizes_prod_;
    /**
     * A failsafe for situations in which an action is received -before- the metadata is known.
     * That action is saved and executed once the metadata is received.
     */
    ActionSymbolConstPtr pending_action_;

    /** The index of this agent with respect to the Decision-Theoretic model represented by the Control Layer. */
    int mdm_agent_index_;

    bool using_named_actions_;
};
}

#endif
