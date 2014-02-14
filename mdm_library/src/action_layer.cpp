/**\file action_layer.cpp
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


#include <mdm_library/common_defs.h>
#include <mdm_library/action_layer.h>


using namespace ros;
using namespace std;
using namespace mdm_library;



ActionLayer::
ActionLayer() :
    action_sub_ ( nh_.subscribe ( "action", 1, &ActionLayer::actionSymbolCallback, this ) ),
    action_metadata_sub_ ( nh_.subscribe ( "action_metadata", 1, &ActionLayer::actionMetadataCallback, this ) ),
    action_callbacks_ (),
    local_action_names_ (),
    action_sizes_ (),
    cached_sizes_prod_ ( 0 ),
    pending_action_ (),
    mdm_agent_index_ ( 0 ),
    using_named_actions_ ( false )
{
    NodeHandle private_nh ( "~" );
    if ( ! ( private_nh.getParam ( "mdm_agent_index", mdm_agent_index_ ) ) )
    {
        ROS_WARN ( "ActionLayer:: MDM Agent Index not specified. Proceeding as agent 0." );
    }
}



ActionLayer::
ActionLayer ( const FactoredSymbolMetadataConstPtr& action_metadata ) :
    action_sub_ ( nh_.subscribe ( "action", 1, &ActionLayer::actionSymbolCallback, this ) ),
    action_metadata_sub_ (),
    action_callbacks_ (),
    local_action_names_ (),
    action_sizes_ (),
    cached_sizes_prod_ ( 0 ),
    pending_action_ (),
    mdm_agent_index_ ( 0 ),
    using_named_actions_ ( false )
{
    NodeHandle private_nh ( "~" );
    if ( ! ( private_nh.getParam ( "mdm_agent_index", mdm_agent_index_ ) ) )
    {
        ROS_WARN ( "ActionLayer:: MDM Agent Index not specified. Proceeding as agent 0." );
    }
    actionMetadataCallback ( action_metadata );
}



ActionLayer::
ActionLayer ( CallbackQueueInterface* queue ) :
    action_sub_ (),
    action_metadata_sub_ (),
    action_callbacks_ (),
    local_action_names_ (),
    action_sizes_ (),
    cached_sizes_prod_ ( 0 ),
    pending_action_ (),
    mdm_agent_index_ ( 0 ),
    using_named_actions_ ( false )
{
    NodeHandle private_nh ( "~" );
    if ( ! ( private_nh.getParam ( "mdm_agent_index", mdm_agent_index_ ) ) )
    {
        ROS_WARN ( "ActionLayer:: MDM Agent Index not specified. Proceeding as agent 0." );
    }
    SubscribeOptions opts = SubscribeOptions::create<ActionSymbol>
                            ( "action", 1, boost::bind ( &ActionLayer::actionSymbolCallback, this,_1 ),VoidPtr(), queue );
    action_sub_ = nh_.subscribe ( opts );
}


void
ActionLayer::
addAction ( boost::function<void () > callback )
{
    addAction ( callback, "" );
}


void
ActionLayer::
addAction ( boost::function<void () > callback, const string& action_name )
{
    if ( ( action_name.empty() && using_named_actions_ )
            || ( !action_name.empty() && action_callbacks_.size() > 0 && !using_named_actions_ ) )
    {
        ROS_FATAL ( "ActionLayer:: Can't use both named and nameless actions." );
        ROS_FATAL ( "ActionLayer:: Please label every action (or none) when adding them to the Action Layer" );
        shutdown();
        return;
    }

    if ( !action_name.empty() )
    {
        using_named_actions_ = true;
        local_action_names_.push_back ( action_name );
    }

    if ( using_named_actions_ )
    {
        named_action_ids_[action_name] = action_callbacks_.size();
    }

    action_callbacks_.push_back ( callback );
}



void
ActionLayer::
actionSymbolCallback ( const ActionSymbolConstPtr& msg )
{
    try
    {
        if ( action_sizes_.empty() )
        {
            ROS_WARN ( "ActionLayer:: Action received, but no metadata is known yet." );
            ROS_WARN ( "ActionLayer:: Proceeding as a single, unverified agent." );
            action_sizes_.push_back ( action_callbacks_.size() );
            action_ids_.clear();
            for ( size_t i = 0; i < action_callbacks_.size(); i++ )
                action_ids_.push_back ( i );
            pending_action_ = boost::shared_ptr<const ActionSymbol> ( new ActionSymbol ( *msg ) );
        }

        uint32_t local_action;
        if ( action_sizes_.size() > 1 )
        {
            local_action = jointToIndividualAction ( msg->action_symbol );
        }
        else
        {
            local_action = msg->action_symbol;
        }
        if ( local_action_names_.empty() )
        {
            ROS_INFO_STREAM ( "ActionLayer:: Agent " << mdm_agent_index_ << " executing action " << local_action );
        }
        else
        {
            ROS_INFO_STREAM ( "ActionLayer:: Agent " << mdm_agent_index_ << " executing action " << local_action << " (" << local_action_names_[local_action] << ")" );
        }
        if ( action_ids_.size() > local_action && action_callbacks_.size() > action_ids_[local_action] )
        {
            size_t id = action_ids_[local_action];
            action_callbacks_[id] ();
        }
        else
        {
            ROS_FATAL_STREAM ( "ActionLayer:: Callback for action " << local_action << " is unknown" );
            shutdown();
        }
    }
    catch ( exception & e )
    {
        ROS_FATAL_STREAM ( e.what() );
        shutdown();
    }
}



void
ActionLayer::
actionMetadataCallback ( const FactoredSymbolMetadataConstPtr& msg )
{
    if ( action_ids_.size() < action_callbacks_.size() )
    {
        action_ids_.reserve ( action_callbacks_.size() );
    }

    uint32_t declared_values = action_callbacks_.size();

    if ( msg->factors.size() < ( size_t ) mdm_agent_index_ )
    {
        ROS_FATAL_STREAM ( "ActionLayer:: Action Metadata for agent " << mdm_agent_index_ << " is not present" );
        shutdown();
        return;
    }

    uint32_t expected_values = msg->factors[mdm_agent_index_].number_of_symbols;

    if ( declared_values != expected_values )
    {
        ROS_FATAL_STREAM ( "ActionLayer:: Number of declared actions for agent " << mdm_agent_index_ << " (" << declared_values
                           << ") is different than the number of expected actions (" << expected_values << ")" );
        shutdown();
        return;
    }

    action_sizes_.clear();
    for ( size_t i = 0; i < msg->factors.size(); i++ )
    {
        action_sizes_.push_back ( msg->factors[i].number_of_symbols );
        if ( ! using_named_actions_ && i == mdm_agent_index_ )
        {
            for ( size_t j = 0; j < msg->factors[j].symbol_names.size(); j++ )
            {
                local_action_names_.push_back ( msg->factors[i].symbol_names[j] );
                action_ids_[j] = j; ///if not using named actions, the ID of each action is its position.
            }
        }
    }

    if ( using_named_actions_ )
    {
        for ( size_t i = 0; i < msg->factors[mdm_agent_index_].symbol_names.size(); i++ )
        {
            string name = msg->factors[mdm_agent_index_].symbol_names[i];
            if ( named_action_ids_.count ( name ) )
            {
                action_ids_[i] = named_action_ids_[name];
                ///when using named actions, each action is paired with an ID that was specified when it was added to the Layer.
            }
            else
            {
                ROS_FATAL_STREAM ( "ActionLayer:: Action " << name << " is specified in the model metadata"
                                   << " but it isn't present in the Action Layer. Please define this action." );
                shutdown();
                return;
            }
        }
    }

    if ( pending_action_ != 0 )
    {
        actionSymbolCallback ( pending_action_ );
        pending_action_.reset();
    }
}



uint32_t
ActionLayer::
jointToIndividualAction ( const uint32_t joint_action )
{
    if ( action_sizes_.empty() )
    {
        ROS_FATAL ( "ActionLayer:: Attempted to convert a joint action to an individual action, but the number of actions per agent is not known." );
        shutdown();
        return ( 0 );
    }

    if ( cached_sizes_prod_ == 0 )
    {
        cached_sizes_prod_ = 1;
        for ( ActionSizesVector::iterator it = action_sizes_.begin() +mdm_agent_index_+1;
                it != action_sizes_.end();
                it ++ )
        {
            cached_sizes_prod_ *= ( uint32_t ) ( *it );
        }
    }

    uint32_t local_action = floor ( ( joint_action % ( cached_sizes_prod_ * action_sizes_[mdm_agent_index_] ) ) / cached_sizes_prod_ );

    return local_action;
}
