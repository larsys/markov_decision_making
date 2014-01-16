/**\file state_layer.cpp
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

#include <markov_decision_making/common_defs.h>
#include <markov_decision_making/state_layer.h>
#include <markov_decision_making/WorldSymbol.h>



using namespace std;
using namespace predicate_manager;
using namespace markov_decision_making;



StateLayer::
StateLayer() :
    pred_map_sub_ ( nh_.subscribe ( "/predicate_maps", 1, &StateLayer::predicateMapCallback, this ) ),
    pred_update_sub_ ( nh_.subscribe ( "/predicate_updates", 1, &StateLayer::predicateUpdatesCallback, this ) ),
    state_pub_ ( nh_.advertise<WorldSymbol> ( "state", 1, true ) )
{}



void
StateLayer::
predicateUpdatesCallback ( const PredicateUpdateConstPtr& msg )
{
    if ( nr_value_map_.empty() )
    {
        ROS_WARN ( "Predicate names table is empty. Trying to re-subscribe to predicate updates and maps." );
        pred_update_sub_.shutdown();
        pred_update_sub_ = nh_.subscribe ( "/predicate_updates", 1, &StateLayer::predicateUpdatesCallback, this );
        pred_map_sub_.shutdown();
        pred_map_sub_ = nh_.subscribe ( "/predicate_maps", 1, &StateLayer::predicateMapCallback, this );

        return;
    }
    SFIDSetPtr sf_to_update ( new set<uint32_t> );

    boost::unordered_map<NrID, boost::shared_ptr<bool> >::const_iterator value_it;
    boost::unordered_map<NrID, SFIDSetPtr >::const_iterator observer_it;
    foreach ( PredicateUpdate::_true_predicates_type::value_type pred_id, msg->true_predicates )
    {
        NrID nr_id ( msg->pm_id, pred_id );
        value_it = nr_value_map_.find ( nr_id );
        if ( value_it != nr_value_map_.end() )
        {
            * ( value_it->second ) = true;
            if ( nr_pred_observer_[nr_id] != 0 && !nr_pred_observer_[nr_id]->empty() )
            {
                sf_to_update->insert ( nr_pred_observer_[nr_id]->begin(), nr_pred_observer_[nr_id]->end() );
            }
        }
    }

    foreach ( PredicateUpdate::_falling_predicates_type::value_type pred_id, msg->falling_predicates )
    {
        NrID nr_id ( msg->pm_id, pred_id );
        value_it = nr_value_map_.find ( nr_id );
        if ( value_it != nr_value_map_.end() )
        {
            * ( value_it->second ) = false;
            if ( nr_pred_observer_[nr_id] != 0 && !nr_pred_observer_[nr_id]->empty() )
            {
                sf_to_update->insert ( nr_pred_observer_[nr_id]->begin(), nr_pred_observer_[nr_id]->end() );
            }
        }
    }
    foreach ( uint32_t sf, *sf_to_update )
    {
        updateStateInfo ( sf );
    }
    if ( !sf_to_update->empty() )
    {
        publishJointState();
    }
}



std::string
StateLayer::
factorString ( unsigned k )
{
    std::stringstream s1;
    std::string s2;

    if ( factored_state_deps_[k].size() == 1 )
    {
        ///binary state factor
        foreach ( NameID nid, factored_state_deps_[k] )
        {
            if ( named_value_map_.count ( nid ) )
            {
                uint32_t pm_id = nid.first;
                string pred_name = nid.second;
                s1 << "PM_Id:" << pm_id << "," << pred_name << "=" << * ( named_value_map_[nid] );
            }
        }
    }
    else
    {
        ///mutually-exclusive-valued state factor
        foreach ( NameID nid, factored_state_deps_[k] )
        {
            uint32_t pm_id = nid.first;
            string pred_name = nid.second;
            if ( named_value_map_.count ( nid ) && * ( named_value_map_[nid] ) == true )
            {
                s1 << "PM_Id:" << pm_id << "," << pred_name;
            }
        }
    }
    s1 >> s2;
    return s2;
}



void
StateLayer::
predicateMapCallback ( const PredicateInfoMapConstPtr& msg )
{
    foreach ( PredicateInfo p, msg->map )
    {
        NameID name_id ( msg->pm_id, p.name );
        NrID nr_id ( msg->pm_id, p.nr );

        boost::unordered_map<NameID, boost::shared_ptr<bool> >::const_iterator it;
        it = named_value_map_.find ( name_id );
        if ( it != named_value_map_.end() ) ///name_id has been registered as the dependency of some SF
        {
            nr_value_map_[nr_id] = it->second; ///now we can access the value by the NrID instead
        }
        nr_pred_observer_[nr_id] = named_pred_observer_[name_id];
    }
}



void
StateLayer::
addStateFactor ( const StateDep::SFDeps& deps )
{
    factored_state_deps_.push_back ( deps );

    foreach ( NameID name_id, deps )
    {
        if ( !named_pred_observer_.count ( name_id ) )
        {
            named_pred_observer_[name_id] = ( SFIDSetPtr ) ( new set<uint32_t> );
            named_value_map_[name_id] = boost::shared_ptr<bool> ( new bool ( false ) );
        }
        named_pred_observer_[name_id]->insert ( factored_state_.size() ); ///factor index is factored_state_.size()
    }

    factored_state_.push_back ( 0 );
}



void
StateLayer::
addStateFactor ( const StateDep& deps )
{
    addStateFactor ( deps.getDependencies() );
}



void
StateLayer::
publishJointState()
{
    WorldSymbol ws;
    uint32_t joint_state = 0;
    uint32_t prod = 1;
    size_t N = factored_state_deps_.size() - 1;
    for ( size_t i = 0; i <= N; i++ )
    {
        ROS_DEBUG_STREAM ( "StateLayer:: Factor " << N - i << ": " << factored_state_[N - i]
                           << " (" << factorString ( N - i ) << ")" );
        joint_state += ( prod * factored_state_[N - i] );
        if ( factored_state_deps_[N - i].size() > 1 )
        {
            prod *= factored_state_deps_[N - i].size();
        }
        else
        {
            prod *= 2;
        }
    }
    ws.world_symbol = joint_state;
    state_pub_.publish ( ws );
}



void
StateLayer::
updateStateInfo ( const uint32_t& factor )
{
    size_t i = 0, val = factored_state_deps_[factor].size();
    bool any = false;
    foreach ( NameID name_id, factored_state_deps_[factor] )
    {
        uint32_t pm_id = name_id.first;
        string pred_name = name_id.second;

        boost::unordered_map<NameID, boost::shared_ptr<bool> >::const_iterator it;
        it = named_value_map_.find ( name_id );
        if ( it ==  named_value_map_.end() )
        {
            ROS_WARN_STREAM ( "StateLayer:: Predicate " << pred_name << " from PM " << pm_id << " is not known to this agent." );
            ROS_WARN_STREAM ( "StateLayer:: Skipping the update of State Factor " << factor );
            return;
        }

        if ( * ( it->second ) == true )
        {
            if ( any )
            {
                ROS_WARN_STREAM ( "StateLayer:: Multiple dependencies active for state factor " << factor << " -- "
                                  << name_id.first << "(" << name_id.second << "); "
                                  << factored_state_deps_[factor][val].first << "(" << factored_state_deps_[factor][val].second << "); " );
                ROS_WARN_STREAM ( "StateLayer:: Multi-valued state factors should depend on mutually exclusive predicates." );
            }
            val = i;
            any = true;
        }
        i++;
    }

    if ( factored_state_deps_[factor].size() == 1 )
    {
        //binary state factor
        factored_state_[factor] = ( uint32_t ) any;
    }
    else
    {
        //n-ary state factor
        if ( val < factored_state_deps_[factor].size() )
        {
            factored_state_[factor] = val;
        }
        else
        {
            ROS_WARN_STREAM ( "StateLayer:: No dependencies active for n-ary state factor " << factor );
        }
    }
}
