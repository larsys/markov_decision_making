/**\file observation_layer.cpp
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


#include <markov_decision_making/observation_layer.h>
#include <markov_decision_making/WorldSymbol.h>



using namespace std;
using namespace predicate_manager;
using namespace markov_decision_making;



ObservationLayer::
ObservationLayer() :
    event_map_sub_ ( nh_.subscribe ( "/event_maps", 1, &ObservationLayer::eventMapCallback, this ) ),
    event_update_sub_ ( nh_.subscribe ( "/event_updates", 1, &ObservationLayer::eventUpdatesCallback, this ) ),
    observation_metadata_sub_ ( nh_.subscribe ( "observation_metadata", 1, &ObservationLayer::observationMetadataCallback, this ) ),
    observation_pub_ ( nh_.advertise<WorldSymbol> ( "observation", 1, true ) )
{}



void
ObservationLayer::
eventUpdatesCallback ( const EventUpdateConstPtr& msg )
{
    bool new_observation = false;

    foreach ( EventUpdate::_events_type::value_type event_id, msg->events )
    {
        NrID ev_nr_id ( msg->pm_id, event_id );
        if ( event_nr_observer_.count ( ev_nr_id ) )
        {
            ObsValuePair value_pair = event_nr_observer_[ev_nr_id]; 
            factored_observations_[value_pair.first] = value_pair.second;
            new_observation = true;
        }
    }

    if ( new_observation )
    {
        publishJointObservation();
    }
}



void
ObservationLayer::
eventMapCallback ( const EventInfoMapConstPtr& msg )
{
    foreach ( EventInfo e, msg->map )
    {
        NameID ev_name_id ( msg->pm_id, e.name );
        NrID ev_nr_id ( msg->pm_id, e.nr );

        if ( !event_nr_observer_.count(ev_nr_id) )
        {
            if( event_named_local_observer_.count(ev_name_id) )
                event_nr_observer_[ev_nr_id] = event_named_local_observer_[ev_name_id];
            else if (event_named_global_observer_.count(e.name) )
                event_nr_observer_[ev_nr_id] = event_named_global_observer_[e.name];
        }        
    }
}



void
ObservationLayer::
observationMetadataCallback ( const FactoredSymbolMetadataConstPtr& msg )
{
    if ( msg->factors.size() != factored_observations_.size() )
    {
        ROS_WARN_STREAM ( "ObservationLayer:: Metadata check failed. The number of declared observation factors "
                          << " (" << factored_observations_.size() << ") is different from the number of expected observation factors ("
                          << msg->factors.size() << ")." );
    }

    for ( size_t i = 0; i < factored_observations_deps_.size() &&
            i < msg->factors.size(); i++ )
    {
        uint32_t declared_values = factored_observations_deps_[i].getNumberOfValues();
        uint32_t expected_values = msg->factors[i].number_of_symbols;
        if ( declared_values != expected_values )
        {
            ROS_WARN_STREAM ( "ObservationLayer:: Metadata check failed. The number of declared observations for factor "
                              << i << " (" << declared_values << ") is different from the number of expected observations ("
                              << expected_values << ")." );
        }
    }
}



void
ObservationLayer::
addObservationFactor ( ObservationDep deps )
{
    const ObservationDep::ObsFactorLocalDeps& local_deps = deps.getLocalDependencies();
    const ObservationDep::ObsFactorGlobalDeps& global_deps = deps.getGlobalDependencies();

    if ( local_deps.size() + global_deps.size() <= 1 )
    {
        ROS_FATAL("ObservationLayer:: Attempted to add an empty or unary observation factor. Observation Factors must be associated with at least two events." );
        ros::shutdown();
    }

    size_t new_factor_index = factored_observations_.size();

    factored_observations_.push_back ( 0 );
    
    foreach ( NameID ev_name_id, local_deps )
    {
        if ( event_named_local_observer_.count(ev_name_id) )
        {
            ROS_FATAL_STREAM("ObservationLayer:: Attempted to reassign dependency for event '" << ev_name_id.second << "' from PM " << ev_name_id.first);
            ROS_FATAL("ObservationLayer:: Events must be associated with a single observation factor / value");
            ros::shutdown();
        }
        event_named_local_observer_[ev_name_id] = make_pair ( new_factor_index, deps.getDependencyIndex ( ev_name_id.second ) );
    }

    foreach ( string event_name, global_deps )
    {
        if ( event_named_global_observer_.count(event_name) )
        {
            ROS_FATAL_STREAM("ObservationLayer:: Attempted to reassign dependency for global event '" << event_name << "'");
            ROS_FATAL("ObservationLayer:: Events must be associated with a single observation factor / value");
            ros::shutdown();          
        }
        event_named_global_observer_[event_name] = make_pair ( new_factor_index, deps.getDependencyIndex ( event_name ) );
    }

    factored_observations_deps_.push_back ( deps );
}



void
ObservationLayer::
publishJointObservation()
{
    WorldSymbol observationinfo;
    uint32_t joint_observation = 0;
    uint32_t prod = 1;
    size_t N = factored_observations_deps_.size() - 1;
    for ( size_t i = 0; i <= N; i++ )
    {
        joint_observation += ( prod * factored_observations_[N - i] );
        prod *= factored_observations_deps_[N - i].getNumberOfValues();
    }
    observationinfo.world_symbol = joint_observation;
    observation_pub_.publish ( observationinfo );
}
