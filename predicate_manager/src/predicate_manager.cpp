/**\file predicate_manager.cpp
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * Predicate Manager is a ROS library to define and manage logical predicates and events.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of Predicate Manager.
 *
 * Predicate Manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Predicate Manager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <time.h> ///To control the update rate

#include <boost/bind.hpp>

#include <predicate_manager/predicate_manager.h>

using namespace std;
using namespace ros;
using namespace predicate_manager;

/**
 * TODO: RequestPredicateMap e RequestPredicateUpdate service
 */

PredicateManager::
PredicateManager() :
    nh_ (),
    started_ ( false ),
    any_changed_ ( false ),
    pm_id_ ( 0 ),
    nr_of_predicates_(),
    initialized_pms_(),
    update_period_ ( 0.1 ),
    predicate_maps_pub_ (),
    predicate_updates_pub_ (),
    predicate_maps_sub_ ( nh_.subscribe ( "/predicate_maps", 10, &PredicateManager::consumePredicateMap, this ) ),
    predicate_updates_sub_ ( nh_.subscribe ( "/predicate_updates", 10, &PredicateManager::consumePredicateUpdate, this ) ),
    pred_update_counters_(),
    local_pred_refs_ (),
    value_map_ (),
    last_value_map_ (),
    registered_predicates_ (),
    pred_name_observer_ (),
    pred_nr_observer_ (),
    event_maps_pub_ (),
    event_updates_pub_ (),
    ev_update_counter_ ( 0 ),
    local_ev_refs_ (),
    registered_events_ (),
    ev_name_observer_ (),
    ev_nr_observer_ (),
    events_to_publish_ ()
{
    nh_.getParam ( "pm_id", ( int& ) pm_id_ );
    double update_rate;
    if ( nh_.getParam ( "pm_update_rate", update_rate ) )
    {
        update_period_ = 1.0/update_rate;
    }
}



void
PredicateManager::
spin()
{
    started_ = true;

    prepareLocalDependencies();

    if( local_pred_refs_.size() > 0 )
    {
      predicate_maps_pub_ = nh_.advertise<PredicateInfoMap> ( "/predicate_maps", 1, true );
      predicate_updates_pub_ = nh_.advertise<PredicateUpdate> ( "/predicate_updates", 1, true );
      publishPredicateMap();
    }

    if( local_ev_refs_.size() > 0 )
    {
      event_maps_pub_ = nh_.advertise<EventInfoMap> ( "/event_maps", 1, true );
      event_updates_pub_ = nh_.advertise<EventUpdate> ( "/event_updates", 1, true );
      publishEventMap();
    }

    spinOnce();

    foreach ( PredRefVector::value_type pred_ref, local_pred_refs_ )
    {
        pred_ref->update();
    }
    publishPredicateUpdate();

    while ( ok() )
    {
        struct timeval pre_update;
        gettimeofday ( &pre_update, NULL );

        PMUpdate();

        struct timeval post_update;
        gettimeofday ( &post_update, NULL );

        double update_duration = post_update.tv_sec-pre_update.tv_sec +
                                 ( post_update.tv_usec-pre_update.tv_usec ) /1000000.0;

        Duration rem ( update_period_- update_duration );
        if ( update_duration > update_period_ )
            ROS_WARN_STREAM ( "PredicateManager:: Update took longer than the desired update period! Update time: " << update_duration
                              << "; Desired update period: " << update_period_ );
        else
        {
            rem.sleep();
        }
    }
}


void
PredicateManager::
PMUpdate()
{
    any_changed_ = false;
    spinOnce();
    if ( any_changed_ )
    {
        publishPredicateUpdate();
    }
    if ( events_to_publish_.size() > 0 )
    {
        publishEventUpdate();
    }
}


void
PredicateManager::
predicateUpdateTrigger ( const NrID pred_nr_id, bool value )
{
    any_changed_ = true;

    value_map_[pred_nr_id] = value;

    updateDependants ( pred_nr_id );
}

void
PredicateManager::
eventTrigger ( const uint32_t ev_nr )
{
    events_to_publish_.push_back ( ev_nr );
}

void
PredicateManager::
addPredicate ( Predicate& p )
{
    if ( started_ )
    {
        ROS_FATAL ( "Trying to add predicates after starting." );
        ros::shutdown();
    }

    string pred_name = p.getName();
    NameID pred_name_id ( pm_id_, pred_name );

    if ( registered_predicates_.count ( pred_name_id ) != 0 )
    {
        ROS_FATAL_STREAM ( "Trying to add a predicate that already exists. Predicate: " << pred_name );
        ros::shutdown();
    }

    uint32_t pred_nr = local_pred_refs_.size();
    registered_predicates_.insert ( pred_name_id );
    local_pred_refs_.push_back ( boost::shared_ptr<Predicate> ( &p ) );

    NrID pred_nr_id ( pm_id_, pred_nr );

    if ( ! ( pred_name_observer_[pred_name_id] ) ) ///pred_name_observer_[pred_name_id] may have already been created by a dependant predicate / event
    {
        pred_name_observer_[pred_name_id] = boost::shared_ptr<set<uint32_t> > ( new set<uint32_t>() );
    }

    pred_nr_observer_[pred_nr_id] = pred_name_observer_[pred_name_id];
    ev_nr_observer_[pred_nr_id] = ev_name_observer_[pred_name_id];

    NameIDSet dep_set = p.getDependencies();

    foreach ( NameID id, dep_set )
    {
        if ( ! ( pred_name_observer_[id] ) )
        {
            pred_name_observer_[id] = boost::shared_ptr<set<uint32_t> > ( new set<uint32_t>() );
        }
        pred_name_observer_[id]->insert ( pred_nr );
    }

    p.setTrigger ( boost::bind ( &PredicateManager::predicateUpdateTrigger, this, pred_nr_id,_1 ) );
}

void
PredicateManager::
addEvent ( Event& e )
{
    if ( started_ )
    {
        ROS_FATAL ( "Trying to add events after starting." );
        ros::shutdown();
    }

    string ev_name = e.getName();
    NameID ev_name_id ( pm_id_, ev_name );

    if ( registered_events_.count ( ev_name_id ) != 0 )
    {
        ROS_FATAL_STREAM ( "Trying to add an event that already exists. Event: " << ev_name );
        ros::shutdown();
    }

    uint32_t ev_nr = local_ev_refs_.size();
    registered_events_.insert ( ev_name_id );
    local_ev_refs_.push_back ( boost::shared_ptr<Event> ( &e ) );

    NameIDSet dep_set = e.getDependencies();

    foreach ( NameID id, dep_set )
    {
        if ( ! ( ev_name_observer_[id] ) )
        {
            ev_name_observer_[id] = boost::shared_ptr<set<uint32_t> > ( new set<uint32_t>() );
        }
        ev_name_observer_[id]->insert ( ev_nr );
    }

    e.setTrigger ( boost::bind ( &PredicateManager::eventTrigger, this, ev_nr ) );
}

void
PredicateManager::
publishPredicateMap()
{
    PredicateInfoMap info_map;

    for ( uint32_t i = 0; i < local_pred_refs_.size(); i++ )
    {
        PredicateInfo info;
        info.name = local_pred_refs_[i]->getName();
        info.nr = i;
        info_map.map.push_back ( info );
    }

    info_map.pm_id = pm_id_;

    predicate_maps_pub_.publish ( info_map );
}

void
PredicateManager::
publishEventMap()
{
    EventInfoMap info_map;

    for ( uint32_t i = 0; i < local_ev_refs_.size(); i++ )
    {
        EventInfo info;
        info.name = local_ev_refs_[i]->getName();
        info.nr = i;
        info_map.map.push_back ( info );
    }

    info_map.pm_id = pm_id_;

    event_maps_pub_.publish ( info_map );
}

void
PredicateManager::
consumePredicateMap ( const PredicateInfoMapConstPtr& msg )
{
    if ( msg->pm_id != pm_id_ )
    {
        foreach ( PredicateInfo p_info, msg->map )
        {
            NameID pred_name_id ( msg->pm_id, p_info.name );
            if ( !registered_predicates_.count ( pred_name_id ) )
            {
                ROS_DEBUG_STREAM ( "Registering predicate " << p_info.name
                                   << " from PM " << msg->pm_id );
                NrID pred_nr_id ( msg->pm_id, p_info.nr );

                setDependencyReferences ( pred_nr_id, pred_name_id );

                value_map_[pred_nr_id] = false;
                last_value_map_[pred_nr_id] = false;
                registered_predicates_.insert ( pred_name_id );
            }
        }
        setNrOfPredicates ( msg->pm_id, msg->map.size() );
    }
}


void
PredicateManager::
publishPredicateUpdate()
{
    PredicateUpdate update;

    update.pm_id = pm_id_;

    for ( size_t pred_nr = 0; pred_nr < local_pred_refs_.size(); pred_nr++ )
    {
        NrID pred_nr_id ( pm_id_, pred_nr );
        string pred_name = local_pred_refs_[pred_nr]->getName();
        if ( value_map_[pred_nr_id] )
        {
            update.true_predicates.push_back ( pred_nr );
            ROS_DEBUG_STREAM ( "Predicate Manager:: Predicate " << pred_name << " is TRUE" );
        }
        else
        {
            ROS_DEBUG_STREAM ( "Predicate Manager:: Predicate " << pred_name << " is FALSE" );
        }
        if ( last_value_map_.count ( pred_nr_id ) )
        {
            if ( last_value_map_[pred_nr_id] != value_map_[pred_nr_id] )
            {
                if ( last_value_map_[pred_nr_id] )
                {
                    update.falling_predicates.push_back ( pred_nr );
                }
                else
                {
                    update.rising_predicates.push_back ( pred_nr );
                }
            }
        }
        last_value_map_[pred_nr_id] = value_map_[pred_nr_id];
    }

    update.update_counter = pred_update_counters_[pm_id_];

    if ( pred_update_counters_[pm_id_] == MAX_UPDATE_COUNTER )
        pred_update_counters_[pm_id_] = 0;
    else
        pred_update_counters_[pm_id_]++;

    predicate_updates_pub_.publish ( update );
}

void
PredicateManager::
publishEventUpdate()
{
    EventUpdate update;

    update.pm_id = pm_id_;

    foreach ( uint32_t ev, events_to_publish_ )
    {
        update.events.push_back ( ev );
        //ROS_DEBUG_STREAM("Predicate Manager:: Event " << pred_name << " is TRUE");
    }

    update.update_counter = ev_update_counter_;

    if ( ev_update_counter_ == MAX_UPDATE_COUNTER )
        ev_update_counter_ = 0;
    else
        ev_update_counter_++;

    event_updates_pub_.publish ( update );

    events_to_publish_.clear();
}

void
PredicateManager::
consumePredicateUpdate ( const PredicateUpdateConstPtr& msg )
{
    if ( msg->pm_id != pm_id_ )
    {
        if ( initialized_pms_.size() <= msg->pm_id )
        {
            initialized_pms_.resize ( msg->pm_id + 1, false );
        }
        if ( pred_update_counters_.size() <= msg->pm_id )
        {
            pred_update_counters_.resize ( msg->pm_id + 1, 0 );
        }
        if ( !initialized_pms_[msg->pm_id] || pred_update_counters_[msg->pm_id] != msg->update_counter )
        {
            ///full update -- only done at initialization or if out of synch
            for ( size_t i = 0; i < nr_of_predicates_[msg->pm_id]; i++ )
            {
                NrID pred_nr_id ( msg->pm_id, i );
                value_map_[pred_nr_id] = false;
            }

            foreach ( uint32_t pred_nr, msg->true_predicates )
            {
                NrID pred_nr_id ( msg->pm_id, pred_nr );
                value_map_[pred_nr_id] = true;
            }

            for ( size_t i = 0; i < nr_of_predicates_[msg->pm_id]; i++ )
            {
                NrID pred_nr_id ( msg->pm_id, i );
                updateDependants ( pred_nr_id );
            }

            pred_update_counters_[msg->pm_id] = msg->update_counter;
        }
        else
        {
            ///reduced update
            foreach ( uint32_t pred_nr, msg->falling_predicates )
            {
                NrID pred_nr_id ( msg->pm_id, pred_nr );
                value_map_[pred_nr_id] = false;
                updateDependants ( pred_nr_id );
            }
            foreach ( uint32_t pred_nr, msg->rising_predicates )
            {
                NrID pred_nr_id ( msg->pm_id, pred_nr );
                value_map_[pred_nr_id] = true;
                updateDependants ( pred_nr_id );
            }

            if ( pred_update_counters_[msg->pm_id] == MAX_UPDATE_COUNTER )
                pred_update_counters_[msg->pm_id] = 0;
            else
                pred_update_counters_[msg->pm_id]++;
        }
    }
}


void
PredicateManager::
updateDependants ( const NrID pred_nr_id )
{
    ///Dependent predicates
    if ( pred_nr_observer_.count ( pred_nr_id ) )
    {
        ObserverPtr obs_ptr = pred_nr_observer_[pred_nr_id];
        if ( obs_ptr != boost::shared_ptr<std::set<uint32_t> >() )
        {
            foreach ( uint32_t dep_nr, *obs_ptr )
            {
                local_pred_refs_[dep_nr]->update();
                ROS_DEBUG_STREAM ( "Updating predicate '" << local_pred_refs_[dep_nr]->getName()
                                   << "' due to an updated dependency." );
            }
        }
    }
    ///Dependent events
    if ( ev_nr_observer_.count ( pred_nr_id ) )
    {
        ObserverPtr obs_ptr = ev_nr_observer_[pred_nr_id];
        if ( obs_ptr != boost::shared_ptr<std::set<uint32_t> >() )
        {
            foreach ( uint32_t dep_nr, *obs_ptr )
            {
                local_ev_refs_[dep_nr]->update();
                ROS_DEBUG_STREAM ( "Updating event '" << local_ev_refs_[dep_nr]->getName()
                                   << "' due to an updated dependency." );
            }
        }
    }
}

void
PredicateManager::
prepareLocalDependencies()
{
    for ( uint32_t i = 0; i < local_pred_refs_.size(); i++ )
    {
        PredicateInfo info;
        string pred_name = local_pred_refs_[i]->getName();
        NrID pred_nr_id ( pm_id_, i );
        NameID pred_name_id ( pm_id_, pred_name );

        setDependencyReferences ( pred_nr_id, pred_name_id );
    }
    setNrOfPredicates ( pm_id_, local_pred_refs_.size() );

    if ( pred_update_counters_.size() <= pm_id_ )
    {
        pred_update_counters_.resize ( pm_id_ + 1, 0 );
    }
}

void
PredicateManager::
setDependencyReferences ( const NrID pred_nr_id, const NameID pred_name_id )
{
    if ( pred_name_observer_[pred_name_id] )
    {
        pred_nr_observer_[pred_nr_id] = pred_name_observer_[pred_name_id];
        ObserverPtr obs_ptr = pred_nr_observer_[pred_nr_id];

        foreach ( uint32_t pred_nr, *obs_ptr )
        {
            local_pred_refs_[pred_nr]->bindDependency ( pred_name_id, & ( value_map_[pred_nr_id] ) );
        }
    }

    if ( ev_name_observer_[pred_name_id] )
    {
        ev_nr_observer_[pred_nr_id] = ev_name_observer_[pred_name_id];
        ObserverPtr obs_ptr = ev_nr_observer_[pred_nr_id];

        foreach ( uint32_t ev_nr, *obs_ptr )
        {
            local_ev_refs_[ev_nr]->bindDependency ( pred_name_id, & ( value_map_[pred_nr_id] ) );
        }
    }
}


void
PredicateManager::
setNrOfPredicates ( const int pm_id, const size_t size )
{
    if ( nr_of_predicates_.size() <= pm_id )
    {
        nr_of_predicates_.resize ( pm_id + 1 );
    }
    nr_of_predicates_[pm_id] = size;
}
