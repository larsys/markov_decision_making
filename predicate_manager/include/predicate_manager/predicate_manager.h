/**\file predicate_manager.h
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

#ifndef _PREDICATE_MANAGER_H_
#define _PREDICATE_MANAGER_H_

#include <string>
#include <map>
#include <vector>

#include <boost/function.hpp>

#include <ros/ros.h>

#include <predicate_manager/common_defs.h>
#include <predicate_manager/predicate.h>
#include <predicate_manager/event.h>

#include <predicate_manager/PredicateInfoMap.h>
#include <predicate_manager/PredicateUpdate.h>
#include <predicate_manager/EventInfoMap.h>
#include <predicate_manager/EventUpdate.h>

#define MAX_UPDATE_COUNTER 4294967295 ///maximum value for a 32-bit usigned int

namespace predicate_manager
{
/**
 * The PredicateManager (PM) class handles Predicate interdependencies, and the communication of Predicate value updates.
 * In a multiagent system, multiple PredicateManager instances may be used concurrently, each of them representing the
 * knowledge that is locally available to each agent. Each active PM instance is identified by an integer ID, which you
 * should specify as a "pm_id" integer parameter in the containing node's local namespace.
 *
 * Predicate updates are communicated as arrays of unsigned integers (check PredicateUpdate.msg), and not by their name,
 * in order to minimize the ammount of information that must be sent at each update. Predicates updates are published by default
 * to the "/predicate_updates" topic (note the absolute path). A Predicate "map", which is translation table from
 * predicate numbers to their names, is sent to "/predicate_maps" when the PM is initialized. Using these maps,
 * a Predicate's number and the respective PM ID uniquely identifies a given predicate in the network.
 *
 * Predicates in a PM can have external dependencies (i.e. Predicates of another PM). All active PMs will attempt to maintain
 * synchrony even if the communication medium is not fully reliable.
 *
 * Registered Predicates are woken up (by calling their update() functions) whenever one of their respective dependencies
 * changes its value.
 */
class PredicateManager
{
public:
    ///An "Observer" is a set of local identifiers (predicate / event numbers). This type is used to represent local dependants.
    typedef boost::shared_ptr<std::set<uint32_t> > ObserverPtr;
    ///A map from NameIDs to local dependants. The access time is O(1) on average (see boost::unordered_map).
    typedef boost::unordered_map<NameID, ObserverPtr> NameIDObserver;
    ///A map from NrIDs to local dependants. The Cantor pairing function ensures O(1) access (no collisions).
    typedef boost::unordered_map<NrID, ObserverPtr, cantor_pair_hash> NrIDObserver;
    ///A map from NrIDs to the values of the respective predicates.
    typedef boost::unordered_map<NrID, bool, cantor_pair_hash> ValueMap;
    ///A vector of predicate references (used to represent the locally registered predicate objects).
    typedef std::vector<boost::shared_ptr<Predicate> > PredRefVector; ///maps local pred nrs to pred objects
    typedef std::vector<boost::shared_ptr<Event> > EvRefVector; ///maps local pred nrs to pred objects

    ///Standard constructor.
    PredicateManager();

    /**
     * PredicateManager implements its own spin function to ensure correct initialization,
     * and that updates are only published when needed. You *must* call this spin function in
     * the parent node, instead of the default ros::spin().
     */
    void spin();

    /**
     * Adds a Predicate to this PredicateManager.
     * Predicates must be added before the PM starts spinning.
     * @param p The predicate to be registered.
     */
    void addPredicate ( Predicate& p );

    /**
     * Adds an Event to this PredicateManager.
     * Events must be added before the PM starts spinning.
     * @param p The predicate to be registered.
     */
    void addEvent ( Event& e );

    /**
     * A trigger function that is called by each Predicate as it updates its value.
     * This prompts the PM to update the dependants of that Predicate.
     * @param pred_nr_id The NrID of the calling predicate (bound during predicate registration);
     * @param value The new value of the calling predicate.
     */
    void predicateUpdateTrigger ( const NrID pred_nr_id, bool value );

    /**
     * A trigger function that is called by each Event when it is ready to be published.
     * @param ev_nr_id The number of the calling event (bound during event registration). Calling events are always local.
     */
    void eventTrigger ( const uint32_t ev_nr );

private:
    /**
     * An update cycle of this PM. Checks for changed values and publishes predicate updates accordingly.
     */
    void PMUpdate();

    /**
     * Consumes Predicate Maps from other Predicate Managers.
     * Predicate dependencies are specified by name, but predicates are communicated as integers.
     * This function establishes a correspondency between the identifiers of future predicate updates
     * from the sender PM, and the local dependencies specified in this receiver PM.
     */
    void consumePredicateMap ( const PredicateInfoMapConstPtr& msg );
    /**
     * Consumes Predicate updates from other Predicate Managers.
     * This updates the local value map and calls local dependent Predicates.
     * If all messages are being correctly transmitted, this PM only uses the
     * "rising" and "falling" predicate sets in the input message to update its values.
     * Otherwise, all known Predicates of the sender PM are forcefully updated.
     */
    void consumePredicateUpdate ( const PredicateUpdateConstPtr& msg );
    /**
     * Publishes this PM's Predicate Map to the "/predicate_maps" topic.
     */
    void publishPredicateMap();
    /**
     * Publishes an update of this PM's Predicate values.
     */
    void publishPredicateUpdate();
    /**
     * Publishes this PM's Event Map to the "/event_maps" topic.
     */
    void publishEventMap();
    /**
     * Publishes an update of this PM's events.
     */
    void publishEventUpdate();
    /**
     * Updates the dependants of a Predicate (the Predicates that depend on it).
     * @param pred_nr_id The NrID of the Predicate whose dependants we want to update.
     */
    void updateDependants ( const NrID pred_nr_id );
    /**
     * Establishes the correct cross-references between local predicates.
     * This must be done after adding all Predicates, and before spinning the PM.
     */
    void prepareLocalDependencies();
    /**
     * Binds a Predicate to its dependents. This allows the dependent Predicates/Events to get the values
     * of its dependency from this PM.
     * @param pred_nr_id The NrID of the target Predicate;
     * @param pred_name_id The NameID of the target Predicate.
     */
    void setDependencyReferences ( const NrID pred_nr_id, const NameID pred_name_id );
    /**
     * Records the number of predicates for a given PM. This is necessary to perform
     * full updates.
     * @param pm_id The ID of the target PM.
     * @param size The number of Predicates registered to that PM. This is inferred from that
     * PM's predicate map.
     */
    void setNrOfPredicates ( const int pm_id, const std::size_t size );

    ros::NodeHandle nh_;

    bool started_; ///Wether this PM has started spinning.
    bool any_changed_; ///Whether there has been at least one Predicate that changed its value.

    uint32_t pm_id_; ///The ID of this PM.
    std::vector<std::size_t> nr_of_predicates_; ///Vector containing the number of predicates for each known PM.
    std::vector<bool> initialized_pms_; ///Vector containing the IDs of all known initialized PMs.
    double update_period_; ///The update period of this Predicate Manager. Default is 0.1s (10 Hz).

    ///Predicate-related members:

    ros::Publisher predicate_maps_pub_; ///Publisher for this PM's predicate map.
    ros::Publisher predicate_updates_pub_; ///Publisher for this PM's predicate updates.
    ros::Subscriber predicate_maps_sub_; ///Subscriber to the predicate maps of other PMs.
    ros::Subscriber predicate_updates_sub_; ///Subscriber to the predicate updates of other PMs.

    std::vector<uint32_t> pred_update_counters_; ///A vector of integer tokens that act as a "synchronization key" for predicate updates.
    PredRefVector local_pred_refs_; ///Vector of references to locally registered Predicate Objects.
    ValueMap value_map_; ///Map containing the up-to-date value of all known Predicates.
    ValueMap last_value_map_; ///The value map according to the last published update.
    NameIDSet registered_predicates_; ///The NameIDs of all registered local predicates.
    NameIDObserver pred_name_observer_; ///The Predicate observer set for each Predicate, accessible through its NameID.
    NrIDObserver pred_nr_observer_; ///The Predicate observer set for each Predicate, accessible through its NrID.

    ///Event-related members:

    ros::Publisher event_maps_pub_;
    ros::Publisher event_updates_pub_;

    uint32_t ev_update_counter_; ///A vector of integer tokens that act as a "synchronization key" for event updates.
    EvRefVector local_ev_refs_; ///Vector of references to locally registered Event Objects.
    NameIDSet registered_events_; ///The NameIDs of all registered local events.
    NameIDObserver ev_name_observer_; ///The Event observer set for each Predicate, accessible through its NameID.
    NrIDObserver ev_nr_observer_; ///The Event observer set for each Predicate, accessible through its NrID.
    std::vector<uint32_t> events_to_publish_; ///The events that have been flagged for publication in this cycle (i.e. called the trigger function).
};
}

#endif
