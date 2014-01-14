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

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <markov_decision_making/observation_layer.h>
#include <markov_decision_making/ObservationInfo.h>



using namespace std;
using namespace event_manager;
using namespace markov_decision_making;



ObservationLayer::
ObservationLayer() :
  event_map_sub_ (nh_.subscribe ("event_map", 1, &ObservationLayer::eventMapCallback, this)),
  event_update_sub_ (nh_.subscribe ("event_updates", 1, &ObservationLayer::eventUpdatesCallback, this)),
  observation_metadata_sub_ (nh_.subscribe ("observation_metadata", 1, &ObservationLayer::observationMetadataCallback, this)),
  observation_pub_ (nh_.advertise<ObservationInfo> ("observation", 1, true))
{}



void
ObservationLayer::
eventUpdatesCallback (const EventsConstPtr& msg)
{
  if (agent_event_ids_.empty() || agent_event_ids_[msg->agent_id].empty()) {
    ROS_WARN ("Event names table is empty. Trying to re-subscribe to event updates.");
    event_update_sub_.shutdown();
    event_update_sub_ = nh_.subscribe ("event_updates", 1, &ObservationLayer::eventUpdatesCallback, this);
    return;
  }
  bool new_observation = false;
  
  foreach (Events::_events_type::value_type event_id, msg->events) {
    id_key k = make_pair (msg->agent_id, event_id);
    if (id_event_observer_.count (k) && id_event_observer_[k] != 0) {
      of_map_type ofs_to_update = id_event_observer_[k];
      new_observation = true;
      
      foreach (const of_map_type::value_type::value_type & of_value_pair, *ofs_to_update) {
        factored_observations_[of_value_pair.first] = of_value_pair.second;
      }
    }
  }
  
  if (new_observation) {
    publishJointObservation();
  }
}



void
ObservationLayer::
eventMapCallback (const EventInfoMapConstPtr& msg)
{
  foreach (EventInfo e, msg->map) {
    //TODO se mensagens deixarem de ter id's, tenho de ter mais um argumento, e fazer bind na subscriçao
    named_key nk = make_pair (msg->agent_id, e.name);
    id_key ik = make_pair (msg->agent_id, e.id);
    
    agent_event_ids_[msg->agent_id][e.name] = e.id;
    
    id_event_observer_[ik] = named_local_event_observer_[nk];
    ///pushing global "dependees"
    if (named_global_event_observer_[e.name] != 0) {
      if (id_event_observer_[ik] == 0) {
        id_event_observer_[ik] = (of_map_type) (new map<uint32_t, uint32_t>);
      }
      
      id_event_observer_[ik]->insert (named_global_event_observer_[e.name]->begin(),
                                      named_global_event_observer_[e.name]->end());
    }
  }
}



void
ObservationLayer::
observationMetadataCallback (const ObservationMetadataConstPtr& msg)
{
  if (msg->number_of_observations.size() != factored_observations_.size()) {
    ROS_WARN_STREAM ("ObservationLayer:: Metadata check failed. The number of declared observation factors "
                     << " (" << factored_observations_.size() << ") is different from the number of expected observation factors ("
                     << msg->number_of_observations.size() << ").");
  }
  
  for (size_t i = 0; i < factored_observations_deps_.size() &&
       i < msg->number_of_observations.size(); i++) {
    uint32_t declared_values = factored_observations_deps_[i].getNumberOfValues();
    uint32_t expected_values = msg->number_of_observations[i];
    if (declared_values != expected_values) {
      ROS_WARN_STREAM ("ObservationLayer:: Metadata check failed. The number of declared observations for factor "
                       << i << " (" << declared_values << ") is different from the number of expected observations ("
                       << expected_values << ").");
    }
  }
}



void
ObservationLayer::
addObservationFactor (ObservationDep deps)
{
  const ObservationDep::of_local_deps_type& local_deps = deps.getLocalDependencies();
  const ObservationDep::of_global_deps_type& global_deps = deps.getGlobalDependencies();
  
  if (local_deps.size() + global_deps.size() <= 1) {
    ROS_ERROR ("ObservationLayer::Attempted to add an empty or unary observation factor. Observation Factors must be associated with at least two events.");
    abort();
  }
  
  size_t new_factor_index = factored_observations_.size();
  
  factored_observations_.push_back (0); ///Initial values shouldn't be necessary, but if they are, they should go here.
  
  foreach (named_key nk, local_deps) {
    if (named_local_event_observer_[nk] == 0) {
      named_local_event_observer_[nk] = (of_map_type) (new map<uint32_t, uint32_t>);
    }
    named_local_event_observer_[nk]->insert (make_pair (new_factor_index, deps.getDependencyIndex (nk.second)));
  }
  
  foreach (string event_name, global_deps) {
    if (named_global_event_observer_[event_name] == 0) {
      named_global_event_observer_[event_name] = (of_map_type) (new map<uint32_t, uint32_t>);
    }
    named_global_event_observer_[event_name]->insert (make_pair (new_factor_index, deps.getDependencyIndex (event_name)));
  }
  
  factored_observations_deps_.push_back (deps);
}



void
ObservationLayer::
publishJointObservation()
{
  ObservationInfo observationinfo;
  uint32_t joint_observation = 0;
  uint32_t prod = 1;
  size_t N = factored_observations_deps_.size() - 1;
  for (size_t i = 0; i <= N; i++) {
    joint_observation += (prod * factored_observations_[N - i]);
    prod *= factored_observations_deps_[N - i].getNumberOfValues();
  }
  observationinfo.observation = joint_observation;
  observation_pub_.publish (observationinfo);
}
