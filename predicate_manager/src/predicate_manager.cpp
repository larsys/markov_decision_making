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

#include <predicate_manager/predicate_manager.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace std;
using namespace ros;
using namespace predicate_manager;

/**
 * TODO: - Integrar eventos;
 *       - Fazer só advertise quando eventos ou predicados são adicionados
 *       - nome dos tópicos de saída / entrada configuraveis
 *       - RequestPredicateMap e RequestPredicateUpdate service
 *       - Rate configuravel
 */

PredicateManager::
PredicateManager() :
  nh_ (),
  started_ (false),
  any_changed_ (false),
  pm_id_ (0),
  predicate_maps_pub_ (nh_.advertise<PredicateInfoMap> ("/predicate_maps", 1, true)),
  predicate_update_pub_ (nh_.advertise<PredicateUpdate> ("/predicate_updates", 1, true)),
  predicate_maps_sub_ (nh_.subscribe("/predicate_maps", 10, &PredicateManager::consumePredicateMap, this)),
  predicate_update_sub_ (nh_.subscribe("/predicate_updates", 10, &PredicateManager::consumeUpdate, this))
//   event_map_pub_ (nh_.advertise<EventInfoMap> ("event_map", 1, true)),
//   event_update_pub_ (nh_.advertise<EventUpdate> ("event_updates", 1, true))  
{
  nh_.getParam ("pm_id", (int&) pm_id_);
}



void
PredicateManager::
spin()
{
  started_ = true;
  
  prepareLocalDependencies();
  publishPredicateMap();
  
  spinOnce();
  
  foreach (PredRefVector::value_type pred_ref, local_pred_refs_) {
    pred_ref->update();
  }
  publishUpdate();
  
  Rate r (10);
  while (ok()) {
    PMUpdate();
    r.sleep();
  }
}


void
PredicateManager::
PMUpdate()
{
  any_changed_ = false;
  spinOnce();
  if (any_changed_) {
    publishUpdate();
  }
}

void
PredicateManager::
predicateUpdateTrigger(const NrID pred_nr_id, bool value)
{
  any_changed_ = true;
  
  value_map_[pred_nr_id] = value;
  
  ObserverPtr obs_ptr = pred_nr_observer_[pred_nr_id];
  
  if(obs_ptr != boost::shared_ptr<std::set<uint32_t> >())
  {
    foreach(uint32_t pred_nr, *obs_ptr)
    {
      local_pred_refs_[pred_nr]->update();
    }
  }
}

void
PredicateManager::
addPredicate (Predicate& p)
{
  if (started_) {
    ROS_FATAL ("Trying to create predicates after starting.");
    ros::shutdown();
  }
  
  string pred_name = p.getName();
  NameID pred_name_id (pm_id_, pred_name);
  
  if (registered_predicates_.count (pred_name_id) != 0) {
    ROS_FATAL_STREAM ("Trying to add a predicate that already exists. Predicate: " << pred_name);
    ros::shutdown();
  }
  
  uint32_t pred_nr = local_pred_refs_.size();
  registered_predicates_.insert(pred_name_id);
  local_pred_refs_.push_back(boost::shared_ptr<Predicate>(&p));
  
  NrID pred_nr_id (pm_id_, pred_nr);

  if(!(pred_name_observer_[pred_name_id])) ///pred_name_observer_[pred_name_id] may have already been created by a dependant predicate
  { 
    pred_name_observer_[pred_name_id] = boost::shared_ptr<set<uint32_t> >(new set<uint32_t>());
  }
  
  pred_nr_observer_[pred_nr_id] = pred_name_observer_[pred_name_id];  
 
  NameIDSet dep_set = p.getDependencies();
  
  foreach (NameID id, dep_set)
  { 
    if(!(pred_name_observer_[id]))
    {
      pred_name_observer_[id] = boost::shared_ptr<set<uint32_t> >(new set<uint32_t>());      
    }
    pred_name_observer_[id]->insert(pred_nr);
  }
  
  p.setTrigger(boost::bind(&PredicateManager::predicateUpdateTrigger, this, pred_nr_id,_1));
}



void
PredicateManager::
publishPredicateMap()
{
  PredicateInfoMap info_map;
  
  for(uint32_t i = 0; i < local_pred_refs_.size(); i++)
  {
    PredicateInfo info;
    info.name = local_pred_refs_[i]->getName();
    info.nr = i;
    info_map.map.push_back (info);
  }
    
  info_map.pm_id = pm_id_;
  
  predicate_maps_pub_.publish (info_map);
}



void
PredicateManager::
consumePredicateMap(const PredicateInfoMapConstPtr& msg)
{
  if(msg->pm_id != pm_id_)
  {
    foreach(PredicateInfo p_info, msg->map)
    {
      NameID pred_name_id (msg->pm_id, p_info.name);
      if(!registered_predicates_.count(pred_name_id)){
        ROS_DEBUG_STREAM("Registering predicate " << p_info.name
                          << " from PM " << msg->pm_id);
        NrID pred_nr_id (msg->pm_id, p_info.nr);
         
        setDependencyReferences(pred_nr_id, pred_name_id);
        
        value_map_[pred_nr_id] = false;
        last_value_map_[pred_nr_id] = false;
        registered_predicates_.insert(pred_name_id);
      }
    }
    setNrOfPredicates(msg->pm_id, msg->map.size());
  }
}



void
PredicateManager::
publishUpdate()
{
  PredicateUpdate update;
  
  update.pm_id = pm_id_;
  
  for(size_t pred_nr = 0; pred_nr < local_pred_refs_.size(); pred_nr++)
  {
    NrID pred_nr_id (pm_id_, pred_nr);
    string pred_name = local_pred_refs_[pred_nr]->getName();
    if (value_map_[pred_nr_id]) {
      update.true_predicates.push_back (pred_nr);
      ROS_DEBUG_STREAM("Predicate Manager:: Predicate " << pred_name << " is TRUE");
    }
    else
    {
      ROS_DEBUG_STREAM("Predicate Manager:: Predicate " << pred_name << " is FALSE");
    }
    if (last_value_map_.count(pred_nr_id) )
    {
      if (last_value_map_[pred_nr_id] != value_map_[pred_nr_id])
      {
        if (last_value_map_[pred_nr_id])
        {
          update.falling_predicates.push_back(pred_nr);
        }
        else
        {
          update.rising_predicates.push_back(pred_nr);
        }
      }
    }
    last_value_map_[pred_nr_id] = value_map_[pred_nr_id];
  }
  
  update.update_counter = counters_[pm_id_];
 
  if(counters_[pm_id_] == MAX_UPDATE_COUNTER)
    counters_[pm_id_] = 0;
  else
    counters_[pm_id_]++;
  
  predicate_update_pub_.publish (update);
}

void
PredicateManager::
consumeUpdate(const PredicateUpdateConstPtr& msg)
{
  if(msg->pm_id != pm_id_)
  {
    if(initialized_pms_.size() <= msg->pm_id)
    {
      initialized_pms_.resize(msg->pm_id + 1, false);      
    }
    if(counters_.size() <= msg->pm_id)
    {
      counters_.resize(msg->pm_id + 1, 0); 
    }
    if(!initialized_pms_[msg->pm_id] || counters_[msg->pm_id] != msg->update_counter)
    { ///full update -- only done at initialization or if out of synch
      for(size_t i = 0; i < nr_of_predicates_[msg->pm_id]; i++)
      {
        NrID pred_nr_id (msg->pm_id, i);
        value_map_[pred_nr_id] = false;
      }
      
      foreach(uint32_t pred_nr, msg->true_predicates)
      {
        NrID pred_nr_id (msg->pm_id, pred_nr);
        value_map_[pred_nr_id] = true;        
      }
      
      for(size_t i = 0; i < nr_of_predicates_[msg->pm_id]; i++)
      {
        NrID pred_nr_id (msg->pm_id, i);
        updateDependants(pred_nr_id);
      }
      
      counters_[msg->pm_id] = msg->update_counter;
    }
    else
    { ///reduced update    
      foreach(uint32_t pred_nr, msg->falling_predicates)
      {
        NrID pred_nr_id (msg->pm_id, pred_nr);
        value_map_[pred_nr_id] = false;
        updateDependants(pred_nr_id);
      }
      foreach(uint32_t pred_nr, msg->rising_predicates)
      {
        NrID pred_nr_id (msg->pm_id, pred_nr);
        value_map_[pred_nr_id] = true;
        updateDependants(pred_nr_id);
      }
      
      if(counters_[msg->pm_id] == MAX_UPDATE_COUNTER)
        counters_[msg->pm_id] = 0;
      else
        counters_[msg->pm_id]++;
    }
  }
}


void
PredicateManager::
updateDependants(const NrID pred_nr_id)
{
  if(pred_nr_observer_.count(pred_nr_id))
  {
    ObserverPtr obs_ptr = pred_nr_observer_[pred_nr_id];
    if(obs_ptr != boost::shared_ptr<std::set<uint32_t> >()){
      foreach(uint32_t dep_nr, *obs_ptr)
      {
        local_pred_refs_[dep_nr]->update();
        ROS_DEBUG_STREAM("Updating predicate '" << local_pred_refs_[dep_nr]->getName()
                        << "' due to an updated dependency.");
      }
    }
  } 
}

void
PredicateManager::
prepareLocalDependencies()
{
  for(uint32_t i = 0; i < local_pred_refs_.size(); i++)
  {
    PredicateInfo info;
    string pred_name = local_pred_refs_[i]->getName(); 
    NrID pred_nr_id (pm_id_, i);    
    NameID pred_name_id (pm_id_, pred_name);
    
    setDependencyReferences(pred_nr_id, pred_name_id);
  }
  setNrOfPredicates(pm_id_, local_pred_refs_.size());
  
  if(counters_.size() <= pm_id_)
  {
    counters_.resize(pm_id_ + 1, 0); 
  }
}

void
PredicateManager::
setDependencyReferences(const NrID pred_nr_id, const NameID pred_name_id)
{
  if(pred_name_observer_[pred_name_id])
  {
    pred_nr_observer_[pred_nr_id] = pred_name_observer_[pred_name_id];
    ObserverPtr obs_ptr = pred_nr_observer_[pred_nr_id];

    foreach(uint32_t pred_nr, *obs_ptr)
    {
      local_pred_refs_[pred_nr]->bindDependency(pred_name_id, &(value_map_[pred_nr_id]));
    }
  }
}

void
PredicateManager::
setNrOfPredicates(const int pm_id, const size_t size)
{
  if(nr_of_predicates_.size() <= pm_id)
  {
    nr_of_predicates_.resize(pm_id + 1);
  }
  nr_of_predicates_[pm_id] = size;
}