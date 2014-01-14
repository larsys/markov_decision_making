/**\file observation_layer.h
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

#ifndef _STATE_LAYER_H_
#define _STATE_LAYER_H_

#include <string>

#include <ros/ros.h>

#include <event_manager/event_manager.h>
#include <event_manager/EventInfoMap.h>
#include <event_manager/Events.h>
#include <markov_decision_making/ObservationMetadata.h>
#include <markov_decision_making/observation_dep.h>



namespace markov_decision_making
{
  class ObservationLayer
  {
    public:
      typedef boost::shared_ptr<std::map<uint32_t, uint32_t> > of_map_type; ///this type maps observation factor indexes and values
      typedef std::pair<uint32_t, uint32_t> id_key; ///<MDM agent ID, event ID>
      typedef std::pair<uint32_t, std::string> named_key; ///<MDM agent ID, event name>
      typedef std::map<std::string, uint32_t> name_id_map_type;
      
      ObservationLayer();
      
      void eventUpdatesCallback (const event_manager::EventsConstPtr& msg);
      
      void eventMapCallback (const event_manager::EventInfoMapConstPtr& msg);
      
      void observationMetadataCallback (const markov_decision_making::ObservationMetadataConstPtr& msg);
      
      void addObservationFactor (ObservationDep deps);
      
    private:
    
      void publishJointObservation();
      
      void updateObservationInfo (const uint32_t& factor);
      
      ros::NodeHandle nh_;
      
      ros::Subscriber event_map_sub_;
      ros::Subscriber event_update_sub_;
      
      std::map<uint32_t, name_id_map_type> agent_event_ids_;
      
      ros::Subscriber observation_metadata_sub_;
      ros::Publisher observation_pub_;
      
      ///Each pair <Agent ID, Event ID> may be associated to a set of dependent factors
      std::map<named_key, of_map_type > named_local_event_observer_;
      ///Observations can also be triggered by global events (i.e. from any agent). In that case, they don't have a fixed ID.
      std::map<std::string, of_map_type > named_global_event_observer_;
      
      std::map<id_key, of_map_type > id_event_observer_;
      
      std::vector<ObservationDep> factored_observations_deps_;
      
      std::vector<uint32_t> factored_observations_;
  };
}

#endif
