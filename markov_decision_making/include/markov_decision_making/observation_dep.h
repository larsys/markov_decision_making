/**\file observation_dep.h
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

#ifndef _OBVSERVATION_DEP_H_
#define _OBVSERVATION_DEP_H_

#include <string>

#include <ros/ros.h>



namespace markov_decision_making
{
  class ObservationDep
  {
    public:
      typedef std::vector<std::pair<uint32_t, std::string> > of_local_deps_type; ///<agent ID, event name>
      typedef std::vector<std::string > of_global_deps_type;
      typedef std::map<std::string, uint32_t> dep_sequence_type;
      
      ObservationDep
      add (uint32_t agent_id, std::string event_name) {
        registerDependencyName (event_name);
        of_local_deps_type::value_type odep (agent_id, event_name);
        local_deps_.push_back (odep);
        return *this;
      }
      
      ObservationDep
      add (std::string event_name) {
        registerDependencyName (event_name);
        of_global_deps_type::value_type odep (event_name);
        global_deps_.push_back (odep);
        return *this;
      }
      
      const of_local_deps_type getLocalDependencies() const {
        return local_deps_;
      }
      const of_global_deps_type getGlobalDependencies() const {
        return global_deps_;
      }
      uint32_t getDependencyIndex (const std::string& event_name) {
        if (!named_dep_sequence_.count (event_name)) {
          ROS_ERROR_STREAM ("ObservationDep:: Dependency '" << event_name << "' not present for this factor.");
          abort();
        }
        return named_dep_sequence_[event_name];
      }
      uint32_t getNumberOfValues() const {
        return local_deps_.size() + global_deps_.size();
      }
    private:
      void registerDependencyName (const std::string& event_name) {
        if (named_dep_sequence_.count (event_name)) {
          ROS_ERROR_STREAM ("ObservationDep:: Attempt to re-include dependency '" << event_name << "'");
          abort();
        }
        named_dep_sequence_[event_name] = getNumberOfValues();
      }
      
      ros::NodeHandle nh_;
      
      of_local_deps_type local_deps_;
      of_global_deps_type global_deps_;
      
      dep_sequence_type named_dep_sequence_;
  };
}
#endif
