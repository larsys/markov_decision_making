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

#ifndef _OBSERVATION_DEP_H_
#define _OBSERVATION_DEP_H_

#include <string>

#include <boost/unordered_map.hpp>

#include <ros/ros.h>

#include <predicate_manager/common_defs.h>


namespace markov_decision_making
{
class ObservationDep
{
public:
    typedef std::vector< predicate_manager::NameID > ObsFactorLocalDeps;
    typedef std::vector< std::string > ObsFactorGlobalDeps;
    typedef boost::unordered_map< std::string, uint32_t > DepIndexMap;

    ObservationDep
    add ( uint32_t pm_id, std::string event_name )
    {
        registerDependencyName ( event_name );
        ObsFactorLocalDeps::value_type odep ( pm_id, event_name );
        local_deps_.push_back ( odep );
        return *this;
    }

    ObservationDep
    add ( std::string event_name )
    {
        registerDependencyName ( event_name );
        ObsFactorGlobalDeps::value_type odep ( event_name );
        global_deps_.push_back ( odep );
        return *this;
    }

    const ObsFactorLocalDeps getLocalDependencies() const
    {
        return local_deps_;
    }
    const ObsFactorGlobalDeps getGlobalDependencies() const
    {
        return global_deps_;
    }
    uint32_t getDependencyIndex ( const std::string& event_name )
    {
        DepIndexMap::const_iterator it = dep_index_map_.find ( event_name );
        if ( it == dep_index_map_.end() )
        {
            ROS_ERROR_STREAM ( "ObservationDep:: Dependency '" << event_name << "' not present for this factor." );
            abort();
        }
        return it->second;
    }
    uint32_t getNumberOfValues() const
    {
        return local_deps_.size() + global_deps_.size();
    }
private:
    void registerDependencyName ( const std::string& event_name )
    {
        if ( dep_index_map_.count ( event_name ) )
        {
            ROS_ERROR_STREAM ( "ObservationDep:: Attempt to re-include dependency '" << event_name << "'" );
            abort();
        }
        dep_index_map_[event_name] = getNumberOfValues();
    }

    ros::NodeHandle nh_;

    ObsFactorLocalDeps local_deps_;
    ObsFactorGlobalDeps global_deps_;

    DepIndexMap dep_index_map_;
};
}
#endif
