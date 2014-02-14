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

#ifndef _OBSERVATION_LAYER_H_
#define _OBSERVATION_LAYER_H_

#include <string>

#include <ros/ros.h>

#include <predicate_manager/predicate_manager.h>
#include <predicate_manager/EventInfoMap.h>
#include <predicate_manager/EventUpdate.h>
#include <mdm_library/FactoredSymbolMetadata.h>
#include <mdm_library/observation_dep.h>



namespace mdm_library
{
class ObservationLayer
{
public:
    typedef std::pair< size_t, uint32_t > ObsValuePair; ///ObsValuePair = < obs factor index, obs value >

    ObservationLayer();

    void eventUpdatesCallback ( const predicate_manager::EventUpdateConstPtr& msg );

    void eventMapCallback ( const predicate_manager::EventInfoMapConstPtr& msg );

    void observationMetadataCallback ( const mdm_library::FactoredSymbolMetadataConstPtr& msg );

    void addObservationFactor ( ObservationDep deps );

private:

    void publishJointObservation();

    void updateWorldSymbol ( const uint32_t& factor );

    ros::NodeHandle nh_;

    ros::Subscriber event_map_sub_;
    ros::Subscriber event_update_sub_;

    ros::Subscriber observation_metadata_sub_;
    ros::Publisher observation_pub_;

    ///Each event may be associated to a single dependent factor
    boost::unordered_map< predicate_manager::NameID, ObsValuePair > event_named_local_observer_;
    ///Observations can also be triggered by global events (i.e. from any agent). In that case, they don't have a fixed NameID.
    std::map< std::string, ObsValuePair > event_named_global_observer_;

    boost::unordered_map< predicate_manager::NrID,
          ObsValuePair,
          predicate_manager::cantor_pair_hash > event_nr_observer_;

    std::vector< ObservationDep > factored_observations_deps_;
    std::vector< uint32_t > factored_observations_;
};
}

#endif
