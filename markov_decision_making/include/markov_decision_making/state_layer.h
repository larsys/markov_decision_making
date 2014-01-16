/**\file state_layer.h
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

#include <predicate_manager/predicate_manager.h>
#include <predicate_manager/PredicateInfoMap.h>
#include <predicate_manager/PredicateUpdate.h>

#include <markov_decision_making/state_dep.h>



namespace markov_decision_making
{
class StateLayer
{
public:
    typedef boost::shared_ptr<std::set<uint32_t> > SFIDSetPtr;

    StateLayer();

    void predicateUpdatesCallback ( const predicate_manager::PredicateUpdateConstPtr& msg );

    void predicateMapCallback ( const predicate_manager::PredicateInfoMapConstPtr& msg );

    void addStateFactor ( const StateDep::SFDeps& deps );
    void addStateFactor ( const StateDep& deps );
private:
    void publishJointState();
    std::string factorString ( unsigned k );
    void updateStateInfo ( const uint32_t& factor );

    ros::NodeHandle nh_;
    ros::Subscriber pred_map_sub_;
    ros::Subscriber pred_update_sub_;
    ros::Subscriber state_metadata_sub_;
    ros::Publisher state_pub_;

    boost::unordered_map<predicate_manager::NrID,
          boost::shared_ptr<bool>,
          predicate_manager::cantor_pair_hash> nr_value_map_;
    boost::unordered_map<predicate_manager::NameID, boost::shared_ptr<bool> > named_value_map_;

    //Each pair <Agent ID, Predicate ID> may be associated to a set of dependent factors
    boost::unordered_map<predicate_manager::NrID,
          SFIDSetPtr,
          predicate_manager::cantor_pair_hash> nr_pred_observer_;
    boost::unordered_map<predicate_manager::NameID, SFIDSetPtr > named_pred_observer_;

    std::vector<StateDep::SFDeps> factored_state_deps_;

    std::vector<uint32_t> factored_state_; //in state factors
};
}

#endif
