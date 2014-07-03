/**\file topological_action_manager.cpp
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * TopologicalTools is a set of utilities to aid the deployment of the MDM library
 * in topological navigation problems.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of TopologicalTools.
 *
 * TopologicalTools is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * TopologicalTools is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <boost/function.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <geometry_msgs/PoseStamped.h>

#include <topological_tools/topological_action_manager.h>
#include <predicate_manager/predicate_manager.h>



using namespace std;
using namespace ros;
using namespace topological_tools;
using namespace predicate_manager;



TopologicalActionManager::
TopologicalActionManager ( TopologicalMap& tm, CallbackQueueInterface* queue ) :
    pred_updates_sub_ (),
    pred_map_sub_ (),
    tm_ ( tm ),
    pred_id_node_map_(),
    pm_id_ ( 0 ),
    current_node_name_(),
    requested_update_()
{
    SubscribeOptions update_opts = SubscribeOptions::create<PredicateUpdate>
                                   ( "/predicate_updates", 1, boost::bind ( &TopologicalActionManager::predicateUpdatesCallback, this,_1 ),VoidPtr(), queue );
    SubscribeOptions map_opts = SubscribeOptions::create<PredicateInfoMap>
                                ( "/predicate_maps", 1, boost::bind ( &TopologicalActionManager::predicateMapCallback, this,_1 ),VoidPtr(), queue );

    pred_updates_sub_ = nh_.subscribe ( update_opts );
    pred_map_sub_ = nh_.subscribe ( map_opts );
    nh_.getParam ( "pm_id", pm_id_ );
}



TopologicalActionManager::
TopologicalActionManager ( const std::string& map_file, CallbackQueueInterface* queue ) :
    pred_updates_sub_ (),
    pred_map_sub_ (),
    tm_ ( map_file ),
    pred_id_node_map_(),
    pm_id_ ( 0 ),
    current_node_name_(),
    requested_update_()
{
    SubscribeOptions update_opts = SubscribeOptions::create<PredicateUpdate>
                                   ( "/predicate_updates", 1, boost::bind ( &TopologicalActionManager::predicateUpdatesCallback, this,_1 ),VoidPtr(), queue );
    SubscribeOptions map_opts = SubscribeOptions::create<PredicateInfoMap>
                                ( "/predicate_maps", 1, boost::bind ( &TopologicalActionManager::predicateMapCallback, this,_1 ),VoidPtr(), queue );

    pred_updates_sub_ = nh_.subscribe ( update_opts );
    pred_map_sub_ = nh_.subscribe ( map_opts );
    nh_.getParam ( "pm_id", pm_id_ );
}



geometry_msgs::Pose
TopologicalActionManager::
getGoalPoseForLabel ( const std::string& connection_label )
{
    if ( !isInitialized() )
    {
        ROS_ERROR ( "TopologicalActionManager:: Not yet initialized. Current node unknown." );

        return geometry_msgs::Pose();
    }

    return getGoalPoseForLabel ( connection_label, current_node_name_ );
}



geometry_msgs::Pose
TopologicalActionManager::
getGoalPoseForLabel ( const std::string& connection_label, const std::string& origin_name )
{
    geometry_msgs::Pose goal_pose;
    boost::shared_ptr<TopologicalNode> goal_node = getGoalNodeForLabel ( connection_label, origin_name );
    if ( goal_node )
    {
        goal_pose = goal_node->getGoalPose();
    }
    else
    {
        ROS_ERROR_STREAM ( "TopologicalActionManager:: Could not get the goal node for the label pair < " << origin_name << ", " << connection_label << " >" );
    }
    return goal_pose;
}



boost::shared_ptr<TopologicalNode>
TopologicalActionManager::
getGoalNodeForLabel ( const std::string& connection_label )
{
    if ( !isInitialized() )
    {
        ROS_ERROR ( "TopologicalActionManager:: Not yet initialized. Current node unknown." );
        return boost::shared_ptr<TopologicalNode>();
    }

    return getGoalNodeForLabel ( connection_label, current_node_name_ );
}



boost::shared_ptr<TopologicalNode>
TopologicalActionManager::
getGoalNodeForLabel ( const std::string& connection_label, const std::string& origin_name )
{
    boost::shared_ptr<TopologicalNode> node = tm_.getNodeByName ( origin_name );
    if ( !node )
        return node;
    if ( !node->hasConnection ( connection_label ) )
    {
        ROS_WARN_STREAM ( "TopologicalActionManager:: Node '" << node->getName() << "' has no connection '" << connection_label << "'. Idling" );

        throw "republish";
        
        return node;
    }
    else
    {
        return node->getConnection ( connection_label );
    }
}



boost::shared_ptr<TopologicalNode>
TopologicalActionManager::
getCurrentNode()
{
    if ( !isInitialized() )
    {
        ROS_ERROR ( "TopologicalActionManager:: Not yet initialized. Current node unknown." );
        return boost::shared_ptr<TopologicalNode>();
    }

    return tm_.getNodeByName ( current_node_name_ );
}



bool
TopologicalActionManager::
isInitialized()
{
    return !current_node_name_.empty();
}



void
TopologicalActionManager::
predicateMapCallback ( const PredicateInfoMapConstPtr& msg )
{
    if ( msg->pm_id != ( uint32_t ) pm_id_ )
    {
        return;
    }

    foreach ( PredicateInfoMap::_map_type::value_type pred_info, msg->map )
        if ( tm_.hasNode ( pred_info.name ) )
        {
            pred_id_node_map_[pred_info.nr] = pred_info.name;
        }

    if ( !requested_update_.true_predicates.empty() )
    {
        predicateUpdatesInternal ( requested_update_ );
    }
}



void
TopologicalActionManager::
predicateUpdatesCallback ( const PredicateUpdateConstPtr& msg )
{
    if ( msg->pm_id != ( uint32_t ) pm_id_ )
    {
        return;
    }

    if ( pred_id_node_map_.empty() )
    {
        ///Got an update before the map
        requested_update_ = *msg;
        return;
    }
    else
    {
        requested_update_.true_predicates.clear();
    }

    predicateUpdatesInternal ( *msg );
}



void
TopologicalActionManager::
predicateUpdatesInternal ( const PredicateUpdate& msg )
{
    foreach ( PredicateUpdate::_true_predicates_type::value_type pred, msg.true_predicates )
        if ( pred_id_node_map_.count ( pred ) )
        {
            current_node_name_ = pred_id_node_map_[pred];
            ROS_INFO_STREAM ( "TopologicalActionManager:: Current node is now " << current_node_name_ );
        }
}
