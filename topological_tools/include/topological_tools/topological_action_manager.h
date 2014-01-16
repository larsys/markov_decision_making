/**\file topological_action_manager.h
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

#ifndef _TOPOLOGICAL_ACTION_MANAGER_H_
#define _TOPOLOGICAL_ACTION_MANAGER_H_

#include <ros/ros.h>
#include <ros/callback_queue_interface.h>
#include <geometry_msgs/PoseStamped.h>

#include <predicate_manager/PredicateInfoMap.h>
#include <predicate_manager/PredicateUpdate.h>
#include <topological_tools/topological_map.h>



namespace topological_tools
{
class TopologicalActionManager
{
public:
    TopologicalActionManager ( const std::string& map_file, ros::CallbackQueueInterface* queue = NULL );
    TopologicalActionManager ( TopologicalMap& tm, ros::CallbackQueueInterface* queue = NULL );

    boost::shared_ptr<TopologicalNode> getCurrentNode();
    boost::shared_ptr<TopologicalNode> getGoalNodeForLabel ( const std::string& connection_label );
    boost::shared_ptr<TopologicalNode> getGoalNodeForLabel ( const std::string& connection_label,
            const std::string& origin_name );
    geometry_msgs::Pose getGoalPoseForLabel ( const std::string& connection_label );
    geometry_msgs::Pose getGoalPoseForLabel ( const std::string& connection_label,
            const std::string& origin_name );
    bool isInitialized();
private:
    void predicateMapCallback ( const predicate_manager::PredicateInfoMapConstPtr& msg );
    void predicateUpdatesCallback ( const predicate_manager::PredicateUpdateConstPtr& msg );
    void predicateUpdatesInternal ( const predicate_manager::PredicateUpdate& msg );

    ros::NodeHandle nh_;

    ros::Subscriber pred_updates_sub_;
    ros::Subscriber pred_map_sub_;

    TopologicalMap tm_;

    std::map<uint32_t, std::string> pred_id_node_map_;

    int pm_id_; ///Agent ID as seen by the Predicate Manager
    std::string current_node_name_;
    ///For initialization purposes:
    predicate_manager::PredicateUpdate requested_update_;
};
}

#endif
