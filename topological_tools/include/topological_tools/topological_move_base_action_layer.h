/**\file topological_move_base_action_layer.h
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

#ifndef _TOPOLOGICAL_MOVE_BASE_ACTION_LAYER_H_
#define _TOPOLOGICAL_MOVE_BASE_ACTION_LAYER_H_

#include <string.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>

#include <topological_tools/topological_action_manager.h>
#include <mdm_library/action_layer.h>


namespace topological_tools
{
/**
 * Assumes that the map frame id is at /map (TODO: configurable)
 */
class TopologicalMoveBaseActionLayer
{
public:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    TopologicalMoveBaseActionLayer ( const std::string& map_file );
    TopologicalMoveBaseActionLayer ( TopologicalMap& tm );

    void addAction ( const std::string& action_name );
    
    mdm_library::ActionLayer getActionLayer ();

    void spin();
protected:
    virtual void moveBaseDoneCB ( const actionlib::SimpleClientGoalState& state,
                                  const move_base_msgs::MoveBaseResultConstPtr& result ) {};
    virtual void moveBaseActiveCB () {};
    virtual void moveBaseFeedbackCB ( const move_base_msgs::MoveBaseFeedbackConstPtr& feedback ) {};

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    
    void moveToLabel ( const std::string& connection_label );

    ros::CallbackQueue actions_cb_queue_;
    ros::CallbackQueue predicates_cb_queue_;

    mdm_library::ActionLayer al_;
    TopologicalActionManager tam_;
    MoveBaseClient move_base_client_;
};
}

#endif
