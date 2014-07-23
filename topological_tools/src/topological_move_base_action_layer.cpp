/**\file topological_move_base_action_layer.cpp
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

#include <topological_tools/topological_move_base_action_layer.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace topological_tools;
using namespace predicate_manager;
using namespace mdm_library;

TopologicalMoveBaseActionLayer::
TopologicalMoveBaseActionLayer ( TopologicalMap& tm ) :
    actions_cb_queue_(),
    predicates_cb_queue_(),
    al_ ( &actions_cb_queue_ ),
    tam_ ( tm, &predicates_cb_queue_ ),
    move_base_client_ ( "move_base", true ), ///auto spin is true
    client_ ( nh_.serviceClient<std_srvs::Empty> ( "publish_new_action" ) )
{
    ros::service::waitForService ( "republish_service", 5 );
    
    ros::Duration d ( 10.0 );
    d.sleep(); ///TODO: This Action Layer should wait for the PM to come up as well. This can be removed when the predicate update service is implemented.
    while ( ! ( move_base_client_.waitForServer ( d ) ) )
    {
        ROS_WARN_STREAM ( "TopologicalMoveBaseActionLayer:: Waiting for the move_base server to come up." );
    };
}

TopologicalMoveBaseActionLayer::
TopologicalMoveBaseActionLayer ( const std::string& map_file ) :
    actions_cb_queue_(),
    predicates_cb_queue_(),
    al_ ( &actions_cb_queue_ ),
    tam_ ( map_file, &predicates_cb_queue_ ),
    move_base_client_ ( "move_base", true ), ///auto spin is true
    client_ ( nh_.serviceClient<std_srvs::Empty> ( "publish_new_action" ) )
{
    ros::service::waitForService ( "publish_new_action", 5 );
    
    ros::Duration d ( 10.0 );
    d.sleep(); ///TODO: This Action Layer should wait for the PM to come up as well. This can be removed when the predicate update service is implemented.
    while ( ! ( move_base_client_.waitForServer ( d ) ) )
    {
        ROS_WARN_STREAM ( "TopologicalMoveBaseActionLayer:: Waiting for the move_base server to come up." );
    };
}

void
TopologicalMoveBaseActionLayer::
addAction ( const std::string& action_name )
{
    al_.addAction ( boost::bind ( &TopologicalMoveBaseActionLayer::moveToLabel, this, action_name ),
                    action_name );
}



mdm_library::ActionLayer
TopologicalMoveBaseActionLayer::
getActionLayer ()
{
    return al_;
}


void
TopologicalMoveBaseActionLayer::
moveToLabel ( const string& connection_label )
{
    geometry_msgs::PoseStamped goal_pose;
    
    try
    {
        goal_pose.pose =  tam_.getGoalPoseForLabel ( connection_label );
    }
    catch ( char const* a )
    {
        if ( strcmp ( a, "republish" ) == 0 )
        {
            std_srvs::Empty request;
            
            cout << "Action impossible to realize. Generating new action." << endl;
            
            move_base_client_.cancelAllGoals();
            
            client_.call ( request );
            
            return;
        }
    }
    
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "/map";

    move_base_msgs::MoveBaseGoal traverse_goal;
    traverse_goal.target_pose = goal_pose;
    
    //move_base_client_.cancelAllGoals();
    move_base_client_.sendGoal ( traverse_goal,
                                 boost::bind ( &TopologicalMoveBaseActionLayer::moveBaseDoneCB, this, _1, _2 ),
                                 boost::bind ( &TopologicalMoveBaseActionLayer::moveBaseActiveCB, this ),
                                 boost::bind ( &TopologicalMoveBaseActionLayer::moveBaseFeedbackCB, this, _1 ) );
    ///We can't wait the for result because this is single-threaded, and we might want to change the goal before the task finishes.
}

void
TopologicalMoveBaseActionLayer::
spin ()
{
    using namespace ros;
    Rate r ( 10 ); // TODO: configuravel
    while ( ok() )
    {
        predicates_cb_queue_.callAvailable();
        actions_cb_queue_.callAvailable();
        r.sleep();
    }
}
