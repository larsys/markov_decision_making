/**\file topological_node.cpp
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


#include <mdm_topological_tools/topological_node.h>



using namespace std;
using namespace ros;
using namespace mdm_topological_tools;



TopologicalNode::TopologicalNode ( const geometry_msgs::Pose& goal, const string& name ) :
    goal_ ( goal ),
    name_ ( name )
{}



void TopologicalNode::connect ( boost::shared_ptr<TopologicalNode> tpn, const string& connection_label )
{
    connections_[connection_label] = tpn;
}



boost::shared_ptr<TopologicalNode> TopologicalNode::getConnection ( const string& connection_label )
{
    if ( !connections_.count ( connection_label ) )
    {
        ROS_ERROR_STREAM ( "TopologicalNode:: Node " << name_ << " has no connection '" << connection_label << "'." );
        abort();
    }

    return connections_[connection_label];
}



bool TopologicalNode::hasConnection ( const string& connection_label )
{
    return connections_.count ( connection_label );
}



const geometry_msgs::Pose& TopologicalNode::getGoalPose()
{
    return goal_;
}



const string& TopologicalNode::getName()
{
    return name_;
}
