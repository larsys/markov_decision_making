/**\file topological_node.h
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

#ifndef _TOPOLOGICAL_NODE_H_
#define _TOPOLOGICAL_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <boost/shared_ptr.hpp>



namespace topological_tools
{
class TopologicalNode
{
public:
    TopologicalNode ( const geometry_msgs::Pose& goal, const std::string& name );

    void connect ( boost::shared_ptr<TopologicalNode> tpn, const std::string& connection_label );
    boost::shared_ptr<TopologicalNode> getConnection ( const std::string& connection_label );

    bool hasConnection ( const std::string& connection_label );
    const geometry_msgs::Pose& getGoalPose();
    const std::string& getName();

private:
    geometry_msgs::Pose goal_;
    std::string name_;

    std::map<std::string, boost::shared_ptr<TopologicalNode> > connections_;
};
}

#endif
