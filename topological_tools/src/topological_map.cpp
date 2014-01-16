/**\file topological_map.cpp
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

#include <topological_tools/topological_map.h>

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH



using namespace std;
using namespace topological_tools;
using namespace geometry_msgs;



TopologicalMap::TopologicalMap ( const string& map_file )
{
    using boost::property_tree::ptree;

    ptree p;
    try
    {
        read_xml ( map_file, p );

        foreach ( ptree::value_type & n, p.get_child ( "TopologicalMap" ) )
        {
            string name = n.second.get<string> ( "<xmlattr>.name" );
            string data = n.second.get<string> ( "Goal" );
            Pose goal;

            float x, y, th;
            sscanf ( data.c_str(), "%f %f %f", &x, &y, &th );
            goal.position.x = x;
            goal.position.y = y;
            goal.orientation.z = sin ( th / 2.0 );
            goal.orientation.w = cos ( th / 2.0 );
            addNode ( goal, name );
        }
        //connections
        foreach ( ptree::value_type & n, p.get_child ( "TopologicalMap" ) )
        {
            string name = n.second.get<string> ( "<xmlattr>.name" );

            foreach ( ptree::value_type & c, n.second.get_child ( "Connections" ) )
            {
                string connection_label = c.second.get<string> ( "<xmlattr>.label" );
                string connection_name = c.second.data();
                node_map_[name]->connect ( getNodeByName ( connection_name ), connection_label );
            }
        }
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( "TopologicalMap:: Exception Thrown: " << e.what() );
    }
}



bool TopologicalMap::hasNode ( const string& node_name )
{
    return node_map_.count ( node_name );
}



boost::shared_ptr<TopologicalNode> TopologicalMap::getNodeByName ( const string& node_name )
{
    if ( !node_map_.count ( node_name ) )
    {
        ROS_ERROR_STREAM ( "TopologicalMap:: Node " << node_name << " not present in map." );
        return boost::shared_ptr<TopologicalNode>();
    }

    return node_map_[node_name];
}



void TopologicalMap::addNode ( Pose goal, const string& node_name )
{
    if ( node_map_.count ( node_name ) )
    {
        ROS_WARN_STREAM ( "TopologicalMap:: Node " << node_name << " already present in map. It is being replaced." );
    }


    node_map_[node_name] = boost::shared_ptr<TopologicalNode> ( new TopologicalNode ( goal, node_name ) );
}
