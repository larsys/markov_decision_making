/**\file topological_map.h
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

#ifndef _TOPOLOGICAL_MAP_H_
#define _TOPOLOGICAL_MAP_H_

#include <topological_tools/topological_node.h>



namespace topological_tools
{
  class TopologicalMap
  {
    public:
      TopologicalMap (const std::string& map_file);
      
      bool hasNode (const std::string& node_name);
      
      boost::shared_ptr<TopologicalNode> getNodeByName (const std::string& node_name);
      
      void addNode (geometry_msgs::Pose goal, const std::string& node_name);
      
    private :
    
      std::map<std::string, boost::shared_ptr<TopologicalNode> > node_map_;
  };
}

#endif
