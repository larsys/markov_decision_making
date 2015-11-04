/**\file pose_labeler.h
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


#ifndef _POSE_LABELER_H_
#define _POSE_LABELER_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stdlib.h>

#include <SDL/SDL_image.h>
#include <mdm_topological_tools/PoseLabel.h>




namespace mdm_topological_tools
{
class PoseLabeler
{
public:
    PoseLabeler ( const std::string& label_map_filename );

    ~PoseLabeler();
    PoseLabel getPoseLabel ( geometry_msgs::PoseWithCovarianceStamped msg );
    uint32_t getPixelValue ( boost::shared_ptr<SDL_Surface> surface, int x, int y );

    void setMapMetaData ( const nav_msgs::MapMetaDataConstPtr& mdata );

    void setMapMetaData ( nav_msgs::MapMetaData mdata );

private:

    boost::shared_ptr<SDL_Surface> label_map_;

    bool map_loaded_;

    nav_msgs::MapMetaData map_metadata_;
};
}

#endif
