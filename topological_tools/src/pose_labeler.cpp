/**\file pose_labeler.cpp
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


#include <topological_tools/pose_labeler.h>



using namespace topological_tools;



PoseLabeler::PoseLabeler (const std::string& label_map_filename) :
  label_map_ (IMG_Load (label_map_filename.c_str())),
  map_loaded_ (false)
{
  if (label_map_ == 0) {
    ROS_ERROR_STREAM ("Unable to open label map file " << label_map_filename);
    abort();
  }
}



uint32_t PoseLabeler::getPixelValue (boost::shared_ptr<SDL_Surface> surface, int x, int y)
{
  //NOTE: From SDL Wiki (http://www.libsdl.org/cgi/docwiki.cgi/Pixel_Access as of 13/12/12)
  int bpp = surface->format->BytesPerPixel;
  Uint8* p = (Uint8*) surface->pixels + y * surface->pitch + x * bpp;
  
  switch (bpp) {
    case 1:
      return *p;
      break;
      
    case 2:
      return * (Uint16*) p;
      break;
      
    case 3:
      if (SDL_BYTEORDER == SDL_BIG_ENDIAN) {
        return p[0] << 16 | p[1] << 8 | p[2];
      }
      else {
        return p[0] | p[1] << 8 | p[2] << 16;
      }
      break;
      
    case 4:
      return * (Uint32*) p;
      break;
      
    default:
      return 0;
  }
}



PoseLabeler::~PoseLabeler()
{
  //SDL_FreeSurface (label_map_.get());
}



void PoseLabeler::setMapMetaData (const nav_msgs::MapMetaDataConstPtr& mdata)
{
  setMapMetaData (*mdata);
}



void PoseLabeler::setMapMetaData (nav_msgs::MapMetaData mdata)
{
  map_metadata_ = mdata;
  map_loaded_ = true;
}


PoseLabel PoseLabeler::getPoseLabel (geometry_msgs::PoseWithCovarianceStamped msg)
{
  if (!map_loaded_) {
    ROS_WARN ("Label not calculated - Metadata not available");
    return PoseLabel();
  }
  
  ROS_DEBUG_STREAM_NAMED ("pose_label", "received_position (x,y) = " << msg.pose.pose.position.x << " , " << msg.pose.pose.position.y);
  //This is simply a coordinate transform from the world frame to map pixel coordinates.
  //TF isn't used as the output coordinate frame is adimensional and shouldn't be accessed by anything else.
  float x0 = map_metadata_.origin.position.x;
  float y0 = map_metadata_.origin.position.y;
  uint32_t px, py;
  if (map_metadata_.origin.orientation.z != 0) {
    double theta = acos (map_metadata_.origin.orientation.w);
    px = floor ( (-x0 + cos (theta) * msg.pose.pose.position.x - sin (theta) * msg.pose.pose.position.y) / map_metadata_.resolution);
    py = label_map_->h - floor ( (-y0 + sin (theta) * msg.pose.pose.position.x + cos (theta) * msg.pose.pose.position.y) / map_metadata_.resolution);
  }
  else {
    //This is the most common case. It saves a little time on the cos and sin calculations.
    px = floor ( (msg.pose.pose.position.x - x0) / map_metadata_.resolution);
    ROS_DEBUG_STREAM_NAMED ("pose_label", "label_map_.h: " << label_map_->h << ", y: " << msg.pose.pose.position.y << ", y0: " << y0 << ", res:" << map_metadata_.resolution << ", Map HxW: " << map_metadata_.height << " , " << map_metadata_.width);
    py = label_map_->h - floor ( (msg.pose.pose.position.y - y0) / map_metadata_.resolution);
  }
  //   ROS_DEBUG_STREAM_NAMED ("pose_label", "(x,y) = " << px << " , " << py << ", Map HxW: " << map_metadata_.height << " , " <<map_metadata_.width);
  
  if (px <= map_metadata_.width &&
      py <= map_metadata_.height) {
    PoseLabel pl;
    uint32_t label = getPixelValue (label_map_, px, py);
    pl.label = label;
    ROS_DEBUG_STREAM_NAMED ("pose_label", "(x,y) = " << px << " , " << py << ". Label: " << label);
    return pl;
  }
  else {
    ROS_WARN_STREAM ("Label not calculated - Position out of bounds. Check the map metadata origin/resolution");
    return PoseLabel();
  }
}
