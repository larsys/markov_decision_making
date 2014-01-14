/**\file pose_labeler_node.cpp
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


#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <topological_tools/pose_labeler.h>
#include <topological_tools/PoseLabel.h>

#include <stdlib.h>

#include <SDL/SDL_image.h>



namespace topological_tools
{
  class PoseLabelerWrapper : public PoseLabeler
  {
    public:
      PoseLabelerWrapper (const std::string& label_map_filename,
                          const std::string& pose_topic) :
        PoseLabeler (label_map_filename),
        map_meta_sub_ (nh_.subscribe ("/map_metadata", 1, &PoseLabelerWrapper::mapMetadataCallback, this)),
        pose_sub_ (nh_.subscribe (pose_topic, 1, &PoseLabelerWrapper::poseCallback, this)),
        label_pub_ (nh_.advertise<PoseLabel> ("pose_label", 1, true)),
        last_label_ (),
        started_ (false) {}
        
      void mapMetadataCallback (const nav_msgs::MapMetaDataConstPtr& msg) {
        setMapMetaData (msg);
      }
      
      void poseCallback (const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        PoseLabel currentLabel = getPoseLabel (*msg);
        if (!started_ || (currentLabel.label != last_label_.label)) {
          label_pub_.publish (currentLabel);
          started_ = true;
          last_label_.label = currentLabel.label;
        }
      }
      
    private:
      ros::NodeHandle nh_;
      
      ros::Subscriber map_meta_sub_;
      ros::Subscriber pose_sub_;
      
      ros::Publisher label_pub_;
      PoseLabel last_label_;
      bool started_;
  };
}



int main (int argc, char** argv)
{
  ros::init (argc, argv, "pose_labeler_node");
  
  if (argc < 3) {
    ROS_ERROR ("\nUSAGE: pose_labeler_node <path to label map> <pose topic>\n");
  }
  else {
    topological_tools::PoseLabelerWrapper plw (argv[1], argv[2]);
    ros::spin();
  }
  
  return 0;
}
