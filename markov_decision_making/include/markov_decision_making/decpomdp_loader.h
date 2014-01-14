/**\file decpomdp_loader.h
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * Markov Decision Making is a ROS library for robot decision-making based on MDPs.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of Markov Decision Making.
 *
 * Markov Decision Making is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Markov Decision Making is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _DECPOMDP_LOADER_H_
#define _DECPOMDP_LOADER_H_

#include <string>

#include <ros/ros.h>



#ifdef HAVE_MADP

#include <madp/PlanningUnitDecPOMDPDiscrete.h>
#include <madp/FactoredDecPOMDPDiscrete.h>
#include <madp/DecPOMDPDiscrete.h>

#endif



namespace markov_decision_making
{
  class DecPOMDPLoader
  {
    protected:
      ros::NodeHandle nh_;
      
    public:
      DecPOMDPLoader (const std::string& problem_file);
      
#ifdef HAVE_MADP
      
      boost::shared_ptr<DecPOMDPDiscreteInterface> decpomdp_;
      
      const boost::shared_ptr<DecPOMDPDiscreteInterface> GetDecPOMDP();
      PlanningUnitMADPDiscreteParameters GetParams();
      
      void publishActionMetadata ();
      void publishStateMetadata (boost::shared_ptr<FactoredDecPOMDPDiscrete> f);
      void publishStateMetadata (boost::shared_ptr<DecPOMDPDiscrete> d);
      void publishObservationMetadata ();
      void publishInitialStateDistribution (boost::shared_ptr<FactoredDecPOMDPDiscrete> f);
      void publishInitialStateDistribution (boost::shared_ptr<DecPOMDPDiscrete> d);
    private:
      ros::Publisher action_metadata_pub_;
      ros::Publisher state_metadata_pub_;
      ros::Publisher observation_metadata_pub_;
      ros::Publisher initial_state_distribution_pub_;
      
#endif
  };
}

#endif
