/**\file topological_predicate.h
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

#ifndef _TOPOLOGICAL_PREDICATE_H_
#define _TOPOLOGICAL_PREDICATE_H_

#include <ros/ros.h>
#include <predicate_manager/predicate.h>
#include <topological_tools/PoseLabel.h>

namespace topological_tools
{
class TopologicalPredicate : public predicate_manager::Predicate
{
public:
    TopologicalPredicate ( const std::string& label_topic,
                           const std::string& name );

    TopologicalPredicate ( const std::string& label_topic,
                           const std::string& name,
                           const uint32_t target_label );


    void labelCallback ( const topological_tools::PoseLabelConstPtr& msg );

    void update();

protected:
    ros::NodeHandle nh_;

private:
    ros::Subscriber label_subs_;

    uint32_t target_label_;
    uint32_t received_label_;
};
}

#endif
