/**\file dependencies.h
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * Predicate Manager is a ROS library to define and manage logical predicates and events.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of Predicate Manager.
 *
 * Predicate Manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Predicate Manager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _DEPENDENCIES_H_
#define _DEPENDENCIES_H_

#include <string>

#include <ros/ros.h>

#include <predicate_manager/common_defs.h>

namespace predicate_manager
{
  /**
   * The Dependencies class allows a user to specify the dependencies of a Predicate
   * through a chained declaration of the dependency labels.
   * 
   * For example, if Predicate "A", depends on Predicates "B","C","D", and if the "MyPredicate"
   * class implements the desired condition, you could declare
   * it as MyPredicate("A",Dependencies().add("B").add("C").add("D"));
   */
  class Dependencies
  {
    public:
      ///Default constructor.
      Dependencies();
      
      ///Specifies a dependency through its name.
      Dependencies add (const std::string& name);
      /**
       * Specifies a dependency through its parent PM and its name.
       * You can use this form to add external dependencies (dependencies on Predicates
       * whose implementation is not known to this agent).
       */
      Dependencies add (const int pm_id, const std::string& name);
      ///Specifies a dependency through a NameID. Equivalent to "add (const int pm_id, const std::string& name)".
      Dependencies add (const NameID& dep_id);

      NameIDSet dependency_set_; ///The set of dependencies, to be passed onto the constructor of a Predicate.
      
    private:
      ros::NodeHandle nh_; ///To access the "pm_id" parameter.
      
      int pm_id_; ///This stores the ID of the local PM to avoid unnecessary queries to the parameter server.
  };
}

#endif
