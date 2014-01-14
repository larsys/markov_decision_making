/**\file dependencies.cpp
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

#include <predicate_manager/dependencies.h>


using namespace predicate_manager;

Dependencies::
Dependencies() :
  nh_ (),
  pm_id_ (0)
{
  nh_.getParam ("pm_id", (int&) pm_id_);
}
 

Dependencies
Dependencies::
add (const std::string& name) 
{
  return add(make_pair(pm_id_, name));
}


Dependencies
Dependencies::
add (const int pm_id, const std::string& name) 
{
  return add(make_pair(pm_id, name));
}
   
  
Dependencies
Dependencies::
add (const NameID& pred_nid)
{
  dependency_set_.insert(pred_nid);
  return *this;
}