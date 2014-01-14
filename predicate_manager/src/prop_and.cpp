/**\file prop_and.cpp
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

#include <predicate_manager/prop_and.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH



using namespace predicate_manager;
using namespace std;



bool
And::
evaluate (boost::function<bool (NameID) > f) const
{
  bool v = true;
  foreach (children_type::value_type c, children_)
  v = v && c->evaluate (f);
  
  return v;
}



boost::shared_ptr<PropLogic>
And::
clone() const
{
  boost::shared_ptr<And> p (new And (*this));
  return (boost::dynamic_pointer_cast<PropLogic> (p));
}
