/**\file prop_xor.cpp
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

#include <predicate_manager/prop_xor.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH



using namespace predicate_manager;
using namespace std;



bool
Xor::
evaluate (boost::function<bool (NameID) > f) const
{
  if(children_.empty())
    return false;
  
  bool base_value = children_[0]->evaluate (f);
  for(size_t i = 1; i < children_.size(); i++)
  {
    if(children_[i]->evaluate(f) != base_value)
      return false;
  }
  
  return true;
}



boost::shared_ptr<PropLogic>
Xor::
clone() const
{
  boost::shared_ptr<Xor> p (new Xor (*this));
  return (boost::dynamic_pointer_cast<PropLogic> (p));
}
