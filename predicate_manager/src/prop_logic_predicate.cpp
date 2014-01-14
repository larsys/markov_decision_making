/**\file prop_logic_predicate.cpp
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

#include <predicate_manager/prop_logic_predicate.h>

#include <ros/ros.h>

using namespace std;
using namespace predicate_manager;


PropLogicPredicate::
PropLogicPredicate(const string& name,
                   const PropLogic& pl) :
  Predicate (name),
  prop_logic_ (pl.clone())
{
  declareAllDependencies();
}

PropLogicPredicate::
PropLogicPredicate(const string& name) :
  Predicate (name),
  prop_logic_ ()
{}

void 
PropLogicPredicate::
bindPropLogic(const PropLogic& pl)
{
  if(prop_logic_ != 0)
  {
    ROS_ERROR_STREAM("Overwriting the Propositional Logic formula associated with predicate " << getName());
  }
  prop_logic_ = pl.clone();
  declareAllDependencies();
  
  update();
}

void 
PropLogicPredicate::
update()
{
  if(prop_logic_ != 0)
  {
    bool val = prop_logic_->evaluate(boost::bind(&PropLogicPredicate::evaluate,this,_1));
    setValue(val);
  }
}

void
PropLogicPredicate::
declareAllDependencies()
{
  if(prop_logic_!=0)
  {
    NameIDSet dep_set;
    prop_logic_->printVariables(dep_set);
    foreach(NameID dep_name_id, dep_set)
    {
      declareDependency(dep_name_id);
    }
  }
}

bool 
PropLogicPredicate::
evaluate(const NameID& dep_name_id)
{
  return getDependencyValue(dep_name_id.second, dep_name_id.first);
}