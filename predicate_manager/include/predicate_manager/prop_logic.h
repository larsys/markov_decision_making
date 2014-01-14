/**\file prop_logic.h
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

#ifndef _PROPLOGIC_H_
#define _PROPLOGIC_H_

#include <set>
#include <string>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <predicate_manager/common_defs.h>

namespace predicate_manager
{
  /**
   * The PropLogic class is a base class for propositional logic operators (connectives).
   */
  class PropLogic
  {
    public:
      ///Evaluates this propositional formula. This should be implemented by each derived operator.
      virtual bool evaluate (boost::function<bool (NameID) > f) const = 0;

      ///A standard clone function for PropLogic and its derived classes.
      virtual boost::shared_ptr<PropLogic> clone() const = 0;
      
      ///Gets the set of variable names for this propositional formula.
      virtual void printVariables (NameIDSet& dep_set) const = 0;
  };
}

#endif
