/**\file prop_operator.h
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

#ifndef _PROP_OPERATOR_H_
#define _PROP_OPERATOR_H_

#include <string>
#include <set>
#include <vector>

#include <predicate_manager/prop_logic.h>

namespace predicate_manager
{
class PropOperator : public PropLogic
{
public:
    ///The type for the container of a propositional connective's variables.
    typedef std::vector<boost::shared_ptr<const PropLogic> > children_type;

    ///Default constructor. Currently, only binary propositional operators are supported.
    PropOperator ( const PropLogic& p1,
                   const PropLogic& p2 );

    ///Gets the set of variable names for this propositional formula.
    void printVariables ( NameIDSet& dep_set ) const;

protected:
    children_type children_; ///The container of a propositional connective's variables.
};
}

#endif
