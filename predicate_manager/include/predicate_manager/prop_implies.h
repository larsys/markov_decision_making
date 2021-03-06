/**\file prop_implies.h
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

#ifndef _PROP_IMPLIES_H_
#define _PROP_IMPLIES_H_

#include <string>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <predicate_manager/prop_operator.h>



namespace predicate_manager
{
/**
 * Implements the logical implication operator (A -> B) in propositional formulas.
 */
class Implies : public PropOperator
{
public:
    /**
     * Default constructor. Implements the condition p1 -> p2.
     */
    Implies ( const PropLogic& p1,
              const PropLogic& p2 ) :
        PropOperator ( p1, p2 )
    {}

    ///Evaluates this PropOperator.
    bool evaluate ( boost::function<bool ( NameID ) > f ) const;

    ///A standard clone function for PropLogic and its derived classes.
    boost::shared_ptr<PropLogic> clone() const;
};
}

#endif
