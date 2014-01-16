/**\file prop_not.h
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

#ifndef _PROP_NOT_H_
#define _PROP_NOT_H_

#include <string>
#include <set>

#include <boost/function.hpp>

#include <predicate_manager/prop_logic.h>



namespace predicate_manager
{
/**
 * Implements the logical negation operator. "Not" is one of the two unary
 * operators that directly reference Predicate instances (the other being PV).
 */
class Not : public PropLogic
{
public:
    /**
     * Default constructor.
     * @param name The name of the Predicate to which this negation operator should refer.
     */
    Not ( const std::string& name, const uint32_t pm_id = 0 );

    ///Evaluates this formula.
    bool evaluate ( boost::function<bool ( NameID ) > f ) const;

    ///A standard clone function for PropLogic and its derived classes.
    boost::shared_ptr<PropLogic> clone() const;

    ///Gets the set of variable names for this propositional formula.
    void printVariables ( NameIDSet& dep_set ) const;

private:
    NameID pred_name_id_; ///The NameID of the Predicate to which this negation operator should refer.
};
}

#endif
