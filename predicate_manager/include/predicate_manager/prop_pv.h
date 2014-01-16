/**\file prop_pv.h
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

#ifndef _PROP_PV_H_
#define _PROP_PV_H_

#include <string>
#include <set>

#include <boost/function.hpp>

#include <predicate_manager/prop_logic.h>



namespace predicate_manager
{
/**
 * PV stands for "Predicate Value".
 * The PV class allows the use value of an actual Predicate in PropLogic structures.
 * PV is one of the two unary operators that directly reference Predicate instances
 * (the other being "Not").
 */

class PV : public PropLogic
{
public:
    /**
     * Default constructor.
     * @param name The name of the Predicate which should be used in a PropLogic formula.
     */
    PV ( std::string const& name, const uint32_t pm_id = 0 );

    ///Evaluates this formula (i.e. gets the predicate value).
    bool evaluate ( boost::function<bool ( NameID ) > f ) const;

    ///A standard clone function for PropLogic and its derived classes.
    boost::shared_ptr<PropLogic> clone() const;

    ///A standard clone function for PropLogic and its derived classes.
    void printVariables ( NameIDSet& dep_set ) const;

private:
    NameID pred_name_id_; ///The NameID of the Predicate which should be used in a PropLogic formula.
};
}

#endif
