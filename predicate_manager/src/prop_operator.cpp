/**\file prop_operator.cpp
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

#include <predicate_manager/prop_operator.h>
#include <predicate_manager/common_defs.h>



using namespace predicate_manager;
using namespace std;



PropOperator::
PropOperator ( const PropLogic& p1,
               const PropLogic& p2 )
{
    children_.push_back ( p1.clone() );
    children_.push_back ( p2.clone() );
}



void
PropOperator::
printVariables ( NameIDSet& dep_set ) const
{
    foreach ( children_type::value_type c, children_ )
        c->printVariables ( dep_set );
}
