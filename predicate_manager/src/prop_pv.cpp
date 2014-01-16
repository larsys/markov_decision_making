/**\file prop_pv.cpp
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

#include <predicate_manager/prop_pv.h>



using namespace predicate_manager;
using namespace std;



PV::
PV ( std::string const& name, const uint32_t pm_id ) :
    pred_name_id_ ( pm_id, name )
{
}



bool
PV::
evaluate ( boost::function<bool ( NameID ) > f ) const
{
    return f ( pred_name_id_ );
}



boost::shared_ptr<PropLogic>
PV::
clone() const
{
    boost::shared_ptr<PV> p ( new PV ( *this ) );
    return ( boost::dynamic_pointer_cast<PropLogic> ( p ) );
}



void
PV::
printVariables ( NameIDSet& dep_set ) const
{
    dep_set.insert ( pred_name_id_ );
}
