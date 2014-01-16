/**\file predicate_dependent_entity.cpp
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

#include <predicate_manager/predicate_dependent_entity.h>

#include <ros/ros.h>

using namespace std;
using namespace predicate_manager;


PredicateDependentEntity::
PredicateDependentEntity ( const Dependencies& deps )
{
    foreach ( NameID dep, deps.dependency_set_ )
        dep_value_map_[dep] = NULL; ///needs to be filled in by the PM
    dep_set_ = deps.dependency_set_;
}

PredicateDependentEntity::
PredicateDependentEntity()
{}

void
PredicateDependentEntity::
declareDependency ( const std::string& dep_name, const int pm_id )
{
    NameID dep_name_id ( pm_id, dep_name );
    declareDependency ( dep_name_id );
}

void
PredicateDependentEntity::
declareDependency ( const NameID dep_name_id )
{
    dep_set_.insert ( dep_name_id );
}

void
PredicateDependentEntity::
bindDependency ( const NameID dep_name_id, const bool* value_ref )
{
    dep_value_map_[dep_name_id] = value_ref;
}


bool
PredicateDependentEntity::
getDependencyValue ( const string& dep_name, const int pm_id, DEP_STATUS* status )
{
    bool val_declared = true;
    boost::unordered_map<NameID,const bool*>::const_iterator it;
    NameID dep_name_id ( pm_id, dep_name );

    it = dep_value_map_.find ( dep_name_id );

    if ( it == dep_value_map_.end() )
    {
        ROS_ERROR_STREAM ( "PredicateDependentEntity attempted to access undeclared dependency "
                           << dep_name );
        ROS_ERROR ( "Continuing, but returning a 'false' value for this dependency." );

        if ( status )
            *status = UNDECLARED;

        return false;
    }

    if ( ! ( it->second ) )
    {
        ROS_WARN_STREAM ( "Requested the value of predicate '" << dep_name << "' which is still unknown."
                          << " Using a default 'false' value." );

        if ( status )
            *status = UNKNOWN_VALUE;

        return false;
    }

    if ( status )
        *status = OK;

    return * ( it->second );
}

NameIDSet PredicateDependentEntity::getDependencies ()
{
    return dep_set_;
}

