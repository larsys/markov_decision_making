/**\file predicate.cpp
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

#include <predicate_manager/predicate.h>

#include <ros/ros.h>

using namespace std;
using namespace predicate_manager;


Predicate::
Predicate ( const string& name,
            const Dependencies& deps,
            bool initial_value ) :
    PredicateDependentEntity ( deps ),
    name_ ( name ),
    value_ ( initial_value ),
    trigger_ (),
    requested_updates_ ( 0 )
{}

Predicate::
Predicate ( const string& name,
            bool initial_value ) :
    PredicateDependentEntity(),
    name_ ( name ),
    value_ ( initial_value ),
    trigger_ (),
    requested_updates_ ( 0 )
{}

void
Predicate::
setValue ( bool val )
{
    if ( value_ != val )
    {
        if ( requested_updates_ > 0 )
        {
            ROS_WARN_STREAM ( "Predicate " << name_ << " is part of a cyclic dependency." );
        }

        if ( requested_updates_ > MAX_CYCLIC_UPDATES )
        {
            ROS_FATAL_STREAM ( "Predicate " << name_ << " is in a livelock. Terminating." );
            ros::shutdown();
        }

        requested_updates_++;

        value_ = val;

        if ( trigger_ )
            trigger_ ( value_ ); ///hands over control to PM, so the dependents can be updated.
        else
            ROS_WARN_STREAM ( "Predicate " << name_ << " changed its value, but it is not bound to any Predicate manager." );

        requested_updates_--;
    }
}

std::string Predicate::getName()
{
    return name_;
}

bool Predicate::getValue()
{
    return value_;
}

std::string Predicate::setName ( const std::string& new_name )
{
    name_ = new_name;
}

void Predicate::setTrigger ( const boost::function<void ( bool ) > trigger )
{
    trigger_ = trigger;
}
