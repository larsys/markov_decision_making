/**\file event.cpp
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

#include <predicate_manager/dependencies.h>


using namespace predicate_manager;

#include <predicate_manager/event.h>

#include <ros/ros.h>

using namespace std;
using namespace predicate_manager;


Event::
Event ( const string& name,
        const Dependencies& deps ) :
    PredicateDependentEntity ( deps ),
    name_ ( name ),
    trigger_ ()
{}

Event::
Event ( const string& name ) :
    PredicateDependentEntity(),
    name_ ( name ),
    trigger_ ()
{}

std::string Event::getName()
{
    return name_;
}

std::string Event::setName ( const std::string& new_name )
{
    name_ = new_name;
}

void Event::setTrigger ( const boost::function<void () > trigger )
{
    trigger_ = trigger;
}

void Event::triggerEvent ()
{
    trigger_();
}
