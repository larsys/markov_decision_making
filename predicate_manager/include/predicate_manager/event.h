/**\file event.h
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

#ifndef _EVENT_H_
#define _EVENT_H_

#include <string>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <predicate_manager/common_defs.h>
#include <predicate_manager/predicate_dependent_entity.h>

#define MAX_CYCLIC_UPDATES 100

namespace predicate_manager
{
/**
 * The Event class is a base class for instantaneous events. An "Event" is
 * an object associated to a condition, and that is triggered whenever that condition becomes true
 * (and only when that condition becomes true). Events can be though of as named representations of 
 * transitions of logical predicates. However, the Event class (and its sub-classes) can be used 
 * independently from the Predicate base class. Events can be bound to arbitrary user-defined conditions.
 * Note that, contrarily to Predicates, Events don't have a "state" -- they are instantaneous signals.
 */
class Event : public PredicateDependentEntity
{
public:
    /**
     * Event constructor.
     * @param name The name of this Event.
     * @param deps The Dependencies of this Event (e.g. a set of names of Predicates that may influence the event condition).
     * @sa Dependencies
     */
    Event ( const std::string& name,
            const Dependencies& deps );

    /**
     * Event constructor.
     * @param name The name of this Event.
     */
    Event ( const std::string& name );

    ///Gets the name of this Event.
    std::string getName();

    ///Sets the name of this Event.
    std::string setName ( const std::string& new_name );

    /**
     * Sets the trigger function for this Event. The trigger function
     * is typically set internally by an associated PredicateManager, and is called whenever the
     * user decides to trigger this event (through the triggerEvent() method).
     */
    void setTrigger ( const boost::function<void () > trigger );

protected:
    ///Triggers this Event (i.e. signals an associated PM that the event should be published). Should be called from within update().
    void triggerEvent();

private:
    std::string name_; ///The name of this Event;

    /**
     * The trigger function. Note that, for events, this function doesn't take any arguments.
     * @sa setTrigger
     */
    boost::function<void ( ) > trigger_;
};
}

#endif
