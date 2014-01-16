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
 * The Predicate class is a base class for logical predicates. A "Predicate" is
 * an object with an associated truth value that can depend on arbitrary sources of information.
 * This base class provides functionalities to ease the definition of predicates and their
 * interface with the PredicateManager class.
 */
class Event : public PredicateDependentEntity
{
public:
    /**
     * Predicate constructor.
     * @param name The name of this Predicate.
     * @param deps The Dependencies of this Predicate (e.g. a set of names of other Predicates that may influence its value).
     * @sa Dependencies
     */
    Event ( const std::string& name,
            const Dependencies& deps );

    /**
     * Predicate constructor.
     * @param name The name of this Predicate.
     */
    Event ( const std::string& name );

    ///Gets the name of this Predicate.
    std::string getName();

    ///Sets the name of this Predicate.
    std::string setName ( const std::string& new_name );

    /**
     * Sets the trigger function for this Predicate. The trigger function
     * is called whenever the Predicate changes its value (this is used by the PredicateManager).
     */
    void setTrigger ( const boost::function<void () > trigger );

protected:
    ///Sets the value of this Predicate. Should be called from within update().
    void triggerEvent();

private:
    std::string name_; ///The name of this Event;

    /**
     * The trigger function. Note that, for events, this function doesn't take any arguments
     * @sa setTrigger
     */
    boost::function<void ( ) > trigger_;
};
}

#endif
