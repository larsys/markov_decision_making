/**\file prop_logic_event.h
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

#ifndef _PROP_LOGIC_EVENT_H_
#define _PROP_LOGIC_EVENT_H_

#include <string>
#include <map>

#include <predicate_manager/event.h>
#include <predicate_manager/prop_logic.h>

namespace predicate_manager
{
/**
 * PropLogicEvent implements an Event that is defined over a propositional formula.
 */
class PropLogicEvent : public Event
{
public:
    /**
     * Preferred constructor.
     * @param name The name of this Event.
     * @param pl The propositional formula which constitutes the event's triggering condition.
     */
    PropLogicEvent ( const std::string& name,
                     const PropLogic& pl );

    /**
     * Alternate constructor. You can use this form, for example, if you
     * want to instantiate the event but you don't yet know the names
     * of the variables involved in the respective propositional formula.
     * @param name The name of this event.
     */
    PropLogicEvent ( const std::string& name );

    ///To be called by the Predicate Manager.
    void update();

    ///Binds the triggering condition of this event to a propositional formula.
    void bindPropLogic ( const PropLogic& pl );

private:
    ///The function that is passed onto the PropLogic structure in order to evaluate the involved variables.
    bool evaluate ( const NameID& dep_name_id );

    ///Extracts the dependency set from the associated PropLogic and declares those dependencies.
    void declareAllDependencies();

    boost::shared_ptr<PropLogic> prop_logic_; ///The propositional formula to which this event is bound.

    bool val_; ///The latest known value of the propositional formula
};
}

#endif
