/**\file predicate_dependent_entity.h
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

#ifndef _PREDICATE_DEPENDENT_ENTITY_H_
#define _PREDICATE_DEPENDENT_ENTITY_H_

#include <string>

#include <predicate_manager/dependencies.h>
#include <predicate_manager/common_defs.h>

namespace predicate_manager
{
/**
 * The PredicateDependentEntity class is a base for classes that depend on the values of Predicates. 
 * This includes Predicates and Events.
 */
class PredicateDependentEntity
{
public:
    /**
     * PredicateDependentEntity constructor.
     * @param deps A set of names and respective PM identifiers of Predicates that may influence this object.
     * @sa Dependencies
     */
    PredicateDependentEntity ( const Dependencies& deps );

    /**
     * Empty constructor.
     */
    PredicateDependentEntity();

    /**
     * Update function. This function is to be called whenever one of the dependent
     * Predicates changes its value. It must be implemented by derived classes. 
     * It will be called by the PredicateManager to which this object is added.
     */
    virtual void update() = 0;

    ///Declare a new dependency.
    void declareDependency ( const std::string& dep_name, const int pm_id = 0 );
    ///Declare a new dependency.
    void declareDependency ( const NameID dep_name_id );

    /**
     * Binds a NameID to a reference to a boolean value. This allows the fast
     * access to the values of this object's dependencies (through the getDependencyValue function).
     * @param dep_name_id A NameID defining a dependency of this predicate.
     * @param value_ref A (persistent) reference to a boolean value representing the value of that dependency.
     */
    void bindDependency ( const NameID dep_name_id, const bool* value_ref );

    ///Gets a set of NameIDs containing the dependencies of this object.
    NameIDSet getDependencies();

protected:
    /**
     * Gets the value of a dependency.
     * @param dep_name The name of the dependency;
     * @param pm_id (optional) The ID of the PredicateManager to which this dependency belongs.
     * @param status (optional) The status of the dependency.
     * @sa DEP_STATUS
     */
    bool getDependencyValue ( const std::string& dep_name, const int pm_id = 0, DEP_STATUS* status = NULL );

private:
    NameIDSet dep_set_; ///The names and PM identifiers of the Predicate dependencies;
    PredValueMap dep_value_map_; ///A map of the values of the Predicate dependencies;
};
}

#endif
