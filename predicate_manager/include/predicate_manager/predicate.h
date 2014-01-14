/**\file predicate.h
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

#ifndef _PREDICATE_H_
#define _PREDICATE_H_

#include <string>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/unordered_map.hpp> ///using the boost implementation since it is faster and more compatible than 0x.
#define foreach BOOST_FOREACH

#include <predicate_manager/dependencies.h>
#include <predicate_manager/common_defs.h>

#define MAX_CYCLIC_UPDATES 100

namespace predicate_manager
{
  /**
   * The Predicate class is a base class for logical predicates. A "Predicate" is 
   * an object with an associated truth value that can depend on arbitrary sources of information.
   * This base class provides functionalities to ease the definition of predicates and their
   * interface with the PredicateManager class.
   */
  class Predicate
  {
    public:
      ///This type maps NameIDs to the logical values of the respective predicates.
      typedef boost::unordered_map<NameID, const bool*> PredValueMap;
      
      /**
       * This type allows predicates to have functional information regarding their dependencies:
       * - OK: The dependency exists and its value is known.
       * - UNDECLARED: The dependency has not been declared in its specified PM.
       * - UNKNOWN_VALUE: The value of the dependency is (still) unknown.
       */ 
      enum DEP_STATUS {OK, UNDECLARED, UNKNOWN_VALUE};
      
      /**
       * Predicate constructor.
       * @param name The name of this Predicate.
       * @param deps The Dependencies of this Predicate (e.g. a set of names of other Predicates that may influence its value).
       * @param initial_value The initial value of this predicate. It may be overridden according to its dependencies.
       * @sa Dependencies  
       */
      Predicate(const std::string& name,
                const Dependencies& deps,
                bool initial_value = false);

      /**
       * Predicate constructor.
       * @param name The name of this Predicate.
       * @param initial_value The initial value of this predicate.
       */
      Predicate(const std::string& name,
                bool initial_value = false);

      /**
       * Update function. This function represents the main logical condition of
       * this predicate. It must be implemented by derived classes and it should contain
       * at least one call to setValue(). It will be called by the PredicateManager
       * to which this predicate is added.
       */
      virtual void update() = 0;
      
      ///Declare a new dependency for this Predicate.
      void declareDependency (const std::string& dep_name, const int pm_id = 0);
      ///Declare a new dependency for this Predicate.
      void declareDependency (const NameID dep_name_id);
      
      /**
       * Binds a NameID to a reference to a boolean value. This allows the fast
       * access to the values of this Predicate's dependencies (through the getDependencyValue function).
       * @param dep_name_id A NameID defining a dependency of this predicate.
       * @param value_ref A (persistent) reference to a boolean value representing the value of that dependency.
       */
      void bindDependency (const NameID dep_name_id, const bool* value_ref);

      ///Gets the name of this Predicate.
      std::string getName();
      
      ///Gets the value of this Predicate.
      bool getValue();
      
      ///Gets a set of NameIDs containing the dependencies of this Predicate.
      NameIDSet getDependencies();
      
      ///Sets the name of this Predicate.
      std::string setName(const std::string& new_name);      

      /**
       * Sets the trigger function for this Predicate. The trigger function
       * is called whenever the Predicate changes its value (this is used by the PredicateManager).
       */
      void setTrigger(const boost::function<void (bool) > trigger);
      
    protected:
      ///Sets the value of this Predicate. Should be called from within update(). 
      void setValue(bool val);
      
      /**
       * Gets the value of a dependency of this Predicate.
       * @param dep_name The name of the dependency;
       * @param pm_id (optional) The ID of the PredicateManager to which this dependency belongs.
       * @param status (optional) The status of the dependency.
       * @sa DEP_STATUS
       */
      bool getDependencyValue(const std::string& dep_name, const int pm_id = 0, DEP_STATUS* status = NULL);
      
    private:
      std::string name_; ///The name of this Predicate;
      NameIDSet dep_set_; ///The names of the dependencies of this Predicate;
      PredValueMap dep_value_map_; ///A map of the values of the dependencies of this Predicate;
      bool value_; ///The logical value of this Predicate;
      /**
       * The number of times that this Predicate's setValue() function has been recursively called.
       * If it exceeds a given threshold (MAX_CYCLIC_UPDATES), the Predicate is considered to be in a
       * livelock.
       */
      uint8_t requested_updates_;
      /**
       * The trigger function.
       * @sa setTrigger
       */
      boost::function<void (bool) > trigger_;
  };
}

#endif