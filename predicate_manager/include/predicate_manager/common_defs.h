/**\file common_defs.h
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

#ifndef _PM_COMMON_DEFS_H_
#define _PM_COMMON_DEFS_H_

#include <set>
#include <string>
#include <boost/unordered_map.hpp> ///using the boost implementation since it is faster and more compatible than 0x.
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace predicate_manager
{
/**
 * The NameID type uniquely identifies a Predicate through the ID of its PM and its name.
 * NameID = <PM ID, name>
 * This type allows for a simpler, easy-to-use interface when implementing Predicates.
 */
typedef std::pair<int, std::string> NameID;
/**
 * The NrID type uniquely identifies a Predicate through the ID of its PM and its registration number.
 * NrID = <PM ID, reg. number>
 * This type allows for faster access to local structures.
 */
typedef std::pair<int, uint32_t> NrID;
/**
 * A set of NameIDs.
 */
typedef std::set<NameID> NameIDSet;

///Cantor pairing function.
///Ensures constant time access (no collisions).
struct cantor_pair_hash
        : std::unary_function<NrID, std::size_t>
{
    std::size_t operator() ( const NrID& id ) const
    {
        return 0.5* ( id.first+id.second+1 ) * ( id.first+id.second ) +id.second;
    }
};

///This type maps NameIDs to the logical values of the respective predicates.
typedef boost::unordered_map<NameID, const bool*> PredValueMap;

/**
 * This type allows predicates and events to have functional information regarding their dependencies:
 * - OK: The dependency exists and its value is known.
 * - UNDECLARED: The dependency has not been declared in its specified PM.
 * - UNKNOWN_VALUE: The value of the dependency is (still) unknown.
 */
enum DEP_STATUS {OK, UNDECLARED, UNKNOWN_VALUE};
}

#endif
