/**\file state_dep.h
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * Markov Decision Making is a ROS library for robot decision-making based on MDPs.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of Markov Decision Making.
 *
 * Markov Decision Making is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Markov Decision Making is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _STATE_DEP_H_
#define _STATE_DEP_H_

#include <string>

#include <predicate_manager/common_defs.h>

#include <ros/ros.h>



namespace mdm_library
{
class StateDep
{
public:
    typedef std::vector<predicate_manager::NameID> SFDeps;

    StateDep
    add ( const std::string& predicate_name )
    {
        int pm_id;
        if ( !nh_.getParam ( "pm_id", pm_id ) )
        {
            ROS_WARN ( "StateDep:: Parameter \"pm_id\" is not set. This could lead to problems in the State Layer" );
            pm_id = 0;
        }
        return ( add ( predicate_name, ( uint32_t ) pm_id ) );
    }
    StateDep
    add ( const std::string& predicate_name, const uint32_t pm_id )
    {
        predicate_manager::NameID sfd ( pm_id, predicate_name );
        deps_.push_back ( sfd );
        return *this;
    }
    SFDeps getDependencies() const
    {
        return deps_;
    }

private:
    ros::NodeHandle nh_;
    SFDeps deps_;
};
}

#endif
