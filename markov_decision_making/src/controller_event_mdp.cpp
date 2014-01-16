/**\file controller_event_mdp.cpp
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

#include <markov_decision_making/controller_event_mdp.h>



using namespace std;
using namespace markov_decision_making;

#ifdef HAVE_MADP

ControllerEventMDP::
ControllerEventMDP ( const string& policy_file_path,
                     const string& problem_file_path,
                     const CONTROLLER_STATUS initial_status ) :
    ControllerMDP ( policy_file_path, problem_file_path, initial_status )
{
}

#endif

ControllerEventMDP::
ControllerEventMDP ( const string& policy_file_path,
                     const CONTROLLER_STATUS initial_status ) :
    ControllerMDP ( policy_file_path, initial_status )
{
}



void
ControllerEventMDP::
stateCallback ( const WorldSymbolConstPtr& msg )
{
    act ( msg->world_symbol );
}
