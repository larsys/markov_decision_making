/**\file controller_timed_mdp.h
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

#ifndef _CONTROLLER_TIMED_MDP_H_
#define _CONTROLLER_TIMED_MDP_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <mdm_library/controller_mdp.h>



namespace mdm_library
{
/**
 * ControllerTimedMDP implements an MDP controller that takes actions at a fixed rate.
 */
class ControllerTimedMDP : public ControllerMDP
{
public:
#ifdef HAVE_MADP
    ControllerTimedMDP ( const std::string& policy_file_path,
                         const std::string& problem_file_path,
                         const CONTROLLER_STATUS initial_status = STARTED );
#endif

    ControllerTimedMDP ( const std::string& policy_file_path,
                         const CONTROLLER_STATUS initial_status = STARTED );

    void stateCallback ( const WorldSymbolConstPtr& msg );
private:
    void scheduleTimer();
    void timerCallback ( const ros::TimerEvent& timer_event );
    void step();

    bool initial_state_known_;

    ros::Timer timer_;

    uint32_t s_;
};
}

#endif
