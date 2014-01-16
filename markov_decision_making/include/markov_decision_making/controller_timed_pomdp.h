/**\file controller_timed_pomdp.h
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

#ifndef _CONTROLLER_TIMED_POMDP_H_
#define _CONTROLLER_TIMED_POMDP_H_

#include <string>

#include <markov_decision_making/controller_pomdp.h>



namespace markov_decision_making
{
class ControllerTimedPOMDP : public ControllerPOMDP
{
public:
    ControllerTimedPOMDP ( const std::string& problem_file,
                           const std::string& value_function_file,
                           const CONTROLLER_STATUS initial_status = STARTED );

    void startController();
private:
    void observationCallback ( const WorldSymbolConstPtr& msg );

    void scheduleTimer();
    void timerCallback ( const ros::TimerEvent& timerEvent );
    void step();

    ros::Timer timer_;

    Index o_;
};
}

#endif
