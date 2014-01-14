/**\file controller_timed_mdp.cpp
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

#include <markov_decision_making/controller_timed_mdp.h>



using namespace std;
using namespace markov_decision_making;

#ifdef HAVE_MADP

ControllerTimedMDP::
ControllerTimedMDP (const string& policy_file_path,
                    const string& problem_file_path,
                    const CONTROLLER_STATUS initial_status) :
  ControllerMDP (policy_file_path, problem_file_path, initial_status),
  initial_state_known_ (false)
{
  if (!nh_.hasParam ("decision_period")) {
    ROS_ERROR ("Decision period was not specified. Synchronous MDP controllers require the 'decision_period' parameter.");
    abort();
  }
  
  timer_ = nh_.createTimer (0, &ControllerTimedMDP::timerCallback, this, true, false);
  scheduleTimer();
}

#endif

ControllerTimedMDP::
ControllerTimedMDP (const string& policy_file_path,
                    const CONTROLLER_STATUS initial_status) :
  ControllerMDP (policy_file_path, initial_status),
  initial_state_known_ (false)
{
  if (!nh_.hasParam ("decision_period")) {
    ROS_ERROR ("Decision period was not specified. Synchronous MDP controllers require the 'decision_period' parameter.");
    abort();
  }
  
  timer_ = nh_.createTimer (0, &ControllerTimedMDP::timerCallback, this, true, false);
  scheduleTimer();
}



void
ControllerTimedMDP::
stateCallback (const WorldSymbolConstPtr& msg)
{
  s_ = msg->world_symbol;
  if (!initial_state_known_) {
    initial_state_known_ = true;
  }
}



void
ControllerTimedMDP::
scheduleTimer()
{
  double dp;
  nh_.getParam ("decision_period", dp);
  
  double timeToWait = dp - fmod (ros::Time::now().toSec(), dp);
  ros::Duration d (timeToWait);
  
  timer_.stop();
  timer_.setPeriod (d);
  timer_.start();
}



void
ControllerTimedMDP::
timerCallback (const ros::TimerEvent& timer_event)
{
  if (initial_state_known_) {
    step();
  }
  
  scheduleTimer();
}



void
ControllerTimedMDP::
step()
{
  act (s_);
}
