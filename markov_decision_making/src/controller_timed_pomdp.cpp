/**\file controller_timed_pomdp.cpp
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

#include <madp/QAV.h>
#include <madp/PerseusPOMDPPlanner.h>
#include <madp/JointBelief.h>
#include <madp/JointBeliefSparse.h>
#include <madp/NullPlanner.h>

#include <markov_decision_making/controller_timed_pomdp.h>



using namespace std;
using namespace markov_decision_making;



ControllerTimedPOMDP::
ControllerTimedPOMDP ( string const& problem_file,
                       string const& value_function_file,
                       const CONTROLLER_STATUS initial_status ) :
    ControllerPOMDP ( problem_file, initial_status ),
    o_ ( 0 )
{
    if ( !nh_.hasParam ( "decision_period" ) )
    {
        ROS_ERROR ( "Decision period was not specified. Synchronous MDP controllers require the 'decision_period' parameter." );
        abort();
    }

    try
    {
        timer_ = nh_.createTimer ( 0, &ControllerTimedPOMDP::timerCallback, this, true, false );
        scheduleTimer();

        bool isSparse;
        if ( !nh_.getParam ( "is_sparse", isSparse ) )
        {
            isSparse = false;
        }


        boost::shared_ptr<PlanningUnitDecPOMDPDiscrete> np ( new NullPlanner ( loader_->GetDecPOMDP().get() ) );
        Q_ = boost::shared_ptr<QAV<PerseusPOMDPPlanner> >
             ( new QAV<PerseusPOMDPPlanner> ( np,
                                              value_function_file ) );

        if ( isSparse )
            belief_ = boost::shared_ptr<JointBeliefSparse>
                      ( new JointBeliefSparse ( loader_->GetDecPOMDP()->GetNrStates() ) );
        else
            belief_ = boost::shared_ptr<JointBelief>
                      ( new JointBelief ( loader_->GetDecPOMDP()->GetNrStates() ) );
        if ( initial_status == STARTED )
        {
            startController();
        }
    }
    catch ( E& e )
    {
        e.Print();
        abort();
    }
}



void
ControllerTimedPOMDP::
observationCallback ( const WorldSymbolConstPtr& msg )
{
    o_ = msg->world_symbol;
}



void
ControllerTimedPOMDP::
scheduleTimer()
{
    double dp;
    nh_.getParam ( "decision_period", dp );

    double timeToWait = dp - fmod ( ros::Time::now().toSec(), dp );
    ros::Duration d ( timeToWait );

    timer_.stop();
    timer_.setPeriod ( d );
    timer_.start();
}



void
ControllerTimedPOMDP::
timerCallback ( const ros::TimerEvent& timerEvent )
{
    step();
    scheduleTimer();
}



void
ControllerTimedPOMDP::
step()
{
    act ( o_ );
}



void
ControllerTimedPOMDP::
startController ()
{
    if ( belief_ == 0 )
    {
        ROS_WARN ( "ControllerTimedPOMDP:: Attempted to start controller, but the belief state hasn't been initialized." );
        return;
    }
    if ( ISD_ != 0 )
    {
        belief_->Set ( ISD_->ToVectorOfDoubles() );
    }
    else
    {
        belief_->Set ( * ( loader_->GetDecPOMDP()->GetISD() ) );
    }
    setStatus ( STARTED );
    resetDecisionEpisode();
}
