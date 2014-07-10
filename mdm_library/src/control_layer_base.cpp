/**\file control_layer_base.cpp
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

#include <mdm_library/control_layer_base.h>



using namespace ros;
using namespace std;
using namespace mdm_library;



ControlLayerBase::
ControlLayerBase ( const CONTROLLER_STATUS initial_status ) :
    status_ ( initial_status ),
    decision_episode_ ( 0 ),
    stop_srv_ ( nh_.advertiseService ( "stop", &ControlLayerBase::stopCallback, this ) ),
    start_srv_ ( nh_.advertiseService ( "start", &ControlLayerBase::startCallback, this ) ),
    reset_srv_ ( nh_.advertiseService ( "reset", &ControlLayerBase::resetCallback, this ) )
{
    NodeHandle private_nh ( "~" );
    int dh;
    if ( private_nh.getParam ( "decision_horizon", dh ) )
    {
        decision_horizon_ = ( uint32_t ) dh;
    }
    else
    {
        decision_horizon_ = MDM_MAXHORIZON;
    }
}



bool
ControlLayerBase::
stopCallback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
{
    stopController();
    return true;
}



bool
ControlLayerBase::
startCallback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
{
    startController();
    return true;
}



bool
ControlLayerBase::
resetCallback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
{
    resetController();
    return true;
}





void
ControlLayerBase::
stopController ()
{
    status_ = STOPPED;
}



void
ControlLayerBase::
startController ()
{
    decision_episode_ = 0;
    status_ = STARTED;
}



void
ControlLayerBase::
resetController ()
{
    ///This is primarily meant for controllers with reimplemented stop/start functions.
    stopController ();
    startController ();
}



void
ControlLayerBase::
setStatus ( const CONTROLLER_STATUS status )
{
    status_ = status;
}



uint32_t
ControlLayerBase::
getDecisionEpisode ()
{
    return decision_episode_;
}



ControlLayerBase::CONTROLLER_STATUS
ControlLayerBase::
getStatus ()
{
    return status_;
}



uint32_t
ControlLayerBase::
getDecisionHorizon ()
{
    return decision_horizon_;
}



void
ControlLayerBase::
resetDecisionEpisode ()
{
    decision_episode_ = 0;
}



uint32_t
ControlLayerBase::
incrementDecisionEpisode ()
{
    if ( ( decision_episode_ + 1 ) >= decision_horizon_ )
    {
        resetController();
        return decision_episode_;
    }
    
    return ++decision_episode_;
}



uint32_t
ControlLayerBase::
decrementDecisionEpisode ()
{    
    return --decision_episode_;
}
