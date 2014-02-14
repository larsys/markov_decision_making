/**
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
#include <madp/PerseusConstrainedPOMDPPlanner.h>
#include <madp/JointBeliefEventDriven.h>
#include <madp/NullPlanner.h>

#include <mdm_library/controller_event_pomdp.h>



using namespace ros;
using namespace std;
using namespace mdm_library;



ControllerEventPOMDP::
ControllerEventPOMDP ( string const& problem_file,
                       string const& value_function_file,
                       const CONTROLLER_STATUS initial_status ) :
    ControllerPOMDP ( problem_file, initial_status )
{
    try
    {
        bool isSparse;
        NodeHandle private_nh ( "~" );
        if ( !private_nh.getParam ( "is_sparse", isSparse ) )
        {
            isSparse = false;  ///Sparse JointBeliefEventDriven type NYI. Assign corresponding belief type then.
        }
        int false_negative_obs = -1; /// <0 means no false negative
        private_nh.getParam ( "false_negative_obs", false_negative_obs );

        QAVParameters qavParams;
        qavParams.falseNegativeObs = false_negative_obs;
        PlanningUnitMADPDiscreteParameters puParams;
        puParams.SetEventObservability ( loader_->GetDecPOMDP()->GetEventObservability() );
        double discount;
        if ( !private_nh.getParam ( "discount", discount ) )
        {
            discount = 0.9; ///this is only needed by the Planner, which is in turn required by the QAV
            /// In the general case, we won't use it, but it does allow for forward-search planning.
        }
        loader_->GetDecPOMDP()->SetDiscount ( discount );

        boost::shared_ptr<PlanningUnitDecPOMDPDiscrete> np ( new NullPlanner ( puParams, MAXHORIZON, loader_->GetDecPOMDP().get() ) );
        /// only considering infinite-horizon models at the moment.
        Q_ = boost::shared_ptr<QAV<PerseusConstrainedPOMDPPlanner> >
             ( new QAV<PerseusConstrainedPOMDPPlanner> ( np,
                     value_function_file,
                     qavParams ) );

        belief_ = boost::shared_ptr<JointBeliefEventDriven>
                  ( new JointBeliefEventDriven ( loader_->GetDecPOMDP()->GetNrStates(), false_negative_obs ) );
        if ( initial_status == STARTED )
        {
            startController();  ///Sets initial belief to the loader's ISD;
        }
    }
    catch ( E& e )
    {
        e.Print();
        abort();
    }
}



void
ControllerEventPOMDP::
observationCallback ( const WorldSymbolConstPtr& msg )
{
    act ( msg->world_symbol );
}



void
ControllerEventPOMDP::
startController ()
{
    if ( belief_ == 0 )
    {
        ROS_WARN ( "ControllerPOMDP:: Attempted to start controller, but the belief state hasn't been initialized." );
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
    act ( 0 ); ///calling first decision at startup, observation is irrelevant.
}
