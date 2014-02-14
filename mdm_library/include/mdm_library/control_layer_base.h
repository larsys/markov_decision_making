/**\file control_layer_base.h
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

#ifndef _CONTROL_LAYER_BASE_H_
#define _CONTROL_LAYER_BASE_H_



#ifdef HAVE_MADP

#include <madp/Globals.h>

#define MDM_MAXHORIZON MAXHORIZON

#else

#define MDM_MAXHORIZON 999999

#endif

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>



namespace mdm_library
{
/** The ControlLayerBase class provides the functionalities that are common to all MDM controllers.
 * These include:
 * - The ability to stop / start / reset a controller via service calls;
 * - The ability to get / reset the decision step number of a controller.
 */
class ControlLayerBase
{
public:
    /**
     * A simple flag to identify whether a controller is active or not.
     * All controllers should allow external stop / start / reset commands.
     */
    enum CONTROLLER_STATUS {STOPPED, STARTED};

    /**
     * Default constructor.
     * @param initial_status The initial status of this controller.
     * By default, Control Layers are auto-started. If the initial status is STOPPED, it must be
     * started externally (this is desirable for lower-level controllers in hierarchical systems).
     */
    ControlLayerBase ( const CONTROLLER_STATUS initial_status = STARTED );

    /** Returns the current decision step number of this controller (mostly for logging purposes).*/
    uint32_t getDecisionEpisode ();

    /** Returns the current status of this controller.*/
    CONTROLLER_STATUS getStatus ();

    /** Returns the horizon of this controller. The MADP-defined MAXHORIZON should be interpreted as infinite.*/
    uint32_t getDecisionHorizon ();
    /** Resets the decision step number of this controller back to 0.*/
    void resetDecisionEpisode ();

    /**
     * Stop this controller. When stopped, a controller does not update internally, not does it output any actions.
     * Be mindful that, for partially observable models, after stopping a controller, you may need to feed the correct initial belief state to
     * the controller before starting it again.
     * @sa startController(), resetController()
     */
    virtual void stopController();
    /**
     * Start this controller. The decision step number is set to 0 whenever a controller is started.
     * @sa stopController, resetController()
     */
    virtual void startController();
    /**
     * Stops and re-starts the controller.
     * @sa stopController(), startController()
     */
    virtual void resetController();

protected:
    /**
     * Increments the decision step number. If the step number reaches the problem horizon, the
     * controller is reset.
     */
    uint32_t incrementDecisionEpisode();
    /**
     * Allows direct access to the status of this controller. This can be used to reimplement the
     * start / stop / reset functions by derived classes.
     */
    void setStatus ( const CONTROLLER_STATUS status );

    ros::NodeHandle nh_;

private:
    /**
     * The callback for the "stop" service (Empty service type).
     * @sa stop_srv_
     */
    bool stopCallback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response );
    /**
     * The callback for the "start" service (Empty service type).
     * @sa start_srv_
     */
    bool startCallback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response );
    /**
    * The callback for the "reset" service (Empty service type).
    * @sa reset_srv_
    */
    bool resetCallback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response );

    /** The status of this controller. */
    CONTROLLER_STATUS status_;
    /** The decision step number. */
    uint32_t decision_episode_;
    /** The decision horizon. While, typically, infinite-horizon policies are used for robotic agents,
     * MDM also contemplates the possibility of using finite-horizon controllers.
     */
    uint32_t decision_horizon_;

    /**
     * The "stop" service server. This is an empty-type service in the namespace of the node containing the
     * Control Layer, which can be used to stop the controller.
     */
    ros::ServiceServer stop_srv_;
    /**
     * The "start" service server. This is an empty-type service in the namespace of the node containing the
     * Control Layer, which can be used to start the controller.
     */
    ros::ServiceServer start_srv_;
    /**
     * The "reset" service server. This is an empty-type service in the namespace of the node containing the
     * Control Layer, which can be used to reset (stop & start) the controller.
     */
    ros::ServiceServer reset_srv_;
};
}

#endif
