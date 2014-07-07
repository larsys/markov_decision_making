/**\file learning_defs.h
 *
 * Author:
 * Pedro Resende <pt.resende@gmail.com>
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

#ifndef _LEARNING_DEFS_
#define _LEARNING_DEFS_


#include <ros/ros.h>


using namespace std;


namespace mdm_library
{
    /** Flag to represent how alpha varies with "time". */
    enum ALPHA_TYPE {ALPHA_CONSTANT, ALPHA_ONE_OVER_T, ALPHA_ONE_OVER_T_SQUARED};
    
    /** Flag to represent how epsilon varies with "time". */
    enum EPSILON_TYPE {EPSILON_CONSTANT, EPSILON_ONE_OVER_T, EPSILON_ONE_OVER_T_SQUARED, EPSILON_ONE_OVER_T_ROOTED};
    
    /** Flag to represent whether the controller is event-based or timed. */
    enum CONTROLLER_TYPE {EVENT, TIMED};



    inline float updateAlpha ( ALPHA_TYPE alpha_type, uint32_t curr_decision_ep )
    {
        double alpha;
        float updated_alpha;

        switch ( alpha_type )
        {
            case ALPHA_ONE_OVER_T:
                updated_alpha = 1.0 / ( float ) curr_decision_ep;
                break;
                
            case ALPHA_ONE_OVER_T_SQUARED:
                updated_alpha = 1.0 / ( ( float ) curr_decision_ep * ( float ) curr_decision_ep );
                break;
                
            default:
                ROS_FATAL ( "LearningLayer:: invalid alpha type. "
                            "Valid types are ALPHA_CONSTANT, ALPHA_ONE_OVER_T and ALPHA_ONE_OVER_T_SQUARED" );
                ros::shutdown();
                break;
        }
        
        return updated_alpha;
    }



    inline float updateEpsilon ( EPSILON_TYPE epsilon_type, uint32_t curr_decision_ep )
    {
        float updated_epsilon;
        
        switch ( epsilon_type )
        {
            case EPSILON_ONE_OVER_T:
                if ( curr_decision_ep == 0 )
                    updated_epsilon = 1;
                else
                    updated_epsilon = 1.0 / ( ( float ) curr_decision_ep );
                
                break;
                
            case EPSILON_ONE_OVER_T_SQUARED:
                
                if ( curr_decision_ep == 0 )
                    updated_epsilon = 1;
                else
                    updated_epsilon = 1.0 / ( ( float ) curr_decision_ep * ( float ) curr_decision_ep + 1 );
                
                break;
                
            case EPSILON_ONE_OVER_T_ROOTED:
                
                if ( curr_decision_ep == 0 )
                    updated_epsilon = 1;
                else
                    updated_epsilon = 1.0 / ( ( float ) pow ( curr_decision_ep, ( double ) 1 / 4 ) );
                
                break;
                
            default:
                ROS_FATAL ( "Invalid provided epsilon value. The epsilon value must be between 0 and 1." );
                ros::shutdown();
                break;
        }
        
        cout << "New Epsilon is " << updated_epsilon << endl;
        
        return updated_epsilon;
    }
}

#endif
