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



namespace mdm_library
{
    /** Flag to represent how alpha varies with "time". */
    enum ALPHA_TYPE {ALPHA_CONSTANT, ALPHA_ONE_OVER_T, ALPHA_ONE_OVER_T_SQUARED};
    
    /** Flag to represent how epsilon varies with "time". */
    enum EPSILON_TYPE {EPSILON_CONSTANT, EPSILON_ONE_OVER_T, EPSILON_ONE_OVER_T_SQUARED};
    
    /** Flag to represent whether the controller is event-based or timed. */
    enum CONTROLLER_TYPE {EVENT, TIMED};



class Parameter
{
public:
    float getValue ()
    {
        return value_;
    }
    
protected:
    float value_;
    std::string name_;
};



class ConstantParameter : public Parameter
{
public:
    ConstantParameter ( float value, std::string name )
    {
        value_ = value;
        name_ = name;
        
        if ( name.compare ( "alpha" ) != 0 && name.compare ( "epsilon" ) )
        {
            ROS_FATAL ( "LearningLayer:: the provided parameter name is invalid."
                        "Valid parameter names are 'alpha' and 'epsilon'." );
            ros::shutdown();
        }
        
        if ( name.compare ( "alpha" ) == 0 )
        {
            if ( value < 0 || value > 1 )
            {
                ROS_FATAL ( "LearningLayer:: the provided alpha value is outside the [0, 1] range." );
                ros::shutdown();
            }
        }
        
        if ( name.compare ( "epsilon" ) == 0 )
        {
            if ( value < 0 || value > 1 )
            {
                ROS_FATAL ( "LearningLayer:: the provided epsilon value is outside the [0, 1] range." );
                ros::shutdown();
            }
        }
    }
};



class Epsilon : public Parameter
{
public:
    Epsilon ( EPSILON_TYPE type )
    {
        type_ = type;
        name_ = "epsilon";
    }
    
protected:
    void updateValue ( uint32_t curr_decision_ep )
    {
        if ( type_ == EPSILON_ONE_OVER_T )
            value_ = 1 / curr_decision_ep;
        else
        {
            if ( type_ == EPSILON_ONE_OVER_T_SQUARED )
                value_ = 1 / ( curr_decision_ep * curr_decision_ep );
            else
            {
                ROS_FATAL ( "LearningLayer:: invalid epsilon type. "
                            "Valid types are EPSILON_ONE_OVER_T and EPSILON_ONE_OVER_T_SQUARED" );
                ros::shutdown();
            }
        }
    }
    
private:
    EPSILON_TYPE type_;
};



class Alpha : public Parameter
{
public:
    Alpha ( ALPHA_TYPE type )
    {
        type_ = type;
        name_ = "alpha";
    }
    
protected:
    void updateValue ( uint32_t curr_decision_ep )
    {
        if ( type_ == ALPHA_ONE_OVER_T )
            value_ = 1 / curr_decision_ep;
        else
        {
            if ( type_ == ALPHA_ONE_OVER_T_SQUARED )
                value_ = 1 / ( curr_decision_ep * curr_decision_ep );
            else
            {
                ROS_FATAL ( "LearningLayer:: invalid alpha type. "
                            "Valid types are ALPHA_ONE_OVER_T and ALPHA_ONE_OVER_T_SQUARED" );
                ros::shutdown();
            }
        }
    }
    
private:
    ALPHA_TYPE type_;
};    
}

#endif
