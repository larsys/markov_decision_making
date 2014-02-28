/**\file online_learning_mdp.h
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

#ifndef _ONLINE_LEARNING_MDP_H_
#define _ONLINE_LEARNING_MDP_H_


#include <mdm_library/control_layer_base.h>


namespace mdm_library
{
/**
 * OnlineLearningMDP is the base class for online learning MDPs.
 */
class OnlineLearningMDP
{
public:
#ifdef HAVE_MADP
    OnlineLearningMDP ( const std::string& policy_file_path,
                        const std::string& problem_file_path,
                        const CONTROLLER_STATUS initial_status = STARTED );
#endif

    OnlineLearningMDP ( const std::string& policy_file_path,
                        const CONTROLLER_STATUS initial_status = STARTED );
    
protected:
    /** Q-table */
    //int q_values[][];
    
    /** Pure virtual function for updating the Q values. To be implemented in each specific method */
    virtual void updateQValues () = 0;
};
}

#endif
