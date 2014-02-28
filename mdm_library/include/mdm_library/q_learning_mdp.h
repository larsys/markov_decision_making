/**\file q_learning_mdp.h
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

#ifndef _Q_LEARNING_MDP_H_
#define _Q_LEARNING_MDP_H_


#include <mdm_library/online_learning_mdp.h>


namespace mdm_library
{
/**
 * QLearningMDP is the class for the Q-Learning method.
 */
class QLearningMDP : public OnlineLearningMDP
{
public:
#ifdef HAVE_MADP
    QLearningMDP ( const std::string& policy_file_path,
                   const std::string& problem_file_path,
                   const CONTROLLER_STATUS initial_status = STARTED );
#endif

    QLearningMDP ( const std::string& policy_file_path,
                   const CONTROLLER_STATUS initial_status = STARTED );
    
private:
    void updateQValues ();
};
}

#endif
