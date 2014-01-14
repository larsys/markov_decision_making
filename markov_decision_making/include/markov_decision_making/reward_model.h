/**\file reward_model.h
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

#ifndef _REWARD_MODEL_H_
#define _REWARD_MODEL_H_

#include <markov_decision_making/common_defs.h>



namespace markov_decision_making
{
  class RewardModel
  {
    public:
      /**
       */
      virtual double getReward (uint32_t state,
                                uint32_t action) = 0;
  };
  
  class RewardMatrix : public RewardModel
  {
    public:
      RewardMatrix (MatrixPtr matrix_ptr) :
        R_ptr_ (matrix_ptr) {}
        
      double getReward (uint32_t state,
                        uint32_t action) {
        return (*R_ptr_) (state, action);
      }
    private:
      MatrixPtr R_ptr_;
  };
  
#ifdef HAVE_MADP
  
  class MADPReward : public RewardModel
  {
      
  };
  
#endif
}

#endif
