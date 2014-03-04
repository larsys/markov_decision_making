/**\file mdp_policy.h
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

#ifndef _MDP_POLICY_H_
#define _MDP_POLICY_H_

#include <mdm_library/common_defs.h>
#include <boost/concept_check.hpp>



namespace mdm_library
{
class MDPPolicy
{
public:
    /**
     */
    virtual uint32_t getAction ( uint32_t index, uint32_t num_actions = 0, float epsilon = 0 ) = 0;
    virtual void setAction ( uint32_t state, uint32_t action );

    uint32_t operator[] ( uint32_t index )
    {
        return getAction ( index );
    }
};



class MDPPolicyVector : public MDPPolicy
{
public:
    MDPPolicyVector ( IndexVectorPtr p_ptr ) :
        policy_vec_ptr_ ( p_ptr ) {}

protected:
    virtual uint32_t getAction ( uint32_t index, uint32_t num_actions = 0, float epsilon = 0 )
    {
        return ( *policy_vec_ptr_ ) [index];
    }

private:
    IndexVectorPtr policy_vec_ptr_;
};



class MDPEpsilonGreedyPolicyVector : public MDPPolicy
{
public:
    MDPEpsilonGreedyPolicyVector ( IndexVectorPtr p_ptr ) :
        policy_vec_ptr_ ( p_ptr ) {}

protected:
    virtual uint32_t getAction ( uint32_t index, uint32_t num_actions, float epsilon )
    {
        // Probability to choose a random action (1 - 100)
        float p = rand() % 100 + 1;
        
        // With probability epsilon choose a random action. Otherwise, follow the policy.
        if ( p <= epsilon )
        {
            // Choose a random index to select a random action
            uint32_t random_index = rand() % num_actions;
            
            return ( *policy_vec_ptr_ ) [random_index];
        }
        else
            return ( *policy_vec_ptr_ ) [index];
    }
    
    
    
    virtual void setAction ( uint32_t state, uint32_t action )
    {
        //policy_vec_ptr_ ( state ) = action;
    }

private:
    IndexVectorPtr policy_vec_ptr_;
};
}

#endif
