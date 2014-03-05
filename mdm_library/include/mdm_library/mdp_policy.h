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
#include <boost/numeric/ublas/io.hpp>

#include <ros/ros.h>



namespace mdm_library
{
class MDPPolicy
{
public:
    /**
     */
    virtual uint32_t getAction ( uint32_t index ) = 0;

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
    virtual uint32_t getAction ( uint32_t index )
    {
        return ( *policy_vec_ptr_ ) [index];
    }

private:
    IndexVectorPtr policy_vec_ptr_;
};



class MDPEpsilonGreedyPolicyVector : public MDPPolicy
{
public:
    MDPEpsilonGreedyPolicyVector ( IndexVectorPtr p_ptr, uint32_t num_states, uint32_t num_actions, float epsilon ) :
        policy_vec_ptr_ ( p_ptr ),
        num_states_ ( num_states ),
        num_actions_ ( num_actions ),
        epsilon_ ( epsilon )
        {}

protected:
    virtual uint32_t getAction ( uint32_t index )
    {
        // Probability to choose a random action (1 - 100)
        float p = rand() % 100 + 1;
        
        // With probability epsilon choose a random action. Otherwise, follow the policy.
        if ( p <= epsilon_ )
        {
            // Choose a random index to select a random action
            uint32_t random_index = rand() % num_actions_;
            
            return ( *policy_vec_ptr_ ) [random_index];
        }
        else
            return ( *policy_vec_ptr_ ) [index];
    }
    
    void updatePolicy ( Matrix q_values )
    {
        uint32_t best_action;
    
        for ( uint32_t state = 0; state < num_states_; state++ )
        {
            best_action = argMaxA ( q_values, state );
            
            try
            {
                ( *policy_vec_ptr_) ( state ) = best_action;
            }
            catch ( std::exception& e )
            {
                ROS_ERROR_STREAM ( e.what() );
                abort();
            }
        }
    }

private:
    IndexVectorPtr policy_vec_ptr_;
    uint32_t num_states_;
    uint32_t num_actions_;
    float epsilon_;
    
        uint32_t argMaxA ( Matrix q_values, uint32_t state )
    {
        // Initialize the current maximum as negative infinity
        double curr_max = std::numeric_limits<double>::infinity() * -1;
        uint32_t index;
        
        // Find the action that leads to the highest Q value
        for (unsigned j = 0; j < q_values.size2(); j++ )
        {
            if ( q_values ( state, j ) > curr_max )
            {
                curr_max = q_values ( state, j );
                index = j;
            }
        }
        
        return index;
    }
};
}

#endif
