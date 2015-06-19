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

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/discrete_distribution.hpp>

#include <mdm_library/common_defs.h>

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
        policy_vec_ptr_ ( p_ptr ){}

protected:
    virtual uint32_t getAction ( uint32_t index )
    {
        return ( *policy_vec_ptr_ ) [index];
    }

private:
    IndexVectorPtr policy_vec_ptr_;
};

class MDPPolicyMatrix : public MDPPolicy
{
public:
    MDPPolicyMatrix ( MatrixPtr m_ptr ) :
        policy_matrix_ptr_ ( m_ptr ),
        gen_ ()
    {}

protected:
    virtual uint32_t getAction ( uint32_t index )
    {
        boost::numeric::ublas::matrix_column<Matrix> mc( *policy_matrix_ptr_, index );
        boost::random::discrete_distribution<> dist(mc);
        return dist(gen_); 
    }

private:
    MatrixPtr policy_matrix_ptr_;
    boost::mt19937 gen_;
};
}

#endif
