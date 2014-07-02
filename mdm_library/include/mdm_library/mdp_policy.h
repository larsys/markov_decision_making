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

#define MDM_DEFAULT_EPSILON 0.1


#include <mdm_library/common_defs.h>
#include <mdm_library/learning_defs.h>
#include <boost/numeric/ublas/io.hpp>
#include <fstream>

#include <ros/ros.h>


using namespace std;


namespace mdm_library
{
class MDPPolicy
{
public:
    MDPPolicy ( IndexVectorPtr policy_vec_ptr ) :
        policy_vec_ptr_ ( policy_vec_ptr ) {}
    
    virtual uint32_t getAction ( uint32_t index ) = 0;
    
    virtual void updatePolicy ( Matrix q_values ) {}
    
    virtual void setCurrDecisionEp ( uint32_t curr_decision_ep ) {}

    uint32_t operator[] ( uint32_t index )
    {
        return getAction ( index );
    }
    
    IndexVectorPtr getVector ()
    {
        return policy_vec_ptr_;
    }
    
protected:
    IndexVectorPtr policy_vec_ptr_;
};



class MDPPolicyVector : public MDPPolicy
{
public:
    MDPPolicyVector ( IndexVectorPtr p_ptr ) :
        MDPPolicy ( p_ptr ) {}

protected:
    virtual uint32_t getAction ( uint32_t index )
    {
        return ( *policy_vec_ptr_ ) [index];
    }
};



class MDPEpsilonGreedyPolicyVector : public MDPPolicy
{
public:
    MDPEpsilonGreedyPolicyVector ( IndexVectorPtr p_ptr,
                                   uint32_t num_states,
                                   uint32_t num_actions,
                                   EPSILON_TYPE epsilon_type,
                                   const string& file_path ) :
        MDPPolicy ( p_ptr ),
        num_states_ ( num_states ),
        num_actions_ ( num_actions ),
        epsilon_type_ ( epsilon_type ),
        file_path_ ( file_path ),
        private_nh_ ( "~" ),
        curr_decision_ep_ ( 0 )
    {
        srand ( time ( NULL ) );
        
        // Gather the alpha value from the parameters if alpha type is set as constant
        double epsilon;
        
        if ( epsilon_type_ == EPSILON_CONSTANT )
        {
            if ( private_nh_.hasParam ( "epsilon" ) )
            {
                private_nh_.getParam ( "epsilon", epsilon );
                
                if ( epsilon < 0 || epsilon > 1 )
                {
                    ROS_FATAL ( "Invalid provided epsilon value. The gamma value must be between 0 and 1." );
                    ros::shutdown();
                }
                else
                    epsilon_ = ( float ) epsilon;
            }
            else
            {
                epsilon_ = MDM_DEFAULT_EPSILON;
                cout << "Using a default value for epsilon of 0.1." << endl;
            }
        }
    }



    virtual void updatePolicy ( Matrix q_values )
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
        
        cout << "Policy updated!" << endl;
        
        savePolicy ();
    }



    void setCurrDecisionEp ( uint32_t curr_decision_ep )
    {
        curr_decision_ep_ = curr_decision_ep;
    }

protected:
    virtual uint32_t getAction ( uint32_t index )
    {
        // Probability to choose a random action (range: 0 - 1)
        //srand ( time ( NULL ) );
        double p = ( ( double ) rand () / ( RAND_MAX ) );
        
        if ( epsilon_type_ != EPSILON_CONSTANT )
            epsilon_ = updateEpsilon ( epsilon_type_, curr_decision_ep_ );
        
        // With probability epsilon choose a random action. Otherwise, follow the policy.
        if ( p <= epsilon_ )
        {
            // Choose a random index to select a random action
            //srand ( time ( NULL ) );
            uint32_t random_index = rand() % num_actions_;
            
            cout << "Choosing random action. Action chosen is " << random_index << endl;
            
            try
            {
                //return ( *policy_vec_ptr_ ) [random_index];
                return random_index;
            }
            catch ( exception& e )
            {
                ROS_ERROR_STREAM ( e.what() );
                abort();
            }
        }
        else
        {
            cout << "Following the policy. Action chosen is " << ( *policy_vec_ptr_ ) [index] << endl;
            
            try
            {
                return ( *policy_vec_ptr_ ) [index];
            }
            catch ( exception& e )
            {
                ROS_ERROR_STREAM ( e.what() );
                abort();
            }
        }
    }

private:
    uint32_t num_states_;
    uint32_t num_actions_;
    uint32_t curr_decision_ep_;
    EPSILON_TYPE epsilon_type_;
    const string& file_path_;
    float epsilon_;
    ros::NodeHandle private_nh_;
    
    
    
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
    
    
    
    void savePolicy ()
    {
//         string save_path;
//         
//         if ( private_nh_.hasParam ( "policy_save_path" ) )
//             private_nh_.getParam ( "policy_save_path", save_path );
//         else
//         {
//             save_path = "learnt_policy";
//             cout << "Param policy_save_path not set; writting the learnt policy to the cwd." << endl;
//             cout << "If no file is being created, make sure to use 'cwd=\"node\"' in your launch file when launching the";
//             cout << " control layer node." << endl;
//         }
            
        try
        {
            ofstream fp;
            
            //fp.open ( save_path.c_str(), ios::out );
            fp.open ( file_path_.c_str(), ios::out );

            fp << ( *policy_vec_ptr_ );
            
            fp.flush ();
            fp.close ();
            
            cout << "Policy saved!" << endl;
        }
        catch ( exception& e )
        {
            ROS_ERROR_STREAM ( e.what() );
            abort();
        }
    }
};
}

#endif
