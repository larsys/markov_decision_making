/**\file q_learning_mdp.cpp
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


#include <mdm_library/q_learning_mdp.h>
#include <mdm_library/Policy.h>


using namespace std;
using namespace mdm_library;


#ifdef HAVE_MADP



QLearningMDP::
QLearningMDP ( ALPHA_TYPE alpha_type,
               EPSILON_TYPE epsilon_type,
               CONTROLLER_TYPE controller_type,
               const string& problem_file_path,
               const std::string& initial_learning_policy_file_path,
               const string& policy_file_path,
               const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, policy_file_path, problem_file_path, initial_status )
{
    try
    {
        if ( learning_policy_ptr_ != 0 )
            ROS_WARN_STREAM ( "The learning policy for this MDP had already been loaded! Overwriting." );

        ifstream fp;
        fp.open ( initial_learning_policy_file_path.c_str() );
        IndexVectorPtr policy_vec ( new IndexVector() );

        fp >> ( *policy_vec );
        
        uint32_t number_of_states = q_values_.size1 ();
        uint32_t number_of_actions = q_values_.size2 ();
        
        MDPPolicy* learning_policy_ptr_ = new MDPEpsilonGreedyPolicyVector ( policy_vec,
                                                                             number_of_states,
                                                                             number_of_actions,
                                                                             epsilon_type );
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
    
    // Create the controller
    if ( controller_type == EVENT )
        controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerEventMDP ( policy_file_path,
                                                                                    epsilon_type,
                                                                                    initial_status );
    else
    {
        if ( controller_type == TIMED )
            controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerTimedMDP ( policy_file_path,
                                                                                        epsilon_type,
                                                                                        initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    state_sub_ = nh_.subscribe ( "state", 1, &QLearningMDP::stateSymbolCallback, this );
    
    initializeQValues ();
}



#endif


QLearningMDP::
QLearningMDP ( ALPHA_TYPE alpha_type,
               EPSILON_TYPE epsilon_type,
               CONTROLLER_TYPE controller_type,
               uint32_t num_states,
               uint32_t num_actions,
               const std::string& learning_policy_file_path,
               const std::string& policy_file_path,
               const std::string& reward_file_path,
               const std::string& q_values_path,
               const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, num_states, num_actions,
                        q_values_path, initial_status ),
    state_ ( 0 ),
    action_ ( 0 )
{
    // Create the controller
    if ( controller_type == EVENT )
        controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerEventMDP ( policy_file_path,
                                                                                    epsilon_type,
                                                                                    num_states,
                                                                                    num_actions,
                                                                                    initial_status );
    else
    {
        if ( controller_type == TIMED )
            controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerTimedMDP ( policy_file_path,
                                                                                        epsilon_type,
                                                                                        num_states,
                                                                                        num_actions,
                                                                                        initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    ( *controller_ ).loadRewardMatrix ( reward_file_path );
    
    state_sub_ = nh_.subscribe ( "state", 1, &QLearningMDP::stateSymbolCallback, this );
    policy_pub_ = nh_.advertise<Policy> ( "policy", 0, true );
    
    republish_service_ = nh_.advertiseService ( "publish_new_action", &QLearningMDP::republish_callback, this );
    
    if ( !loadQValues () )
        initializeQValues ();
    
    if ( lambda_ != 0)
        initializeEligibilityTraces ();
    
    if ( !loadLearningPolicy ( learning_policy_file_path, epsilon_type ) )
        initializeLearningPolicy ( learning_policy_file_path, epsilon_type );
}



QLearningMDP::
QLearningMDP ( ALPHA_TYPE alpha_type,
               EPSILON_TYPE epsilon_type,
               CONTROLLER_TYPE controller_type,
               uint32_t num_states,
               uint32_t num_actions,
               const std::string& learning_policy_file_path,
               const std::string& policy_file_path,
               const std::string& reward_file_path,
               const std::string& q_values_path,
               const string& eligibility_traces_path,
               const ControlLayerBase::CONTROLLER_STATUS initial_status ) :
    LearningLayerBase ( alpha_type, epsilon_type, controller_type, num_states, num_actions,
                        q_values_path, eligibility_traces_path, initial_status ),
    state_ ( 0 ),
    action_ ( 0 )
{
    // Create the controller
    if ( controller_type == EVENT )
        controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerEventMDP ( policy_file_path,
                                                                                    epsilon_type,
                                                                                    num_states,
                                                                                    num_actions,
                                                                                    initial_status );
    else
    {
        if ( controller_type == TIMED )
            controller_ = ( boost::shared_ptr<ControllerMDP> ) new ControllerTimedMDP ( policy_file_path,
                                                                                        epsilon_type,
                                                                                        num_states,
                                                                                        num_actions,
                                                                                        initial_status );
        else
        {
            ROS_FATAL ( "LearningLayer:: invalid controller type. Valid controller types are EVENT and TIMED." );
            ros::shutdown();
        }
    }
    
    ( *controller_ ).loadRewardMatrix ( reward_file_path );
    
    state_sub_ = nh_.subscribe ( "state", 1, &QLearningMDP::stateSymbolCallback, this );
    policy_pub_ = nh_.advertise<Policy> ( "policy", 0, true );
    
    republish_service_ = nh_.advertiseService ( "publish_new_action", &QLearningMDP::republish_callback, this );
    
    if ( !loadQValues () )
        initializeQValues ();
    
    if ( lambda_ != 0)
    {
        if ( !loadEligibilityTraces() )
            initializeEligibilityTraces ();
    }
    
    if ( !loadLearningPolicy ( learning_policy_file_path, epsilon_type ) )
        initializeLearningPolicy ( learning_policy_file_path, epsilon_type );
}



bool
QLearningMDP::
loadLearningPolicy ( const std::string& learning_policy_file_path, EPSILON_TYPE epsilon_type )
{
    try
    {
        ifstream fp;
        fp.open ( learning_policy_file_path.c_str() );
        
        if ( fp.peek() == std::ifstream::traits_type::eof() )
            return false;
        
        IndexVectorPtr policy_vec ( new IndexVector() );
        fp >> ( *policy_vec );
        
        cout << "Learning policy file is not empty. Loading the policy from the file." << endl;
        
        fp.close ();
        
        learning_policy_ptr_ = boost::shared_ptr<MDPPolicy> ( new MDPEpsilonGreedyPolicyVector ( policy_vec,
                                                                                                 num_states_,
                                                                                                 num_actions_,
                                                                                                 epsilon_type,
                                                                                                 learning_policy_file_path ) );
        
        return true;
    }
    catch ( exception& e )
    {
        ROS_ERROR_STREAM ( e.what() );
        abort();
    }
}



void
QLearningMDP::
initializeLearningPolicy ( const std::string& learning_policy_file_path, EPSILON_TYPE epsilon_type )
{
    IndexVectorPtr policy_vec ( new IndexVector ( num_states_ ) );
    
    for ( unsigned i = 0; i < num_states_; ++i )
        ( *policy_vec ) ( i ) = 0;
    
    cout << "Learning policy initialized as zeros!" << endl;
    
    learning_policy_ptr_ = boost::shared_ptr<MDPPolicy> ( new MDPEpsilonGreedyPolicyVector ( policy_vec,
                                                                                             num_states_,
                                                                                             num_actions_,
                                                                                             epsilon_type,
                                                                                             learning_policy_file_path ) );
}



void
QLearningMDP::
initializeQValues ()
{   
    size_t num_states = controller_.get() -> getNumberOfStates ();
    size_t num_actions = controller_ .get() -> getNumberOfActions ();

    q_values_ = Matrix ( num_states_, num_actions_ );
    
    // Initialize the Q values as 0
    for ( unsigned i = 0; i < q_values_.size1(); ++i )
        for ( unsigned j = 0; j < q_values_.size2(); ++j )
            q_values_ ( i, j ) = 0;
}



void
QLearningMDP::
updateQValues ()
{
    if ( alpha_type_ != ALPHA_CONSTANT )
        alpha_ = updateAlpha ( alpha_type_, curr_decision_ep_ );
    
    if ( lambda_ == 0 )
        q_values_ ( state_, action_ ) = q_values_ ( state_, action_ ) + alpha_ * ( reward_ + gamma_ *
                                        maxOverA() - q_values_ ( state_, action_ ) );
    else
    {
        uint32_t optimal_action = argMaxA();
        
        double delta = reward_ + gamma_ * q_values_ ( next_state_, optimal_action ) - q_values_ ( state_, action_ );
        
        et_ ( state_, action_ ) = et_ ( state_, action_ ) + 1;
        
        // For all (s,a) pair
        for ( unsigned state = 0; state < num_states_; ++state )
        {
            for ( unsigned action = 0; action < num_actions_; ++action )
            {
                q_values_ ( state, action ) = q_values_ ( state, action ) + alpha_ * delta * et_ ( state, action );
                
                if ( next_action_ == optimal_action )
                    et_ ( state, action ) = gamma_ * lambda_ * et_ ( state, action );
                else
                    et_ ( state, action ) = 0;
            }
        }
    }
}



float
QLearningMDP::
maxOverA ()
{
    // Initialize the current maximum as negative infinity
    double curr_max = std::numeric_limits<double>::infinity() * -1;
    
    // Find the action that leads to the highest Q value
    for (unsigned j = 0; j < num_actions_; j++ )
    {
        if ( q_values_ ( next_state_, j ) > curr_max )
            curr_max = q_values_ ( next_state_, j );
    }
    
    return curr_max;
}



uint32_t
QLearningMDP::
argMaxA ()
{
    // Initialize the current maximum as negative infinity
    double curr_max = std::numeric_limits<double>::infinity() * -1;
    uint32_t index;
    
    // Find the action that leads to the highest Q value
    for (unsigned j = 0; j < num_actions_; j++ )
    {
        if ( q_values_ ( next_state_, j ) > curr_max )
        {
            curr_max = q_values_ ( next_state_, j );
            index = j;
        }
    }
    
    return index;
}



void
QLearningMDP::
updatePolicy ()
{
    learning_policy_ptr_ -> updatePolicy ( q_values_ );
}



void
QLearningMDP::
stateSymbolCallback ( const mdm_library::WorldSymbolConstPtr& msg )
{   
    ROS_ERROR_STREAM ( "RECEIVED STATE MESSAGE!!!!" );
    cout << "State received is " << msg ->world_symbol << endl;
    newDecisionEpisode ( msg -> world_symbol );
}



bool
QLearningMDP::
republish_callback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
{
    ROS_ERROR_STREAM ( "REPUBLISH!!!!" );
    cout << "State sent is " << (*controller_).getLastState() << endl;
    
    ( *controller_ ).act ( ( *controller_ ).getLastState () );
    
    republish_ = true;
    
    newDecisionEpisode ( ( *controller_ ).getLastState () );
    
    return true;
}



void
QLearningMDP::
newDecisionEpisode ( uint32_t state )
{
    curr_decision_ep_ = ( *controller_ ).getDecisionEpisode ();
    
    if ( lambda_ == 0 )
        backupWithoutET ( state );
    else
        backupWithET ( state );
    
    if ( curr_decision_ep_ == 1 )
        publishPolicy ();
    
    if ( curr_decision_ep_ % policy_update_frequency_ == 0 )
    {
        updatePolicy ();
        publishPolicy ();
        saveQValues ();
        saveEligibilityTraces();
    }
    
    cout << "Decision episode finished." << endl;
}



void
QLearningMDP::
backupWithoutET ( uint32_t state )
{
    uint32_t obs_state = state;
    uint32_t obs_action = ( *controller_ ).getAction ();
    float obs_reward = ( *controller_ ).getReward ();
    
    if ( republish_ )
    {
        reward_ = impossible_action_reward_;
        republish_ = false;
    }
    
    if ( curr_decision_ep_ == 1 )
    {
        state_ = obs_state;
        action_ = obs_action;
        reward_ = obs_reward;
        
        cout << "Decision Episode #1:" << endl;
        cout << "\tState:\t\t" << state_ << endl;
        cout << "\tAction:\t\t" << action_ << endl;
        cout << "\tReward:\t\t" << reward_ << endl;
    }
    else
    {
        next_state_ = obs_state;
        
        cout << "Decision Episode #" << curr_decision_ep_ << ". Updating the QValues with:" << endl;
        cout << "\tState:\t\t" << state_ << endl;
        cout << "\tAction:\t\t" << action_ << endl;
        cout << "\tReward:\t\t" << reward_ << endl;
        cout << "\tNext State:\t" << next_state_ << endl;
        
        updateQValues ();
        
        state_ = next_state_;
        action_ = obs_action;
        reward_ = obs_reward;
    }
}



void
QLearningMDP::
backupWithET ( uint32_t state )
{
    cout << "BACKUP WITH ET" << endl;
    
    if ( curr_decision_ep_ == 1 )
    {
        uint32_t obs_state = state;
        
        state_ = obs_state;
        
        cout << "Decision Episode #1:" << endl;
        cout << "\tState:\t\t" << state_ << endl;
    }
    else
    {
        uint32_t obs_state = state;
        uint32_t obs_action = ( *controller_ ).getAction ();
        float obs_reward = 0;
        
        if ( republish_ )
        {
            obs_reward = impossible_action_reward_;
            republish_ = false;
        }
        else
            obs_reward = ( *controller_ ).getReward ();
        
        if ( curr_decision_ep_ == 2 )
        {
            action_ = obs_action;
            reward_ = obs_reward;
            next_state_ = obs_state;
            
            cout << "Decision Episode #2:" << endl;
            cout << "\tAction:\t\t" << action_ << endl;
            cout << "\tReward:\t\t" << reward_ << endl;
            cout << "\tNext State:\t\t" << next_state_ << endl;
        }
        else
        {
            next_action_ = obs_action;
            next_state_ = obs_state;
            
            updateQValues ();
            
            cout << "Decision Episode #" << curr_decision_ep_ << ". Updating the QValues with:" << endl;
            cout << "\tState:\t\t" << state_ << endl;
            cout << "\tAction:\t\t" << action_ << endl;
            cout << "\tReward:\t\t" << reward_ << endl;
            cout << "\tNext State:\t\t" << next_state_ << endl;
            cout << "\tNext Action:\t\t" << next_action_ << endl;
            
            state_ = next_state_;
            action_ = next_action_;
            reward_ = obs_reward;
        }
    }
}



void
QLearningMDP::
publishPolicy ()
{
    Policy pol;
    IndexVector v;
    
    v = ( * ( *learning_policy_ptr_ ).getVector () );
    
    pol.number_of_states = ( *controller_ ).getNumberOfStates ();
    
    std::vector<uint32_t> p ( pol.number_of_states );
    pol.policy = p;
    
    for ( int i = 0; i < ( *controller_ ).getNumberOfStates (); i++ )
        pol.policy[i] = v [i];
    
    policy_pub_.publish ( pol );
}
