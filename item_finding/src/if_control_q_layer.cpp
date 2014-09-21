
#include <ros/ros.h>
#include <mdm_library/controller_event_mdp.h>
#include <mdm_library/sarsa_learning_mdp.h>
#include <mdm_library/q_learning_mdp.h>

using namespace std;
using namespace ros;
using namespace mdm_library;

int main ( int argc, char** argv )
{
    init ( argc, argv, "control_layer" );

    if ( argc < 5 )
    {
        ROS_ERROR ( "Usage: rosrun mdm_example demo_control_layer <path to policy file>" );
        abort();
    }

    string policy_path = argv[1];
    string learning_policy_path = argv[2];
    string reward_path = argv[3];
    string q_values_path = argv[4];

    ALPHA_TYPE alpha = ALPHA_CONSTANT;
    EPSILON_TYPE epsilon = EPSILON_CONSTANT;
    CONTROLLER_TYPE controller = EVENT;

    uint32_t num_states = 72;
    uint32_t num_actions = 6;

    QLearningMDP ql (alpha, epsilon, controller, num_states, num_actions, learning_policy_path, policy_path, reward_path, q_values_path );

    spin();

    return 0;
}
