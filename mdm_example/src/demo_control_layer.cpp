#include <ros/ros.h>
#include <markov_decision_making/controller_event_mdp.h>

using namespace std;
using namespace ros;
using namespace markov_decision_making;

int main (int argc, char** argv)
{
  init (argc, argv, "control_layer");
  
  if (argc < 2) {
    ROS_ERROR ("Usage: rosrun mdm_example demo_control_layer <path to policy file>");
    abort();
  }
  
  string policy_path = argv[1];
  
  ControllerEventMDP cl(policy_path);

  spin();
  
  return 0;
}
