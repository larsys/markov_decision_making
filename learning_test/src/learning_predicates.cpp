#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <predicate_manager/predicate_manager.h>
#include <predicate_manager/dependencies.h>
#include <predicate_manager/prop_logic_predicate.h>
#include <predicate_manager/prop_logic_event.h>
#include <predicate_manager/prop_and.h>
#include <predicate_manager/prop_pv.h>
#include <predicate_manager/prop_not.h>

#include <topological_tools/topological_predicate.h>

using namespace std;
using namespace ros;
using namespace predicate_manager;
using namespace topological_tools;



int main ( int argc, char** argv )
{
    init ( argc, argv, "predicates" );

    string label_target = "pose_label";

    PredicateManager pm;

    ///Predicate Instantiation (predicates used in the MDP's state layer)
    TopologicalPredicate isInSoccerField ( label_target, "IsInSoccerField" );
    TopologicalPredicate isInLRM ( label_target, "IsInLRM" );
    TopologicalPredicate isInCoffeeRoom ( label_target, "IsInCoffeeRoom" );
    TopologicalPredicate isInSouthCorridor ( label_target, "IsInSouthCorridor" );
    TopologicalPredicate isInWestCorridor ( label_target, "IsInWestCorridor" );
    TopologicalPredicate isInElevatorHallway ( label_target, "IsInElevatorHallway" );

    ///Registering predicates in the PM
    pm.addPredicate ( isInSoccerField );
    pm.addPredicate ( isInLRM );
    pm.addPredicate ( isInCoffeeRoom );
    pm.addPredicate ( isInSouthCorridor );
    pm.addPredicate ( isInWestCorridor );
    pm.addPredicate ( isInElevatorHallway );

    ///Starting PM
    pm.spin();

    return 0;
}
