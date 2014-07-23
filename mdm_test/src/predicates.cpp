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
    TopologicalPredicate isInBedroom ( label_target, "IsInBedroom" );
    TopologicalPredicate isInBathroom ( label_target, "IsInBathroom" );
    TopologicalPredicate isInInsideHallway ( label_target, "IsInInsideHallway" );
    TopologicalPredicate isInOutsideHallway ( label_target, "IsInOutsideHallway" );
    TopologicalPredicate isInDiningArea ( label_target, "IsInDiningArea" );
    TopologicalPredicate isInTVArea ( label_target, "IsInTVArea" );
    TopologicalPredicate isInKitchenArea ( label_target, "IsInKitchenArea" );

    ///Registering predicates in the PM                                                                                                                        
    pm.addPredicate ( isInBedroom );
    pm.addPredicate ( isInBathroom );
    pm.addPredicate ( isInInsideHallway );
    pm.addPredicate ( isInOutsideHallway );
    pm.addPredicate ( isInDiningArea );
    pm.addPredicate ( isInTVArea );
    pm.addPredicate ( isInKitchenArea );

    ///Starting PM
    pm.spin();

    return 0;
}
