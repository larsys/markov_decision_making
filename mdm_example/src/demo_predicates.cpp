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


class PatrolHalfwayThroughPred : public Predicate
{
public:
    PatrolHalfwayThroughPred() :
        Predicate ( "PatrolHalfwayThrough",Dependencies()
                    .add ( "IsInElevatorHallway" )
                    .add ( "IsInSoccerField" ) )
    {}

    void update()
    {
        if ( getValue() )
        {
            if ( getDependencyValue ( "IsInSoccerField" ) )
                setValue ( false );
        }
        else
        {
            if ( getDependencyValue ( "IsInElevatorHallway" ) )
                setValue ( true );
        }
    }
};

/**
 * An example of a Predicate that defines a condition over an active ROS topic.
 * (This Predicate isn't used in the accompanying MDP state representation)
 */
class IsMovingPredicate : public Predicate
{
public:
    IsMovingPredicate() :
        Predicate ( "IsMoving" ),
        vel_subs_ ( nh_.subscribe ( "cmd_vel", 10, &IsMovingPredicate::velocityCallback, this ) ),
        is_moving_forward_ ( false )
    {}

    void velocityCallback ( const geometry_msgs::TwistConstPtr& msg )
    {
        bool val = msg->linear.x > 0.1;
        if ( val != is_moving_forward_ )
        {
            is_moving_forward_ = val;
            update();
        }
    }

    void update()
    {
        setValue ( is_moving_forward_ );
    }

private:
    NodeHandle nh_;
    Subscriber vel_subs_;
    bool is_moving_forward_;
};


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
    PatrolHalfwayThroughPred patrolHalfwayThrough;

    /// Other example predicates:
    IsMovingPredicate isMoving;
    /// An example of a predicate defined through a propositional formula:
    PropLogicPredicate plp ( "IsMovingInSouthCorridor", And ( PV ( "IsInSouthCorridor" ),PV ( "IsMoving" ) ) );

    /// An example of an event defined through a propositional formula:
    ///Triggers whenever the PatrolHalfwayThrough predicate falls (i.e. the patrol completes one round)
    PropLogicEvent ple ( "PatrolCompleted",Not ( "PatrolHalfwayThrough" ) );

    ///Registering predicates in the PM
    pm.addPredicate ( isInSoccerField );
    pm.addPredicate ( isInLRM );
    pm.addPredicate ( isInCoffeeRoom );
    pm.addPredicate ( isInSouthCorridor );
    pm.addPredicate ( isInWestCorridor );
    pm.addPredicate ( isInElevatorHallway );
    pm.addPredicate ( patrolHalfwayThrough );

    pm.addPredicate ( isMoving );
    pm.addPredicate ( plp );
    pm.addEvent ( ple );

    ///Starting PM
    pm.spin();

    return 0;
}
