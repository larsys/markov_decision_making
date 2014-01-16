#include <ros/ros.h>
#include <markov_decision_making/state_layer.h>

using namespace std;
using namespace ros;
using namespace markov_decision_making;

int main ( int argc, char** argv )
{
    init ( argc, argv, "state_layer" );

    StateLayer sl;

    sl.addStateFactor ( StateDep().add ( "IsInSoccerField" )
                        .add ( "IsInLRM" )
                        .add ( "IsInSouthCorridor" )
                        .add ( "IsInCoffeeRoom" )
                        .add ( "IsInWestCorridor" )
                        .add ( "IsInElevatorHallway" ) );

    sl.addStateFactor ( StateDep().add ( "PatrolHalfwayThrough" ) );

    spin();

    return 0;
}
