#include <ros/ros.h>
#include <mdm_library/state_layer.h>

using namespace std;
using namespace ros;
using namespace mdm_library;

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

    spin();

    return 0;
}
