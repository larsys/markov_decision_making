#include <ros/ros.h>
#include <mdm_library/state_layer.h>

using namespace std;
using namespace ros;
using namespace mdm_library;

int main ( int argc, char** argv )
{
    init ( argc, argv, "state_layer" );

    StateLayer sl;

    sl.addStateFactor ( StateDep().add ( "IsInBedroom" )
                                  .add ( "IsInBathroom" )
                                  .add ( "IsInInsideHallway" )
                                  .add ( "IsInOutsideHallway" )
                                  .add ( "IsInDiningArea" )
                                  .add ( "IsInTVArea" ) 
			          .add ( "IsInKitchenArea" ) );

    spin();

    return 0;
}
