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
                                  .add ( "IsInDiningArea" )
                                  .add ( "IsInTVArea" )
                                  .add ( "IsInKitchenArea" ) );

    sl.addStateFactor ( StateDep().add ( "IsObjectGrasped" ) );
    
    sl.addStateFactor ( StateDep().add ( "IsObjectNotFound" )
                                  .add ( "IsObjectFoundWithLowConfidence" )
                                  .add ( "IsObjectFoundWithHighConfidence" ) );
    
    sl.addStateFactor ( StateDep().add ( "IsPersonFound" ) );
    
    spin();

    return 0;
}
