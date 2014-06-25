#include <ros/ros.h>

#include <topological_tools/topological_move_base_action_layer.h>


using namespace ros;
using namespace std;
using namespace topological_tools;


int main ( int argc, char** argv )
{
    init ( argc, argv, "action_layer" );

    if ( argc < 2 )
    {
        ROS_ERROR ( "Usage: rosrun mdm_example demo_action_layer <path to topological map>" );
        abort();
    }

    string topological_map_path = argv[1];

    TopologicalMoveBaseActionLayer al ( topological_map_path );
    al.addAction ( "Up" );
    al.addAction ( "Down" );
    al.addAction ( "Left" );
    al.addAction ( "Right" );

    al.spin();

    return 0;
}
