#include <ros/ros.h>

#include <topological_tools/topological_move_base_action_layer.h>
#include <mdm_library/action_layer.h>
#include <item_finding/HandleObjectAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/concept_check.hpp>


using namespace ros;
using namespace std;
using namespace topological_tools;

typedef actionlib::SimpleActionClient<item_finding::HandleObjectAction> Client;


class Actions
{
public:
    Actions () :
        client_ ( "handle_object", true ) {}
    
    void graspObject ()
    {
        client_.waitForServer ();
        
        item_finding::HandleObjectGoal goal;
        
        goal.grab_or_release = 1;

        client_.sendGoal ( goal );
    }
    
//     void releaseObject ()
//     {
//         client_.waitForServer ();
//         
//         item_finding::HandleObjectGoal goal;
//         
//         goal.grab_or_release = 0;
// 
//         client_.sendGoal ( goal );
//     }
    
private:
    NodeHandle nh_;
    Client client_;
};



int main ( int argc, char** argv )
{
    init ( argc, argv, "action_layer" );

    if ( argc < 2 )
    {
        ROS_ERROR ( "Usage: rosrun mdm_example demo_action_layer <path to topological map>" );
        abort();
    }

    string topological_map_path = argv[1];

    Actions am;
    
    TopologicalMoveBaseActionLayer al ( topological_map_path );
    al.addAction ( "Up" );
    al.addAction ( "Down" );
    al.addAction ( "Left" );
    al.addAction ( "Right" );
    
    al.getActionLayer().addAction ( boost::bind ( &Actions::graspObject, &am ), "GraspObject" );
    //al.getActionLayer().addAction ( boost::bind ( &Actions::releaseObject, &am ), "ReleaseObject" );

    al.spin();

    return 0;
}
