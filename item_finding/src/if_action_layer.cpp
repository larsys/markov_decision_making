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
        client_grasp_ ( "handle_object_grasp", true ),
        client_release_ ( "handle_object_release", true )
        {}
    
    void graspObject ()
    {
        client_grasp_.waitForServer ();
        
        item_finding::HandleObjectGoal goal;
        
        goal.grab_or_release = 1;
        
        client_grasp_.sendGoal ( goal );
        
        client_grasp_.waitForResult(ros::Duration(5.0));
    }
    
    void releaseObject ()
    {
        client_release_.waitForServer ();
        
        item_finding::HandleObjectGoal goal;
        
        goal.grab_or_release = 0;

        client_release_.sendGoal ( goal );
        
        client_release_.waitForResult(ros::Duration(5.0));
    }
    
private:
    NodeHandle nh_;
    Client client_grasp_;
    Client client_release_;
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
    
    al.getActionLayer() -> addAction ( boost::bind ( &Actions::graspObject, &am ), "GraspObject" );
    al.getActionLayer() -> addAction ( boost::bind ( &Actions::releaseObject, &am ), "ReleaseObject" );

    al.spin();

    return 0;
}
