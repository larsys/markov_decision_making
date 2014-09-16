#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

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



class IsObjectGrasped : public Predicate
{
public:
    IsObjectGrasped() :
        Predicate ( "IsObjectGrasped" ),
        grasp_sub_ ( nh_.subscribe ( "/agent/100/object_possession", 10, &IsObjectGrasped::graspCallback, this ) ),
        is_object_grasped_ ( false )
    {}

    void graspCallback ( const std_msgs::BoolConstPtr& msg )
    {
        bool val = msg -> data;
        
        if ( val != is_object_grasped_ )
        {
            is_object_grasped_ = val;
            update();
        }
    }

    void update ()
    {
        setValue ( is_object_grasped_ );
    }

private:
    NodeHandle nh_;
    Subscriber grasp_sub_;
    bool is_object_grasped_;
};



class IsObjectFoundWithHighConfidence : public Predicate
{
public:
    IsObjectFoundWithHighConfidence() :
        Predicate ( "IsObjectFoundWithHighConfidence" ),
        obj_found_sub_ ( nh_.subscribe ( "/agent/100/object_confidence", 10, &IsObjectFoundWithHighConfidence::objectFoundCallback, this ) ),
        is_object_found_with_high_confidence_ ( false )
    {}

    void objectFoundCallback ( const std_msgs::Int32ConstPtr& msg )
    {
        uint32_t val = msg -> data;
        
        if ( val == 2 )
        {
            if ( !is_object_found_with_high_confidence_ )
            {
                is_object_found_with_high_confidence_ = true;
                update ();
            }
        }
        else
        {
            if ( is_object_found_with_high_confidence_ )
            {
                is_object_found_with_high_confidence_ = false;
                update ();
            }
        }
    }

    void update ()
    {
        setValue ( is_object_found_with_high_confidence_ );
    }

private:
    NodeHandle nh_;
    Subscriber obj_found_sub_;
    bool is_object_found_with_high_confidence_;
};



class IsObjectFoundWithLowConfidence : public Predicate
{
public:
    IsObjectFoundWithLowConfidence() :
        Predicate ( "IsObjectFoundWithLowConfidence" ),
        obj_found_sub_ ( nh_.subscribe ( "/agent/100/object_confidence", 10, &IsObjectFoundWithLowConfidence::objectFoundCallback, this ) ),
        is_object_found_with_low_confidence_ ( false )
    {}

    void objectFoundCallback ( const std_msgs::Int32ConstPtr& msg )
    {
        uint32_t val = msg -> data;
        
        if ( val == 1 )
        {
            if ( !is_object_found_with_low_confidence_ )
            {
                is_object_found_with_low_confidence_ = true;
                update ();
            }
        }
        else
        {
            if ( is_object_found_with_low_confidence_ )
            {
                is_object_found_with_low_confidence_ = false;
                update ();
            }
        }
    }

    void update ()
    {
        setValue ( is_object_found_with_low_confidence_ );
    }

private:
    NodeHandle nh_;
    Subscriber obj_found_sub_;
    bool is_object_found_with_low_confidence_;
};



class IsObjectNotFound : public Predicate
{
public:
    IsObjectNotFound() :
        Predicate ( "IsObjectNotFound" ),
        obj_found_sub_ ( nh_.subscribe ( "/agent/100/object_confidence", 10, &IsObjectNotFound::objectFoundCallback, this ) ),
        is_object_not_found_ ( true )
    {}

    void objectFoundCallback ( const std_msgs::Int32ConstPtr& msg )
    {
        uint32_t val = msg -> data;
        
        if ( val == 0 )
        {
            if ( !is_object_not_found_ )
            {
                is_object_not_found_ = true;
                update ();
            }
        }
        else
        {
            if ( is_object_not_found_ )
            {
                is_object_not_found_ = false;
                update ();
            }
        }
    }

    void update ()
    {
        setValue ( is_object_not_found_ );
    }

private:
    NodeHandle nh_;
    Subscriber obj_found_sub_;
    bool is_object_not_found_;
};



class IsPersonFound : public Predicate
{
public:
   IsPersonFound() :
       Predicate ( "IsPersonFound" ),
       person_found_sub_ ( nh_.subscribe ( "/agent/100/person_found", 10, &IsPersonFound::personFoundCallback, this ) ),
       is_person_found_ ( false )
   {}

   void personFoundCallback ( const std_msgs::BoolConstPtr& msg )
   {
       bool val = msg -> data;
       
       if ( val )
       {
           if ( !is_person_found_ )
           {
               is_person_found_ = true;
               update ();
           }
       }
       else
       {
           if ( is_person_found_ )
           {
               is_person_found_ = false;
               update ();
           }
       }
   }

   void update ()
   {
       setValue ( is_person_found_ );
   }

private:
    NodeHandle nh_;
    Subscriber person_found_sub_;
    bool is_person_found_;
};



int main ( int argc, char** argv )
{
    init ( argc, argv, "predicates" );

    string label_target = "pose_label";

    PredicateManager pm;

    ///Predicate Instantiation (predicates used in the MDP's state layer)
    TopologicalPredicate isInBedroom ( label_target, "IsInBedroom" );
    TopologicalPredicate isInBathroom ( label_target, "IsInBathroom" );
    TopologicalPredicate isInInsideHallway ( label_target, "IsInInsideHallway" );
    TopologicalPredicate isInDiningArea ( label_target, "IsInDiningArea" );
    TopologicalPredicate isInTVArea ( label_target, "IsInTVArea" );
    TopologicalPredicate isInKitchenArea ( label_target, "IsInKitchenArea" );
    
    IsObjectGrasped isObjectGrasped;
    
    IsObjectFoundWithHighConfidence isObjectFoundWithHighConfidence;
    IsObjectFoundWithLowConfidence isObjectFoundWithLowConfidence;
    IsObjectNotFound isObjectNotFound;
    
    IsPersonFound isPersonFound;
    

    ///Registering predicates in the PM
    pm.addPredicate ( isInBedroom );
    pm.addPredicate ( isInBathroom );
    pm.addPredicate ( isInInsideHallway );
    pm.addPredicate ( isInDiningArea );
    pm.addPredicate ( isInTVArea );
    pm.addPredicate ( isInKitchenArea );
    
    pm.addPredicate ( isObjectGrasped );
    
    pm.addPredicate ( isObjectFoundWithHighConfidence );
    pm.addPredicate ( isObjectFoundWithLowConfidence );
    pm.addPredicate ( isObjectNotFound );
    
    pm.addPredicate ( isPersonFound );

    ///Starting PM
    pm.spin();

    return 0;
}
