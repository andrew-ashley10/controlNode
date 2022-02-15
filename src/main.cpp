
#include <ros/ros.h>
#include "robot.h"
#include <Eigen/Dense>
#include <vector>
#include <mavros_msgs/ParamSet.h>
#include <iostream>

#include <GeographicLib/LambertConformalConic.hpp> 



// #include <core_planning_tutorial/planning_tutorial.h>
// #include <octomap/octomap.h>


using namespace Eigen; 
using namespace GeographicLib; 
using namespace std;


int main(int argc, char** argv)
{

    ///////////////////////////////////////////////////////////

    //ROS INITIALIZATION 

    ///////////////////////////////////////////////////////////
    //INITIATE ROS NODE
    std::cout<<"starting"<<std::endl; 
    ros::init(argc, argv, "control_node"); 

    ros::NodeHandle nh; 


    robot baseRobot(nh); 

    //only needed for final demo testing
    //wait until receivs distmap
    while(!baseRobot.octoMap->have_distmap)
    {
        ros::spinOnce(); 

        usleep(1000); 
    }
    ROS_INFO("Octomap has distmap"); 


    ////////////////////////////////////////////////////////////

    //EXPERIMENT SYSTEM

    ////////////////////////////////////////////////////////////

    // plan to target
    auto planner = new obstPlanner(baseRobot.octoMap, nh);

    usleep(30000); 

    vector<vector<double>> solutionPath = planner->plan();

    ROS_INFO("SOLUTION TO TARGET'S SIZE IS: %d",solutionPath.size()); 

    //send trajectories to vehicle

    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;

//     ///////////////////////////////////
//     //TODO: FIGURE OUT TRANSFORM STUFF
//     ///////////////////////////////////

//     //waypoint format
//     /*
//         uint8 FRAME_GLOBAL=0
//         uint8 FRAME_LOCAL_NED=1
//         uint8 FRAME_MISSION=2
//         uint8 FRAME_GLOBAL_REL_ALT=3
//         uint8 FRAME_LOCAL_ENU=4
//         uint8 frame
//         uint16 command
//         bool is_current
//         bool autocontinue
//         float32 param1
//         float32 param2
//         float32 param3
//         float32 param4
//         float64 x_lat
//         float64 y_long
//         float64 z_alt
//     */

   //first is takeoff command
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3977418;
    wp.y_long         = 8.5455936;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);

    //add waypoints from planner's solution
     for(int i = 0; i < solutionPath.size(); i++)
    {

        wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.is_current     = false;
        wp.autocontinue   = true;
        wp.x_lat          = 47.39772;
        wp.y_long         = 8.5455939;
        wp.z_alt          = 10;
        wp_push_srv.request.waypoints.push_back(wp);

    }


// WAIT FOR RPIC TO COMMAND MISSION START ON COMMAND LINE
   do 
    {
    cout << '\n' << "Press a key to continue...";
    } while (cin.get() != '\n');


//for this experiment: takeoff and THEN start mission
baseRobot.takeoff();

usleep(10000000);

//send waypoints
baseRobot.executeMission(wp_push_srv, (int)solutionPath.size());


//monitor waypoint progress, do not proceed until complete
  while(baseRobot.currentWaypointFinished!=solutionPath.size())
  {
      ros::spinOnce(); 
      ROS_INFO("Navigating to waypoint #%d",baseRobot.currentWaypointFinished+1);
      usleep(1000000);
  }

  //navigation mission finished
  ROS_INFO("Completed navigation mission, transitioning to make contact...");


// //get in contact TODO FIGURE THIS OUT

// //to cancel current mission, send empty mission
// //current thoughts on strategy... make mission with like 1/2 foot as each waypoint, move towards. When contact detected
// //cancel waypoint

// //once in contact: plan for rotation
//  std::vector <double> soln = baseRobot.rpyPlanner->plan();




//rotate
    //send roll change requests
    // for(int i = 0; i<soln.size(); i++)
    // {
    //     ROS_INFO("commanding roll change");
    //     ros::ServiceClient service_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set"); 
    //     mavros_msgs::ParamSet paramTest; 
    //     paramTest.request.param_id = "OMNI_ATT_MODE";
    //     paramTest.request.value.integer = soln[i];
    //     service_client.call(paramTest); 
    //     usleep(4000000); 

    // }






    ///////////////////////////////////////////////////////////
    //INDIVIDUAL COMPONENT TESTING
    ///////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////

    //TEST PLANNER

    ////////////////////////////////////////////////////////////
    // OctomapMapRepresentation octoMap = new OctomapMapRepresentation(nh);

    // usleep(3000000);
    //
    // // auto planner = new obstPlanner(octoMap);
    // // baseRobot.xyzPlanner->plan();
    //
    // std::vector <double> soln = baseRobot.rpyPlanner->plan();

    ///////////////////////////////////////////////////////////
    
    ///////////////////////////////////////////////////////////
    



    ////////////////////////////////////////////////////////////

    //MISSION TESTING

    ////////////////////////////////////////////////////////////

    // baseRobot.connect(); 

    //ARM VEHICLE
    // baseRobot.arm(); 

    // baseRobot.takeoff(); 


    //roll change based on rotationPlanner solution
    //send roll change requests
    // for(int i = 0; i<soln.size(); i++)
    // {
    //     ROS_INFO("commanding roll change");
    //     ros::ServiceClient service_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set"); 
    //     mavros_msgs::ParamSet paramTest; 
    //     paramTest.request.param_id = "OMNI_ATT_MODE";
    //     paramTest.request.value.integer = soln[i];
    //     service_client.call(paramTest); 
    //     usleep(4000000); 

    // }
    
    ///////////////////////////////////////////////////////////
    
    ///////////////////////////////////////////////////////////
    

    // //dumb waypoint test
    // mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    // mavros_msgs::Waypoint wp;
    // /*
    //     uint8 FRAME_GLOBAL=0
    //     uint8 FRAME_LOCAL_NED=1
    //     uint8 FRAME_MISSION=2
    //     uint8 FRAME_GLOBAL_REL_ALT=3
    //     uint8 FRAME_LOCAL_ENU=4
    //     uint8 frame
    //     uint16 command
    //     bool is_current
    //     bool autocontinue
    //     float32 param1
    //     float32 param2
    //     float32 param3
    //     float32 param4
    //     float64 x_lat
    //     float64 y_long
    //     float64 z_alt
    // */
    // // WP 0
    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    // wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    // wp.is_current     = true;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.3977418;
    // wp.y_long         = 8.5455936;
    // wp.z_alt          = 10;
    // wp_push_srv.request.waypoints.push_back(wp);
    // // WP 1
    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    // wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
    // wp.is_current     = false;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.3977418;
    // wp.y_long         = 8.5455936;
    // wp.z_alt          = 10;
  	// wp.param1			= 10;
  	// wp.param3			= 2;
  	// wp.param4			= 1;
    // wp_push_srv.request.waypoints.push_back(wp);
    // // WP 2
    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    // wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    // wp.is_current     = false;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.39772;
    // wp.y_long         = 8.5455939;
    // wp.z_alt          = 10;
    // wp_push_srv.request.waypoints.push_back(wp);


    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    // wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    // wp.is_current     = false;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.39772;
    // wp.y_long         = 8.5455930;
    // wp.z_alt          = 10;
    // wp_push_srv.request.waypoints.push_back(wp);


    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    // wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    // wp.is_current     = false;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.39772;
    // wp.y_long         = 8.5455915;
    // wp.z_alt          = 10;
    // wp_push_srv.request.waypoints.push_back(wp);
    // WP 3
    // wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    // wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    // wp.is_current     = false;(
    // wp.autocontinue   = true;
    // wp.x_lat          = 0;
    // wp.y_long         = 0;
    // wp.z_alt          = 0;
    // wp_push_srv.request.waypoints.push_back(wp);



    //dont execute or send mission until user presses key
    //  do 
    // {
    // cout << '\n' << "Press a key to continue...";
    // } while (cin.get() != '\n');




    //empty mission to clear other missions from vehicle
    // baseRobot.clearWaypoints(); 

    // usleep(2000000);

// SEND WAYPOINTS TO VEHICLE

    // baseRobot.executeMission(wp_push_srv, 5);

//TODO CHANGE TO SOLUTION SIZE
//   while(baseRobot.currentWaypointFinished!=4)
//   {
//       ros::spinOnce(); 
//       ROS_INFO("Navigating to waypoint #%d",baseRobot.currentWaypointFinished+1);
//       usleep(1000000);
//   }


//     cout<<"cancelling"<<endl; 
//     //test empty sending to cancel
//     mavros_msgs::WaypointPush wp_push_srv2; // List of Waypoints

//     baseRobot.sendWaypoints(wp_push_srv2);

// cout<<"mission finished"<<endl; 

     ros::spin(); 


    return 0; 


}