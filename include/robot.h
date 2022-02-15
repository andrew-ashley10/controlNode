#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointClear.h>
#include <sensor_msgs/Imu.h>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include "imuDataContainer.h"
#include <cmath>
#include <fstream>
#include <time.h>
#include "LMS_estimation.h"
#include "obstPlanner.h"
#include "so3Planner.h"

#include "core_octomap_map_representation/octomap_map_representation.h"

#include <vector>


//#include "Iir.h"

using namespace std; 
using Eigen::MatrixXd;


class robot 
{
    public: 

        //constructor
        robot(const ros::NodeHandle& nodeHandle); 

        //planner 
        obstPlanner* xyzPlanner; 
        so3Planner* rpyPlanner; 

        //Vehicle Control 
        arm(); 
        bool connect(); 
        sendWaypoints(mavros_msgs::WaypointPush wp_push_srv); 
        modeSetting(); 
        takeoff(); 
        clearWaypoints(); 
        bool executeMission(mavros_msgs::WaypointPush wp_push_srv, double missionSize); 
        std::vector<vector<double>> plan(); 

        MatrixXd estimate_cog(MatrixXd angular_velocity, MatrixXd angular_accel, MatrixXd linear_accel);

    //map
        OctomapMapRepresentation* octoMap; 


    //waypoint status
        int currentWaypointFinished = 0; 

    
    private: 

        //number of calculations for cog estimate
        double calcNum = 0; 
        
        //ros variables
        ros::NodeHandle nh; 
        mavros_msgs::State current_state;
        sensor_msgs::Imu last_imu_reading; 
        
        ros::Subscriber state_sub; 
        ros::Subscriber mission_state_sub; 
        ros::Subscriber imu_sub;
        ros::ServiceClient arming_client; 
        ros::ServiceClient set_mode_client; 
        ros::ServiceClient wp_client;
        ros::Publisher local_pose_pub; 

        //setup for waypoints
        mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
        mavros_msgs::Waypoint wp;


        //previous time to calculate acceleration
        int last_time; 


        //filter angular velocity and linear accel
        double last_veloc_x=0; 
        double last_veloc_y=0;
        double last_veloc_z=0;

        double last_linear_accel_x = 0; 
        double last_linear_accel_y = 0; 
        double last_linear_accel_z = 0; 

        double last_filtered_angl_veloc_x = 0; 
        double last_filtered_angl_veloc_y = 0; 
        double last_filtered_angl_veloc_z = 0;

        double last_filtered_linear_accel_x = 0;  
        double last_filtered_linear_accel_y = 0;  
        double last_filtered_linear_accel_z = 0;  

        double angular_accel_x; 
        double angular_accel_y;
        double angular_accel_z; 

    
        std::ofstream logFile;



        int filterGyro(); 
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void mission_state_cb(const mavros_msgs::WaypointReached::ConstPtr& msg);
        void imu_cb(const sensor_msgs::Imu::ConstPtr& msg); 



        //Matrix for cog estimations, get 2000 readings (can change later)
        MatrixXd cogEstimationsX = MatrixXd::Constant(2000,2,0); 
        MatrixXd cogEstimationsY = MatrixXd::Constant(2000,2,0); 
        MatrixXd cogEstimationsZ = MatrixXd::Constant(2000,2,0); 

        //Iteration count for estimation
        double itercount = 0; 

        
        bool inContact; 


};