#include <Eigen/Dense>
#include <ros/ros.h>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <time.h>

#include <string>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
//#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "core_octomap_map_representation/octomap_map_representation.h"

#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"
#include "core_planning_state_space/state_space_utils/xyzpsi_state_space_utils.h"

#include <pluginlib/class_loader.h>

using Eigen::MatrixXd;

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std; 

class obstPlanner{

    public: 

    //constructor
    obstPlanner(OctomapMapRepresentation* octoMap, const ros::NodeHandle& nodeHandle); 

    vector<vector<double>> plan();

    bool initialize(); 

    private: 


    ros::NodeHandle nh; 

    OctomapMapRepresentation* map; 

    bool isvalid(const ob::State *state);

     ompl::base::ScopedState<XYZPsiStateSpace> getStartState();

    ompl::base::ScopedState<XYZPsiStateSpace> getGoalState();

    ompl::base::SpaceInformationPtr si_xyzpsi;

    ompl::base::ProblemDefinitionPtr pdef;

    ros::Publisher path_pub;


    MatrixXd m_inputData; 


};