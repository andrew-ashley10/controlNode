#include <ros/ros.h>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <time.h>
#include <vector>
#include <math.h>

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


using Eigen::MatrixXd;

namespace ob = ompl::base;
namespace og = ompl::geometric;


class so3Planner{

    public: 

    //constructor
    so3Planner(); 

    std::vector<double> plan();


    private: 


    static bool isStateValid(const ob::State *state);



    MatrixXd m_inputData; 


};