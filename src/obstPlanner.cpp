#include "obstPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std; 


obstPlanner::obstPlanner(OctomapMapRepresentation* octoMap, const ros::NodeHandle& nodeHandle)
{

    nh = nodeHandle;
    map = octoMap;
    path_pub = this->nh.advertise<visualization_msgs::Marker>("solved_path",10);

    
}


bool obstPlanner::initialize()
{


    // std::string map_representation = pnh->param("map_representation", std::string("PointCloudMapRepresentation"));

    // path_pub = nh->advertise<visualization_msgs::Marker>("solved_path",10);

    // pluginlib::ClassLoader<MapRepresentation> map_representation_loader("core_map_representation_interface", "MapRepresentation");
    // try
    // {
    //     map = map_representation_loader.createInstance(map_representation);
    // }
    // catch (pluginlib::PluginlibException &ex)
    // {
    //     ROS_ERROR("The MapRepresentation plugin failed to load. Error: %s", ex.what());
    // }

}

bool obstPlanner::isvalid(const ob::State *state){

    
    Eigen::Vector3d pos = state->as<XYZPsiStateSpace::StateType>()->GetXYZ();
    geometry_msgs::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z(); //TODO Push in utils

    return map->isvalid(point);



}

vector<vector<double>> obstPlanner::plan(){

    // geometry_msgs::Point point;
    // point.x = 0;
    // point.y = 0;
    // point.z = 0;

    
    // // std::cout<<"Is starting point valid?  "<<map->isvalid(point)<<std::endl;

    ompl::base::RealVectorBounds bounds(3);

    double x(0),y(0),z(0);

    map->getLowBounds(x,y,z);
    bounds.setLow(0, x);
    bounds.setLow(1, y);
    bounds.setLow(2, z);


    std::cout<<"Passed the low bounds call"<<std::endl; 

    map->getHighBounds(x,y,z);
    bounds.setHigh(0, x);
    bounds.setHigh(1, y);
    bounds.setHigh(2, 100);

    si_xyzpsi = GetStandardXYZPsiSpacePtr();
    si_xyzpsi->getStateSpace()->as<XYZPsiStateSpace>()->SetBounds(bounds);
    si_xyzpsi->setStateValidityCheckingResolution(1.0/si_xyzpsi->getMaximumExtent());
    si_xyzpsi->setStateValidityChecker(std::bind(&obstPlanner::isvalid,this,std::placeholders::_1));
    si_xyzpsi->setup();


    pdef = std::make_shared<ob::ProblemDefinition>(si_xyzpsi);
    pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_xyzpsi)));
    
    auto start = getStartState();
    auto goal = getGoalState();
    pdef->setStartAndGoalStates(start,goal);

    auto planner = std::make_shared<og::RRTstar>(si_xyzpsi);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ROS_INFO("Planner Initialized");
    


    auto solved = planner->ob::Planner::solve(5); // seconds (edited) 

    std::cout<<"Solved"<<std::endl;
    
    vector<vector<double>> solutionPath; 

    if (solved == ob::PlannerStatus::EXACT_SOLUTION){
        ROS_INFO("Solved");

        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        path->interpolate(std::max(20, (int)path->getStateCount()));
        path_pub.publish(GetMarker(*path,5,0,1,0,1));


        //format for quick waypoint conversion
        for(int i = 0; i<(int)path->getStateCount(); i++)
        {
            auto tempState = path->getState(i); 
            Eigen::Vector3d pos = tempState->as<XYZPsiStateSpace::StateType>()->GetXYZ();

            vector<double> state; 
            state.push_back(pos.x()); 
            state.push_back(pos.y());
            state.push_back(pos.z());

            solutionPath.push_back(state); 
        }


    }

    return solutionPath; 
}

ob::ScopedState<XYZPsiStateSpace>  obstPlanner::getStartState() {

    //its x,z,y
    ob::ScopedState<XYZPsiStateSpace> state(si_xyzpsi);
    state->SetXYZ(Eigen::Vector3d(562,-584,50));//TODO add tf
    // state->SetXYZ(Eigen::Vector3d(-1133.912,0,-232.914));//TODO add tf
    state->SetPsi(0.0);

    return state;
}

ob::ScopedState<XYZPsiStateSpace>  obstPlanner::getGoalState() {


    //its x,z,y
    ob::ScopedState<XYZPsiStateSpace> state(si_xyzpsi);
    state->SetXYZ(Eigen::Vector3d(-1240,-700,50));
    // state->SetXYZ(Eigen::Vector3d(-633.277,5,-154.359));
    state->SetPsi(0.0);

    return state;
}



