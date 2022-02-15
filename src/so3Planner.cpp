#include "so3Planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


so3Planner::so3Planner()
{


    
}

bool so3Planner::isStateValid(const ob::State *state){


    auto pos = state->as<ob::SO3StateSpace::StateType>();


    double w(0),x(0),y(0),z(0); 

    x = pos->x; 
    y = pos->y; 
    z = pos->z;
    w = pos->w; 

    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Matrix3d rx = q.toRotationMatrix();
    Eigen::Vector3d ea = rx.eulerAngles(2,1,0);

    //0=z or yaw,1=x or roll ,2=y or pitch


    double r = ea(2); 
    double p = ea(1); 
    double yaw = ea(0); 

    if(p != 0)
    {
        return false;
    }else{return true;}




}

std::vector<double> so3Planner::plan(){

  // construct the state space we are planning in
    auto space(std::make_shared<ob::SO3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    // ob::RealVectorBounds bounds(3);
    // bounds.setLow(-1);
    // bounds.setHigh(1);

    // space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a start state
    // ob::ScopedState<> start(space);
    // start.random();

    Eigen::Vector3d ea1; 
    //roll 
    ea1(2) = 0; 
    //pitch
    ea1(1) = 0; 
    //yaw
    ea1(0) = 0; 
    Eigen::Quaterniond q1 = Eigen::AngleAxisd(ea1[0], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ea1[1], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ea1[2], ::Eigen::Vector3d::UnitX());   


    ob::ScopedState<> start(space);
    start->as<ob::SO3StateSpace::StateType>()->x = q1.x();
    start->as<ob::SO3StateSpace::StateType>()->y = q1.y();
    start->as<ob::SO3StateSpace::StateType>()->z = q1.z();
    start->as<ob::SO3StateSpace::StateType>()->w = q1.w();





    // create goal state
    

    Eigen::Vector3d ea; 
    //roll 
    ea(2) = 0.15708; 
    //pitch
    ea(1) = 0; 
    //yaw
    ea(0) = 0; 
    Eigen::Quaterniond q = Eigen::AngleAxisd(ea[0], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ea[1], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ea[2], ::Eigen::Vector3d::UnitX());   


    ob::ScopedState<> goal(space);
    goal->as<ob::SO3StateSpace::StateType>()->x = q.x();
    goal->as<ob::SO3StateSpace::StateType>()->y = q.y();
    goal->as<ob::SO3StateSpace::StateType>()->z = q.z();
    goal->as<ob::SO3StateSpace::StateType>()->w = q.w();


    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTstar>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(2);

    std::vector<double> solvedPath; 

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);


        //get individual states convert to roll pitch yaw
        auto numStates = path->as<og::PathGeometric>()->getStateCount();
        for(int i = 0; i< numStates; i++){
            auto solvedX = path->as<og::PathGeometric>()->getState(i)->as<ob::SO3StateSpace::StateType>()->x;
            auto solvedY = path->as<og::PathGeometric>()->getState(i)->as<ob::SO3StateSpace::StateType>()->y;
            auto solvedZ = path->as<og::PathGeometric>()->getState(i)->as<ob::SO3StateSpace::StateType>()->z;
            auto solvedW = path->as<og::PathGeometric>()->getState(i)->as<ob::SO3StateSpace::StateType>()->w;


            //convert to roll pitch yaw
            Eigen::Quaterniond q2(solvedW, solvedX, solvedY, solvedZ);
            Eigen::Matrix3d rx2 = q2.toRotationMatrix();
            Eigen::Vector3d ea2 = rx2.eulerAngles(2,1,0);

            //for now just do roll TODO return all angles
            //add roll soln to path after converting to degrees
            solvedPath.push_back((ea2(1)*180)/M_PI); 
        }; 
    }
    else
        std::cout << "No solution found" << std::endl;

        return solvedPath; 

}


