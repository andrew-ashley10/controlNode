
#include <Eigen/Dense>
#include <ros/ros.h>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <time.h>


using Eigen::MatrixXd;


class LMS_estimation{

    public: 

    //constructor
    LMS_estimation(MatrixXd InputData); 


    private: 

    MatrixXd m_inputData; 


};