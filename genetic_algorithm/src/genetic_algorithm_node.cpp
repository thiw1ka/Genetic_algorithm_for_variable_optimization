#include "genetic_algorithm/genetic_algorithm.hpp"


int main(int argc, char* argv[]) {
    // std::printf("Initiating genetic algorithem node. \n");
    ros::init( argc, argv,  "genetic_algorithm_node" );
    ros::NodeHandle nh;
    std::cout<<"[genetic_algorithm_node] genetic_algorithm_node started.."<<std::endl;
    ros::AsyncSpinner spinner(0); // Use all threads
    spinner.start();
    // std::unique_ptr<genetic_algorithm> ga (new genetic_algorithm(nh));
    genetic_algorithm ga(nh);
    ros::waitForShutdown();
    return 0;
};