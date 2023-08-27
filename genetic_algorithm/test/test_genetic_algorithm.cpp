#include "../test/Unit_test.hpp"

int main (int argc, char* argv[]) {
    std::cout << "testing genetic algorithm \n";


    // std::printf("Initiating genetic algorithem node. \n");
    ros::init( argc, argv,"test_genetic_algorithm_node");
    ros::NodeHandle nh;
    std::cout<<"[genetic_algorithm_Test] genetic_algorithm_test_node started.."<<std::endl;
    ros::AsyncSpinner spinner(0); // Use all threads
    spinner.start();
    // std::unique_ptr<genetic_algorithm> ga (new genetic_algorithm(nh));
    UnitTest ua;


    // ros::waitForShutdown();
    // return 0;
    // ga.test_function();

    return 0;

};