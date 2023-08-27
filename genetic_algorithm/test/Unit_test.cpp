#include "../test/Unit_test.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

UnitTest::UnitTest() {
    ros::NodeHandle nh;
    genetic_algorithm ga(nh);
    ga.test_function();
    ga.~genetic_algorithm();
    exit(0);
    // // test_seperate_nodes();
    // test_population_class();
    
}

void UnitTest::test_population_class () {
    std::printf("[test_population_class] starting \n");
    bloom_filter::readfile file_Object;
    std::string path = "/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/results/filter_output/";
    // file_Object.setDirPath(path);
    // auto list = file_Object.GetListOfAllFilesInDirectory ();
    // file_Object.getFilesClusteredIntoFilterTimeSteps(list);
    // file_Object.SetPopulationFiles(path);
    std::string gndtruth = "/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/Data_files/2021_simulation/1.groundtruth_blocations/bloom_coordinates_northeast_wind_drift/";
    // bloom_filter::readfile ground_truth;
    // ground_truth.updateGroundTruthFiles(gndtruth);
    // auto GTRfileCount = ground_truth.get_filter_timestep_count();
    // std::cout << "count "<< GTRfileCount <<std::endl;
    Population p1;
    p1.reset_population (5);
    p1.calculateFitness(gndtruth, path);
}


/*sample test that I ran to check if ros node can be created within anothre. Answer is NO*/
class node {
    // ros::NodeHandle nh;
    void chatterCallback(const std_msgs::String::ConstPtr& msg) {
                ROS_INFO("I heard: [%s]", msg->data.c_str());
    }
    public:
    node (string a,ros::NodeHandle nn) /*: nh(nn, a)*/ {
        std::printf("node started () \n");
        // ros::M_string remappings = {};
        std::string sub_topic = "chatter";
        ros::VP_string remappings;
        remappings.push_back(std::make_pair("chatter", sub_topic));
        ros::init(remappings,"testing_"+a, ros::init_options::NoSigintHandler);
        ros::NodeHandle nh(a);
        cout<< "node : " << ros::this_node::getName() << endl;
        ros::AsyncSpinner spinner(0);
        spinner.start();
        ros::Subscriber sub = nh.subscribe(sub_topic, 1000, &node::chatterCallback, this);
        // ros::spinOnce();
        ros::waitForShutdown();
        // while(ros::ok());
    };
};
/*sample test that I ran to check if ros node can be created within anothre. Answer is NO*/
void UnitTest::test_seperate_nodes () {
    std::printf("test_seperate_nodes () \n");
    ros::M_string remappings = {};
    ros::init(remappings,"testing_", ros::init_options::AnonymousName);
    ros::NodeHandle nnn;
    ros::AsyncSpinner spinner(0);
    spinner.start();  
    auto pop = AncientHuman();
    auto var_tuple = pop.getVariablesInTuple();
    std::promise <int> prom;
    auto prom_fut = prom.get_future(); 
    // bloom_filter::parallel_filter pf;
    // node n();
    // void n = [](string b){node(b);};
    ros::Publisher chatter_pub = nnn.advertise<std_msgs::String>("/chatter", 1000);
    auto ret_fut = std::async(std::launch::async,[&]{node("/_1_", nnn);});
    ret_fut.wait_for(std::chrono::seconds(5));
    auto ret_fut2 = std::async(std::launch::async, [&]{node("/_2_",nnn);});
    ret_fut.wait_for(std::chrono::seconds(5));
    ret_fut.get();     
    ret_fut2.get();
    // auto ret_fut = std::async(std::launch::async,  
    //                                             [&] {bloom_filter::parallel_filter("test_node", var_tuple, /*std::move(f_promise)*/std::move(prom));}
    //                                           );
    // std::printf("while wait for ros ok \n");

    // ros::M_string remappings = {};
    // ros::init(remappings,"testing");
    // while(ros::ok());
    ros::requestShutdown();

    // pf.set_exiting_condition_true();
    // prom_fut.get();
}