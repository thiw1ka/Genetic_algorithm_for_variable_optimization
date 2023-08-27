// #include "genetic_algorithm/genetic_algorithm.hpp"
#include <iostream>
#include "ros/ros.h"
#include "genetic_algorithm/genetic_algorithm.hpp"
#include "cyanob_phd_filter_sabbatus/bloom_phd_filter_readfile.hpp"
#ifndef UNIT_TEST_HPP
#define UNIT_TEST_HPP

// class genetic_algorithm;

// class node {
//     public:
//     node () {
//         ros::M_string remappings = {};
//         ros::init(remappings,"testing");
//         ros::NodeHandle nh;
//         cout<< "node : " << ros::this_node::getName();
//     };
// };

struct UnitTest {
    UnitTest();

    void test_seperate_nodes();

     void test_population_class ();
};



#endif