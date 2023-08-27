#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

#include <cstdlib>
#include <ros/ros.h>
#include <ros/console.h>
#include "diagnostic_msgs/DiagnosticStatus.h"

#include "AncientHuman.hpp"
#include "bag_player/bag_player.hpp"
// #include "intersection_over_union/intersection_over_union.hpp"
#include "parallel_filter/parallel_filter.hpp"
#include "Population.hpp"


// #include "cyanob_phd_filter_sabbatus/bloom_phd_filter.hpp"

#ifndef GENETIC_ALGORITHM_HPP
#define GENETIC_ALGORITHM_HPP

// #include "../test/Unit_test.hpp"


class genetic_algorithm /*: IntersectionOverUnion*/ {
            
    public:
        /**
         * @brief testing the functions
         * 
         */
        void test_function ();

        genetic_algorithm() = delete; //No default constructor is allowed

        genetic_algorithm(ros::NodeHandle n);
        
        ~genetic_algorithm();

    private:
        friend class UnitTest;

        int Population_size;
        // friend class Population;

        /**
         * @brief top percentage% from population will be used to create children
         * default 10
         */
        int top_selection = 10;

        /**
         * @brief mutating percentage from the population
         * 
         */
        int mutating_percentage = 10;

        /**
         * @brief crossover percentage from the population
         * 
         */
        int crossover_percentage = 10;

        /**
         * @brief execution speed
         * 
         */
        int running_speed = 10;

        std::string ns; // namespace from the launchfile 

    protected:
        ros::NodeHandle n_;

        /**
         * @brief initial data container with default values
         * use this to create copies of chromosomes
         * 
         */
        const std::array <std::pair<std::string, double>, 5> first_chromosome = {
                {{"tempreture", 0.0},
                {"ps", 0.0},
                {"pd", 0.0},
                {"wind_speed", 0.0},
                {"wind_heading", 0.0}}
        };

        /**
         * @brief creating data type based on choromosome array
         * 
         */
        using chrom_dtype = std::remove_const<decltype(first_chromosome)>::type;

        /*print chromosome to the screen*/
        void print_chromos(chrom_dtype c) const;

        /**
         * @brief class to generate random numbers
         * 
         */
        struct RandomNumberGenerator {
            std::random_device rd_; // a seed source for the random number engine
            std::mt19937* gen_; // mersenne_twister_engine seeded with rd()
            std::uniform_int_distribution <>* int_dist_;
            std::uniform_real_distribution <>* double_dist_;
            double getRandDoubleValue ();
            int getRandIntValue ();
            RandomNumberGenerator ();
            ~RandomNumberGenerator ();
        };

        /**
         * @brief random number generator to select numbr from chromosome
         * 
         */
        RandomNumberGenerator rand_for_selecting_element, rand_for_create_child;

        /**
         * @brief holds the probability distributions for each variables
         * 
         */
        std::map <std::string, std::unique_ptr< RandomNumberGenerator> > map_of_random_generators;

        /**
         * @brief list of variables and it's upper and lower bounds
         * 
         */
        std::unordered_map <std::string, std::pair<double, double> > variable_boundaries = {
            {"tempreture", {20.0, 35.0}},
            {"wind_heading", {0.0,360.0}},
            {"wind_speed", {0.0,15.0}},
            {"pd", {0.0, 1.0}},
            {"ps", {0.0, 1.0}}
        };

        /**
         * @brief values for the variables
         * 
         *
        */
       void CreateHuman (AncientHuman& g);

        /**
         * @brief apply mutations to g
         * we randomly resample a variable from the gene
         * @param g 
         */
        void getMutedHuman (AncientHuman& g);

        /**
         * @brief cross over operation basically recombine selected person's chromosomes.
         * basically interchange the values.
         * 
         */
        void getCrossoverOperation (AncientHuman& a, AncientHuman& b);

        /**
         * @brief create a population
         * 
         */
        void PopulatePopulation (Population& p);

        /**
         * @brief mutate percentage of population
         * 
         * @param p : population
         * @param percentage : percentage (1 : 100)
         */
        void getMutatantPopulation (Population& p, int percentage);

        /**
         * @brief do crossover chromosome swap in the population
         * 
         * @param p 
         * @param percentage : percentage (1 : <100)
         */
        void getCrossOverPopulation (Population& p, int percentage);

        /**
         * @brief create new population from previous one.
         * pass fittest from previous and create children
         * 
         * @param p population to breed
         * @param Fit_percentage fit percentage to be passed to next population
         * @return Population 
         */
        void getNewBornChildrenFromPopulation (Population& p, int Fit_percentage);

        /*population*/
        Population p1;

        std::unique_ptr <Population> population_ptr;
        Population baberpond_population;


        /**
         * @brief create a child from parents
         * 
         * @param a 
         * @param b 
         * @return AncientHuman 
         */
        AncientHuman getCreateChild (AncientHuman& a, AncientHuman& b);

        /**
         * @brief This is the order of genertic algo performing.
         * 
         */
        enum STATES {
            INIT_ = 0,
            EVAL_ = 1,
            INTER_POPULATION_ = 2,
            RECOMBINE_ = 3,
            MUTATION_ = 4,
            ERROR_ = 5,
            GOAL_REACHED_ = 6
        };

        STATES current_state_ = STATES::INIT_;

        /**
         * @brief execute states
         * 
         */
        void run_states (void);

        /**
         * @brief calculate fitness of each individual inside the population
         * 
         * @param p 
         */
        void evaluatePopulation (Population& p);

        /**
         * @brief this is to set the static variables inside the parallel filter
         * 
         */
        // bloom_filter::parallel_filter pf_static_member_setter; //CANCELLED because filters are running in different processes. No shared memories

        /**/
        std::unique_ptr<bag_player> bagPlayerPtr;

        /**
         * @brief these are to inform filter sub process to shutdown because rosbag is finished.
         * 
         */
        ros::Publisher filter_process_shutdown_pub_;
        void PassFilterShutDownSignal();

        void calculateFitness (Population& p);

        /*folder paths*/
        std::string GROUND_TRUTH_DIR_ ="";
        std::string RESULT_DIR_ ="";
        std::string ROSBAG_DIR_ = "";



};

#endif