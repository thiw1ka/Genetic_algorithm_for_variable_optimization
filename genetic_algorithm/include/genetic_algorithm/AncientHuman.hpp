#include <iostream>
#include <array>
#include <algorithm>
#include <tuple>
#include <vector>

#ifndef ANCIENTHUMAN_HPP
#define ANCIENTHUMAN_HPP


/**
 * @brief Contains the variable list for optimization
 * 
 */
struct AncientHuman {
        AncientHuman() {};
        /**
         * @brief initial data container with default values
         * use this to create copies of chromosomes
         * 
         */
        std::array <std::pair<std::string, double>, 5> chromosome = {
                {{"tempreture", 0.0},
                {"ps", 0.0},
                {"pd", 0.0},
                {"wind_speed", 0.0},
                {"wind_heading", 0.0}}
        };

        /**
         * @brief setting specific value pair
         * 
         * @param name 
         * @param val 
         */
        // void setAChoromosome (std::string name,  auto val);

        /**for equal operator*/
        // AncientHuman operator= (AncientHuman other);

        /*fitness value*/
        double fitness = 0;

        std::vector<std::tuple<std::string, double> > getVariablesInTuple();

        /*to pass into a launch file*/
        std::string getVariableInString ();

        /**
         * @brief saving to file
         * 
         * @return std::string tempreture,0.000000,ps,0.000000,pd,0.000000,wind_speed,0.000000,wind_heading,0.000000,fitness,0.560769
         */
        std::string getVariableToSave ();

        friend void print_chromos(const AncientHuman& g); 
};

void print_chromosomes(const AncientHuman& g);



#endif