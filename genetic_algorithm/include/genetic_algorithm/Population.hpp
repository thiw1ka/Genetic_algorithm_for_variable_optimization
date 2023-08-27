#include <vector>


#include "AncientHuman.hpp"
#include "cyanob_phd_filter_sabbatus/bloom_phd_filter_readfile.hpp"
#include "intersection_over_union/intersection_over_union.hpp"
#include <experimental/filesystem>
#include <fstream>
// #include "genetic_algorithm/genetic_algorithm.hpp"

#ifndef POPULATION_HPP
#define POPULATION_HPP
class genetic_algorithm;


struct Population {

    Population();

    Population (int size);

    ~Population();

    // variables 
    /*hold the county*/
    std::vector <AncientHuman> residents_list;

    //functions

    /**
     * @brief replace the residents from another population
     * 
     * @param newResidents 
     */
    void ReplaceResidents (std::vector <AncientHuman>& newResidents);

    /**
     * @brief sort population in decending order of fittness
     * 
     */
    void sortDecendingOrderOfHumanFitness();

    /**
     * @brief print population to the screen
     * 
     */
    void print() const;

    /**
     * @brief Get population size
     * 
     * @return int 
     */
    int getSize ();


    /**overloading operator for copy*/
    Population operator= (Population& other);

    /*plus operator*/
    Population operator+ (Population& other);

    /*index modifications*/
    void incrementIndex();

    int get_index();

    void set_index(int ind);

    /*update population fitness*/
    void updatePopulationFitness();

    /*hihgets individual from the population*/
    double getHighestFitnessIndividual ();

    /**
     * @brief clearing existing residents to populate new
     * 
     * @param size 
     */
    void reset_population (int size);

    /**
     * @brief calculate fitness for each indivdual
     * 
     */
    void calculateFitness(std::string groundPath, std::string result);

    enum STATE_TO_SAVE {
        NEWPOPULATION = 0,
        RECOMBINE =1,
        MUTATION = 2,
        EVAL = 3
    };

    /*population saves here*/
    /**
     * @brief saving the population file. eg: population-0-eval.txt
     * 
     * @param directory_Path 
     * @param saving_state eval, new_population, recombine, mutation
     */
    void save_poupulation_file(std::string directory_Path, STATE_TO_SAVE ss);

    private:

        /**
         * @brief population number from the begining
         * 
         */
        static int index;

        double highest_fitness_ = 0.0;

        /*sum of all finess of residents*/
        double population_total_fitness_ = 0.0;


        /**
         * @brief create a size of population using ancient humans
         * 
         * @param size 
         */
        void setPopulation (int size);

        /*summing all the fitness values of population*/ ///TODO should each human fitness should be devided by the sum of the population?
        // int getTotalFitnessOfAllResidents();



};

#endif